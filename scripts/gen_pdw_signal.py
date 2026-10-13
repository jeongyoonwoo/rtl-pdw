#!/usr/bin/env python3
"""
gen_pdw_signal.py  --  rtl-pdw 합성 신호, 정답표, 비트 단위 모델 생성

시나리오 3종(S1 단일 방출원, S2 방출원 3개 + 3샘플 선형 가장자리, S3 경계 조건)에 대해
  iq_rom_sN.hex       ROM 초기값 (32비트 = I[15:0] | Q[15:0], Q1.15), 8,192샘플
  cordic_ref_sN.hex   샘플별 CORDIC 크기·위상 (RTL과 같은 정수 연산을 비트 단위로 재현)
  expected_pdw_sN.hex 정답표 PDW (TOA, PW, PA, FREQ, FLAGS)
  model_pdw_sN.hex    비트 단위 모델이 검출한 PDW (RTL이 이것과 차이 0이어야 함)
를 만든다. 난수 시드가 고정이라 매번 같은 파일이 나온다.

사용법: python3 scripts/gen_pdw_signal.py sim_data
"""
import csv, math, os, sys
from dataclasses import dataclass
from typing import List, Optional
import numpy as np

FS = 1.0e6; N = 8192; SIGMA = 0.004; K = 1.646760258121066
TH_HI = 3000; TH_LO = 2000; MIN_W = 4; END_LOW = 2
FREQ_SKIP = 3; FREQ_WIN = 8; MIN_FREQ_W = FREQ_SKIP + FREQ_WIN
GUARD = 5; EDGE_R = 3; SEED = 20261006

LUT = [0x2000, 0x12E4, 0x09FB, 0x0511, 0x028B, 0x0146, 0x00A3, 0x0051,
       0x0029, 0x0014, 0x000A, 0x0005, 0x0003, 0x0001, 0x0001, 0x0000]


@dataclass
class Pulse:
    pid: int; emitter: str; toa: int; pw: int; amp: float; f_hz: float; phase: float
    expect: str = "detect"; dip_at: Optional[int] = None; edge: int = 0; note: str = ""


def w16(v): return ((int(v) + 32768) & 0xFFFF) - 32768
def w18(v): return ((int(v) + 131072) & 0x3FFFF) - 131072


def cordic_vec(xi, yi):
    if xi < 0:
        x, y, z = -xi, -yi, (-32768 if yi < 0 else 32767)
    else:
        x, y, z = xi, yi, 0
    for i in range(16):
        xs, ys = x >> i, y >> i
        if y < 0:
            x, y, z = w18(x - ys), w18(y + xs), w16(z - LUT[i])
        else:
            x, y, z = w18(x + ys), w18(y - xs), w16(z + LUT[i])
    xo = 32767 if x > 32767 else (-32768 if x < -32768 else x)
    return xo, z


def extent(p):
    if p.edge:
        return p.toa - 1, p.toa + p.pw + 2
    return p.toa, p.toa + p.pw


def scenario_s1(rng):
    pulses, k, toa = [], 0, 200
    while toa + 12 + 60 < N:
        pulses.append(Pulse(k, "A", toa, 12, 0.40, +100e3, rng.uniform(-math.pi, math.pi)))
        toa += 100; k += 1
    return pulses


def scenario_s2(rng):
    emitters = [("A", +100e3, 12, 0.40, 97, 150), ("B", -200e3, 25, 0.25, 251, 230),
                ("C", +350e3, 40, 0.55, 401, 330)]
    cand = []
    for name, f, pw, amp, pri, off in emitters:
        toa = off
        while toa + pw + 60 < N:
            cand.append((toa, name, f, pw, amp)); toa += pri
    cand.sort()
    kept, dropped, prev_hi = [], {"A": 0, "B": 0, "C": 0}, -10**9
    for toa, name, f, pw, amp in cand:
        probe = Pulse(0, name, toa, pw, amp, f, 0.0, edge=EDGE_R)
        lo, hi = extent(probe)
        if lo - prev_hi >= GUARD:
            kept.append(probe); prev_hi = hi
        else:
            dropped[name] += 1
    pulses = [Pulse(i, p.emitter, p.toa, p.pw, p.amp, p.f_hz, rng.uniform(-math.pi, math.pi), edge=EDGE_R)
              for i, p in enumerate(kept)]
    return pulses, dropped


def scenario_s3(rng):
    spec = [(4, 0.30, +150e3, "detect", None), (3, 0.30, +150e3, "reject", None),
            (20, 0.07, -120e3, "detect", None), (15, 0.65, +50e3, "detect", None),
            (20, 0.35, +80e3, "detect", 14), (10, 0.35, -250e3, "detect", None),
            (11, 0.35, +200e3, "detect", None), (16, 0.35, +450e3, "detect", None)]
    pulses, toa = [], 300
    for i, (pw, amp, f, ex, dip) in enumerate(spec):
        pulses.append(Pulse(i, "X", toa, pw, amp, f, rng.uniform(-math.pi, math.pi), ex, dip, 0))
        toa += 400
    pulses.append(Pulse(len(pulses), "X", toa, 12, 0.40, +100e3, rng.uniform(-math.pi, math.pi)))
    pulses.append(Pulse(len(pulses), "X", toa + 12 + GUARD, 12, 0.40, -100e3, rng.uniform(-math.pi, math.pi)))
    return pulses


def envelope(p, n):
    k = n - p.toa
    if p.edge == 0:
        return 1.0 if 0 <= k < p.pw else 0.0
    rise = min(1.0, max(0.0, 0.5 + k / p.edge))
    fall = min(1.0, max(0.0, 0.5 - (k - p.pw) / p.edge))
    return rise * fall


def synthesize(pulses, rng):
    s = np.zeros(N, dtype=np.complex128)
    for p in pulses:
        lo, hi = extent(p)
        for n in range(lo, hi):
            e = envelope(p, n)
            if e <= 0.0:
                continue
            a = p.amp * e
            if p.dip_at is not None and (n - p.toa) == p.dip_at:
                a = 0.01
            s[n] += a * np.exp(1j * (2 * math.pi * p.f_hz * (n - p.toa) / FS + p.phase))
    s += SIGMA * (rng.standard_normal(N) + 1j * rng.standard_normal(N))
    I = np.round(s.real * 32768.0).astype(np.int64)
    Q = np.round(s.imag * 32768.0).astype(np.int64)
    return np.clip(I, -32768, 32767), np.clip(Q, -32768, 32767)


def truth_fields(p):
    pa = min(int(round(p.amp * K * 32768.0)), 32767)
    sat = 1 if p.amp * K >= 1.0 else 0
    fr = max(-32768, min(32767, int(round(p.f_hz / (FS / 2.0) * 32768.0))))
    fvalid = 1 if p.pw >= MIN_FREQ_W else 0
    return pa, fr, fvalid, sat, fvalid | (sat << 1)


def pdw_hex(toa, pw, pa, freq, flags):
    return "%08X%04X%04X%04X%04X" % (toa & 0xFFFFFFFF, pw & 0xFFFF, pa & 0xFFFF, freq & 0xFFFF, flags & 0xFFFF)


def run_model(I, Q):
    cm = np.zeros(N, dtype=np.int64); cz = np.zeros(N, dtype=np.int64)
    for n in range(N):
        cm[n], cz[n] = cordic_vec(int(I[n]), int(Q[n]))
    out, state = [], 0
    toa = last_hi = peak = low = fsum = msum = 0
    for n in range(N):
        m = int(cm[n])
        dphi = w16(cz[n] - cz[n - 1]) if n > 0 else 0
        if state == 0:
            if m >= TH_HI:
                state, toa, last_hi, peak, low, fsum, msum = 1, n, n, m, 0, 0, 0
        else:
            r = n - toa
            if m >= TH_LO:
                last_hi, low = n, 0; peak = max(peak, m)
            else:
                low += 1
            if FREQ_SKIP <= r < FREQ_SKIP + FREQ_WIN:
                fsum += dphi; msum += m
            if low >= END_LOW:
                pw = last_hi - toa + 1
                if pw >= MIN_W:
                    fvalid = 1 if pw >= MIN_FREQ_W else 0
                    out.append(dict(toa=toa, pw=pw, pa=(msum >> 3) if fvalid else peak,
                                    freq=(fsum >> 3) if fvalid else 0,
                                    flags=fvalid | ((1 if peak >= 32767 else 0) << 1)))
                state = 0
    return out, cm, cz


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else "sim_data"
    os.makedirs(outdir, exist_ok=True)
    rng = np.random.default_rng(SEED)
    s1 = scenario_s1(rng); s2, _ = scenario_s2(rng); s3 = scenario_s3(rng)
    for tag, pulses in (("s1", s1), ("s2", s2), ("s3", s3)):
        I, Q = synthesize(pulses, rng)
        dets, cm, cz = run_model(I, Q)
        with open(os.path.join(outdir, "iq_rom_%s.hex" % tag), "w") as f:
            for i, q in zip(I, Q):
                f.write("%04X%04X\n" % (int(i) & 0xFFFF, int(q) & 0xFFFF))
        with open(os.path.join(outdir, "cordic_ref_%s.hex" % tag), "w") as f:
            for m, z in zip(cm, cz):
                f.write("%04X%04X\n" % (int(m) & 0xFFFF, int(z) & 0xFFFF))
        n_exp = 0
        with open(os.path.join(outdir, "expected_pdw_%s.hex" % tag), "w") as f:
            for p in pulses:
                if p.expect != "detect":
                    continue
                pa, fr, fv, sat, flags = truth_fields(p)
                f.write(pdw_hex(p.toa, p.pw, pa, fr if fv else 0, flags) + "\n"); n_exp += 1
        with open(os.path.join(outdir, "model_pdw_%s.hex" % tag), "w") as f:
            for d in dets:
                f.write(pdw_hex(d["toa"], d["pw"], d["pa"], d["freq"], d["flags"]) + "\n")
        print(tag, "expected", n_exp, "model", len(dets))


if __name__ == "__main__":
    main()
