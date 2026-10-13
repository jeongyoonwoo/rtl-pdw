#!/usr/bin/env python3
"""
gen_frames.py  --  rtl-pdw 5단계: 기대 텔레메트리 프레임 생성

model_pdw_sN.hex(비트 단위 모델 PDW, 96비트/줄)를 읽어 프레임 하나에 PDW 하나를 담은
21바이트 프레임을 만들고 model_frame_sN.hex(42자리 16진/줄)로 쓴다.

프레임 (big-endian)
  ASM 4 | FRAME_CNT 2 | N_PDW 1 | TOA 4 | PW 2 | PA 2 | FREQ 2 | FLAGS 2 | CRC 2
  CRC-16-CCITT: 다항식 0x1021, 초기값 0xFFFF, 반사 없음, 최종 XOR 없음
                (CRC-16/CCITT-FALSE, "123456789"의 검사값 0x29B1)
  CRC 범위: FRAME_CNT부터 FLAGS까지 15바이트

사용법: python3 gen_frames.py [sim_data]
"""
import os
import sys

ASM = [0x1A, 0xCF, 0xFC, 0x1D]
FRAME_LEN = 21


def crc16_ccitt(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(cnt, pdw_hex):
    v = int(pdw_hex, 16)
    toa, pw, pa = (v >> 64) & 0xFFFFFFFF, (v >> 48) & 0xFFFF, (v >> 32) & 0xFFFF
    freq, flags = (v >> 16) & 0xFFFF, v & 0xFFFF
    body = [(cnt >> 8) & 0xFF, cnt & 0xFF, 0x01,
            (toa >> 24) & 0xFF, (toa >> 16) & 0xFF, (toa >> 8) & 0xFF, toa & 0xFF,
            pw >> 8, pw & 0xFF, pa >> 8, pa & 0xFF, freq >> 8, freq & 0xFF, flags >> 8, flags & 0xFF]
    crc = crc16_ccitt(body)
    frame = ASM + body + [crc >> 8, crc & 0xFF]
    assert len(frame) == FRAME_LEN
    return frame


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "sim_data"
    chk = crc16_ccitt(b"123456789")
    assert chk == 0x29B1, "CRC 구현 오류: 0x%04X" % chk
    print("CRC-16/CCITT-FALSE 검사값 확인: 0x%04X" % chk)
    for tag in ("s1", "s2", "s3"):
        pdws = [l.strip() for l in open(os.path.join(d, "model_pdw_%s.hex" % tag)) if l.strip()]
        with open(os.path.join(d, "model_frame_%s.hex" % tag), "w") as f:
            for i, p in enumerate(pdws):
                f.write("".join("%02X" % b for b in build_frame(i & 0xFFFF, p)) + "\n")
        print("%s: 프레임 %d개 -> model_frame_%s.hex" % (tag, len(pdws), tag))


if __name__ == "__main__":
    main()
