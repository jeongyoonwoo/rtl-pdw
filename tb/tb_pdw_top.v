//==============================================================================
// tb_pdw_top.v   (rtl-pdw 6단계 통합 테스트벤치)
// pdw_top 3개(S1~S3)를 동시에 돌리고, 출력 바이트 스트림만 보고 프레임을 해독해
// 정답표(expected_pdw_sN.hex)와 비교한다. 계획서 4장의 판정 기준을 그대로 적용한다.
//
// 프레임 검사 : ASM, FRAME_CNT 순번, 길이 21, CRC 재계산
// 짝 짓기     : 정답 TOA와 검출 TOA 차이가 3샘플 이내면 같은 펄스 (둘 다 TOA 순서)
// 판정 기준
//   TOA   : |오차| <= 1샘플
//   PW    : S1·S3 -1~+1샘플, S2(3샘플 가장자리) -2~+3샘플
//   PA    : 포화 펄스 제외. 주파수 유효(PW >= 11) 펄스는 +-(1% + 230 LSB),
//           짧은 펄스는 +-(2% + 650 LSB)
//   FREQ  : 주파수 유효 & 진폭 >= 0.25(정답 PA >= 13490) 펄스만, +-3kHz
//   FLAGS : 정답과 같아야 함
//   검출률 100%, 오검출 0, CRC 통과 100%, pdw_drop 0, overrun 0
// 오차 = 검출 - 정답. TOA/PW는 범위와 평균(편향), PA/FREQ는 절댓값 최대/평균을 낸다.
//
// READY_MODE: 0 = ready 항상 1, 1 = 무작위 약 50%, 2 = 약 1/8
// 실행 위치: rtl-pdw 폴더(sim_data가 보이는 곳)
//==============================================================================
`timescale 1ns/1ps

module ready_gen_top #(
    parameter MODE = 0,
    parameter [15:0] SEED = 16'hACE1
) (
    input  wire clk,
    input  wire rst_n,
    output wire ready
);
    reg [15:0] lfsr;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) lfsr <= SEED;
        else        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
    assign ready = (MODE == 0) ? 1'b1 : (MODE == 1) ? lfsr[0] : (lfsr[2:0] == 3'b000);
endmodule

//------------------------------------------------------------------------------
// 정확도 검사기 (시나리오 하나)
//------------------------------------------------------------------------------
module acc_chk #(
    parameter EXP_FILE = "sim_data/expected_pdw_s1.hex",
    parameter N_EXP    = 80,
    parameter EDGE     = 0          // 1이면 PW 기준 -2~+3 (S2)
) (
    input wire       clk,
    input wire [7:0] m_data,
    input wire       m_valid,
    input wire       m_ready,
    input wire       m_last
);
    reg [95:0] exp_mem [0:N_EXP-1];
    initial $readmemh(EXP_FILE, exp_mem);

    function [15:0] crc16_byte;
        input [15:0] crc_in;
        input [7:0]  data;
        integer      k;
        reg   [15:0] c;
        begin
            c = crc_in ^ {data, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_byte = c;
        end
    endfunction

    // 프레임 수신
    reg [7:0] fb [0:20];
    integer   pos = 0;

    // 프레임 단위 집계
    integer n_frames = 0, e_crc = 0, e_asm = 0, e_cnt = 0, e_len = 0;
    // 짝 짓기 집계
    integer ptr = 0, n_match = 0, n_miss = 0, n_fa = 0;
    // 기준 초과 건수
    integer b_toa = 0, b_pw = 0, b_pa = 0, b_fr = 0, b_fl = 0;
    // 오차 통계
    integer toa_min = 1000, toa_max = -1000, pw_min = 1000, pw_max = -1000;
    integer toa_sum = 0, pw_sum = 0;
    integer pa_max = 0, pa_sum = 0, pa_n = 0;
    real    fr_max = 0.0, fr_sum = 0.0;
    integer fr_n = 0;

    // 작업 변수
    reg [31:0] toa, t_toa;
    reg [15:0] pw, pa, freq, flags, t_pw, t_pa, t_freq, t_flags, c;
    integer    i, dt, dw, dp, lo, hi;
    real       dk, tol;
    reg        matched;

    always @(posedge clk) begin
        if (m_valid && m_ready) begin
            if (pos <= 20) fb[pos] = m_data;
            if (!m_last) begin
                if (pos >= 20) e_len = e_len + 1;
                pos = pos + 1;
            end else begin
                if (pos != 20) e_len = e_len + 1;
                pos = 0;

                // ---- 프레임 검사 ----
                if ({fb[0], fb[1], fb[2], fb[3]} !== 32'h1ACFFC1D) e_asm = e_asm + 1;
                if ({fb[4], fb[5]} !== n_frames[15:0])              e_cnt = e_cnt + 1;
                c = 16'hFFFF;
                for (i = 4; i <= 18; i = i + 1) c = crc16_byte(c, fb[i]);
                if ({fb[19], fb[20]} !== c) e_crc = e_crc + 1;
                n_frames = n_frames + 1;

                // ---- 해독 ----
                toa   = {fb[7], fb[8], fb[9], fb[10]};
                pw    = {fb[11], fb[12]};
                pa    = {fb[13], fb[14]};
                freq  = {fb[15], fb[16]};
                flags = {fb[17], fb[18]};

                // ---- 정답과 짝 짓기 (정답 TOA가 검출보다 3샘플 넘게 앞서면 놓친 펄스) ----
                while ((ptr < N_EXP) && (exp_mem[ptr][95:64] + 3 < toa)) begin
                    n_miss = n_miss + 1;
                    ptr = ptr + 1;
                end
                matched = 0;
                if (ptr < N_EXP) begin
                    dt = $signed(toa) - $signed(exp_mem[ptr][95:64]);
                    if (dt >= -3 && dt <= 3) matched = 1;
                end

                if (!matched) begin
                    n_fa = n_fa + 1;
                end else begin
                    t_toa   = exp_mem[ptr][95:64];
                    t_pw    = exp_mem[ptr][63:48];
                    t_pa    = exp_mem[ptr][47:32];
                    t_freq  = exp_mem[ptr][31:16];
                    t_flags = exp_mem[ptr][15:0];
                    ptr = ptr + 1;
                    n_match = n_match + 1;

                    // TOA
                    if (dt < toa_min) toa_min = dt;
                    if (dt > toa_max) toa_max = dt;
                    toa_sum = toa_sum + dt;
                    if (dt < -1 || dt > 1) b_toa = b_toa + 1;

                    // PW
                    dw = $signed({16'd0, pw}) - $signed({16'd0, t_pw});
                    if (dw < pw_min) pw_min = dw;
                    if (dw > pw_max) pw_max = dw;
                    pw_sum = pw_sum + dw;
                    if (EDGE) begin lo = -2; hi = 3; end else begin lo = -1; hi = 1; end
                    if (dw < lo || dw > hi) b_pw = b_pw + 1;

                    // PA (포화 펄스 제외)
                    if (!t_flags[1]) begin
                        dp = $signed({16'd0, pa}) - $signed({16'd0, t_pa});
                        if (dp < 0) dp = -dp;
                        if (dp > pa_max) pa_max = dp;
                        pa_sum = pa_sum + dp;
                        pa_n = pa_n + 1;
                        if (t_flags[0]) tol = 0.01 * t_pa + 230.0;
                        else            tol = 0.02 * t_pa + 650.0;
                        if (dp > tol) b_pa = b_pa + 1;
                    end

                    // FLAGS
                    if (flags !== t_flags) b_fl = b_fl + 1;

                    // FREQ (주파수 유효, 진폭 >= 0.25)
                    if (t_flags[0] && (t_pa >= 16'd13490)) begin
                        dk = ($signed(freq) - $signed(t_freq)) / 32768.0 * 500.0;   // kHz
                        if (dk < 0.0) dk = -dk;
                        if (dk > fr_max) fr_max = dk;
                        fr_sum = fr_sum + dk;
                        fr_n = fr_n + 1;
                        if (dk > 3.0) b_fr = b_fr + 1;
                    end
                end
            end
        end
    end

    // 결과 출력 (시뮬레이션 끝에서 상위 모듈이 호출)
    task print_report;
        input  [8*2-1:0] name;
        input  [15:0]    drop;
        input            ovr;
        output           ok;
        integer          miss_all;
        begin
            miss_all = n_miss + (N_EXP - ptr);
            ok = (n_match == N_EXP) && (miss_all == 0) && (n_fa == 0)
                 && (e_crc == 0) && (e_asm == 0) && (e_cnt == 0) && (e_len == 0)
                 && (b_toa == 0) && (b_pw == 0) && (b_pa == 0) && (b_fr == 0) && (b_fl == 0)
                 && (drop == 0) && !ovr;
            $display("[%s] %s  detected %0d / truth %0d, missed %0d, false alarm %0d",
                     ok ? "PASS" : "FAIL", name, n_match, N_EXP, miss_all, n_fa);
            $display("     frames %0d: crc_fail %0d, asm_err %0d, cnt_err %0d, len_err %0d, pdw_drop %0d, overrun %0d",
                     n_frames, e_crc, e_asm, e_cnt, e_len, drop, ovr);
            $display("     TOA  err %0d ~ %0d samples, mean %0.3f            (limit +-1)           over %0d",
                     toa_min, toa_max, (n_match ? 1.0 * toa_sum / n_match : 0.0), b_toa);
            $display("     PW   err %0d ~ %0d samples, mean %0.3f            (limit %s)   over %0d",
                     pw_min, pw_max, (n_match ? 1.0 * pw_sum / n_match : 0.0),
                     EDGE ? "-2~+3" : "-1~+1", b_pw);
            $display("     PA   |err| max %0d / mean %0.1f LSB (n=%0d, saturated excluded)  over %0d",
                     pa_max, (pa_n ? 1.0 * pa_sum / pa_n : 0.0), pa_n, b_pa);
            $display("     FREQ |err| max %0.3f / mean %0.3f kHz (n=%0d)  (limit 3 kHz)  over %0d",
                     fr_max, (fr_n ? fr_sum / fr_n : 0.0), fr_n, b_fr);
            $display("     FLAGS mismatch %0d", b_fl);
        end
    endtask
endmodule

//------------------------------------------------------------------------------
// 테스트벤치 본체
//------------------------------------------------------------------------------
module tb_pdw_top;
    parameter READY_MODE = 0;

    reg clk = 1'b0, rst_n = 1'b0, run = 1'b0;
    always #10 clk = ~clk;      // 50MHz

    wire        r1, r2, r3;
    wire [7:0]  d1, d2, d3;
    wire        v1, v2, v3, l1, l2, l3, rd1, rd2, rd3, ov1, ov2, ov3;
    wire [15:0] gl1, gl2, gl3, fc1, fc2, fc3, dr1, dr2, dr3;

    ready_gen_top #(.MODE(READY_MODE), .SEED(16'hACE1)) rg1 (.clk(clk), .rst_n(rst_n), .ready(r1));
    ready_gen_top #(.MODE(READY_MODE), .SEED(16'h1D2B)) rg2 (.clk(clk), .rst_n(rst_n), .ready(r2));
    ready_gen_top #(.MODE(READY_MODE), .SEED(16'h7F31)) rg3 (.clk(clk), .rst_n(rst_n), .ready(r3));

    pdw_top #(.ROM_FILE("sim_data/iq_rom_s1.hex")) dut1 (
        .clk(clk), .rst_n(rst_n), .run(run), .m_data(d1), .m_valid(v1), .m_ready(r1), .m_last(l1),
        .rom_done(rd1), .overrun(ov1), .glitch_cnt(gl1), .frame_cnt(fc1), .pdw_drop(dr1));
    pdw_top #(.ROM_FILE("sim_data/iq_rom_s2.hex")) dut2 (
        .clk(clk), .rst_n(rst_n), .run(run), .m_data(d2), .m_valid(v2), .m_ready(r2), .m_last(l2),
        .rom_done(rd2), .overrun(ov2), .glitch_cnt(gl2), .frame_cnt(fc2), .pdw_drop(dr2));
    pdw_top #(.ROM_FILE("sim_data/iq_rom_s3.hex")) dut3 (
        .clk(clk), .rst_n(rst_n), .run(run), .m_data(d3), .m_valid(v3), .m_ready(r3), .m_last(l3),
        .rom_done(rd3), .overrun(ov3), .glitch_cnt(gl3), .frame_cnt(fc3), .pdw_drop(dr3));

    acc_chk #(.EXP_FILE("sim_data/expected_pdw_s1.hex"), .N_EXP(80),  .EDGE(0)) k1 (
        .clk(clk), .m_data(d1), .m_valid(v1), .m_ready(r1), .m_last(l1));
    acc_chk #(.EXP_FILE("sim_data/expected_pdw_s2.hex"), .N_EXP(102), .EDGE(1)) k2 (
        .clk(clk), .m_data(d2), .m_valid(v2), .m_ready(r2), .m_last(l2));
    acc_chk #(.EXP_FILE("sim_data/expected_pdw_s3.hex"), .N_EXP(9),   .EDGE(0)) k3 (
        .clk(clk), .m_data(d3), .m_valid(v3), .m_ready(r3), .m_last(l3));

    reg ok1, ok2, ok3;
    integer pass_cnt;

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        run = 1'b1;
        wait (rd1 && rd2 && rd3);
        repeat (2000) @(posedge clk);

        $display("========================================================================");
        $display("rtl-pdw stage 6: end-to-end accuracy vs truth table (READY_MODE=%0d)", READY_MODE);
        $display("========================================================================");
        k1.print_report("S1", dr1, ov1, ok1);
        k2.print_report("S2", dr2, ov2, ok2);
        k3.print_report("S3", dr3, ov3, ok3);
        $display("     S3 glitch rejected: %0d (expect 1)", gl3);
        $display("------------------------------------------------------------------------");
        pass_cnt = ok1 + ok2 + ((gl3 == 1) ? ok3 : 0);
        $display("TOTAL: %0d / 3 scenarios PASS  (stage6 end-to-end)", pass_cnt);
        $display("------------------------------------------------------------------------");
        $finish;
    end

    initial begin
        #40_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
