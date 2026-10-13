//==============================================================================
// tb_pdw_framer.v   (rtl-pdw 5단계 테스트벤치)
// pdw_frontend -> pdw_detector -> pdw_framer 전체 경로를 S1~S3에서 동시에 재생하고,
// 출력 바이트 스트림에서 프레임을 직접 해독해 검사한다.
//
// 합격 조건 (시나리오마다)
//   - 프레임 수가 모델 PDW 수와 같다 (S1 80, S2 102, S3 9)
//   - 프레임마다 21바이트 전부가 기대 프레임(model_frame_sN.hex, Python 생성)과 같다
//   - 테스트벤치가 직접 다시 계산한 CRC가 프레임의 CRC와 같다
//   - ASM이 1ACFFC1D, FRAME_CNT가 0부터 1씩 증가, m_last가 21번째 바이트에서만 선다
//   - ready가 0인 동안 m_data/m_last/m_valid가 바뀌지 않는다
//   - pdw_drop = 0, overrun 없음
//
// READY_MODE: 0 = ready 항상 1, 1 = 무작위 약 50%, 2 = 약 1/8만 1 (강한 역압)
//             3 = 약 1/256 (프레임이 펄스 간격보다 느려짐 -> pdw_drop 동작 확인용, FAIL이 정상)
// 실행 위치: rtl-pdw 폴더(sim_data가 보이는 곳)
//==============================================================================
`timescale 1ns/1ps

//------------------------------------------------------------------------------
// ready 생성기 (16비트 LFSR)
//------------------------------------------------------------------------------
module ready_gen #(
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
    assign ready = (MODE == 0) ? 1'b1 :
                   (MODE == 1) ? lfsr[0] :
                   (MODE == 2) ? (lfsr[2:0] == 3'b000) :
                                 (lfsr[7:0] == 8'h00);    // 3: 약 1/256, PDW 유실 시험용
endmodule

//------------------------------------------------------------------------------
// 프레임 검사기
//------------------------------------------------------------------------------
module frame_chk #(
    parameter FRAME_FILE = "sim_data/model_frame_s1.hex",
    parameter N_EXP      = 80
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  m_data,
    input  wire        m_valid,
    input  wire        m_ready,
    input  wire        m_last,
    output reg  [31:0] n_frames,
    output reg  [31:0] e_byte,     // 기대 프레임과 다른 프레임 수
    output reg  [31:0] e_crc,      // 다시 계산한 CRC가 다른 프레임 수
    output reg  [31:0] e_asm,
    output reg  [31:0] e_cnt,      // FRAME_CNT 순번 오류
    output reg  [31:0] e_len,      // 길이 / m_last 위치 오류
    output reg  [31:0] e_hold      // ready=0 동안 출력이 바뀐 횟수
);
    reg [167:0] exp_mem [0:N_EXP-1];
    initial $readmemh(FRAME_FILE, exp_mem);

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

    reg [7:0]   fb [0:20];      // 받은 프레임
    reg [5:0]   pos;
    reg         stall;          // 직전 클록이 valid && !ready
    reg [7:0]   d_prev;
    reg         l_prev;
    reg [167:0] e;
    reg [15:0]  c;
    integer     i;
    reg         bad;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            n_frames <= 0; e_byte <= 0; e_crc <= 0; e_asm <= 0; e_cnt <= 0; e_len <= 0; e_hold <= 0;
            pos <= 0; stall <= 0; d_prev <= 0; l_prev <= 0;
        end else begin
            // 대기 중 출력 유지 검사
            if (stall && (!m_valid || (m_data !== d_prev) || (m_last !== l_prev)))
                e_hold <= e_hold + 1;
            stall  <= m_valid && !m_ready;
            d_prev <= m_data;
            l_prev <= m_last;

            if (m_valid && m_ready) begin
                if (pos <= 20) fb[pos] = m_data;
                if (m_last) begin
                    if (pos != 20) e_len <= e_len + 1;
                    // ASM
                    if ({fb[0], fb[1], fb[2], fb[3]} !== 32'h1ACFFC1D) e_asm <= e_asm + 1;
                    // FRAME_CNT
                    if ({fb[4], fb[5]} !== n_frames[15:0]) e_cnt <= e_cnt + 1;
                    // CRC 재계산
                    c = 16'hFFFF;
                    for (i = 4; i <= 18; i = i + 1) c = crc16_byte(c, fb[i]);
                    if ({fb[19], fb[20]} !== c) e_crc <= e_crc + 1;
                    // 기대 프레임과 비교
                    bad = 1'b0;
                    if (n_frames < N_EXP) begin
                        e = exp_mem[n_frames];
                        for (i = 0; i <= 20; i = i + 1)
                            if (fb[i] !== e[167 - 8*i -: 8]) bad = 1'b1;
                    end else begin
                        bad = 1'b1;
                    end
                    if (bad) e_byte <= e_byte + 1;
                    n_frames <= n_frames + 1;
                    pos <= 0;
                end else begin
                    if (pos >= 20) e_len <= e_len + 1;   // 21번째 바이트에 last가 없음
                    pos <= pos + 1;
                end
            end
        end
    end
endmodule

//------------------------------------------------------------------------------
// 한 시나리오 전체 경로
//------------------------------------------------------------------------------
module pdw_chain #(
    parameter ROM_FILE   = "sim_data/iq_rom_s1.hex",
    parameter N_SAMPLES  = 8192,
    parameter SAMPLE_DIV = 50,
    parameter [15:0] CRC_INIT = 16'hFFFF
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run,
    input  wire        m_ready,
    output wire [7:0]  m_data,
    output wire        m_valid,
    output wire        m_last,
    output wire        rom_done,
    output wire        overrun,
    output wire [15:0] frame_cnt,
    output wire [15:0] pdw_drop,
    output wire [15:0] glitch_cnt
);
    wire               mv;
    wire [12:0]        mi;
    wire signed [15:0] mm, mp;
    wire               pv;
    wire [31:0]        t;
    wire [15:0]        w, a, f, g;

    pdw_frontend #(.ROM_FILE(ROM_FILE), .N_SAMPLES(N_SAMPLES), .SAMPLE_DIV(SAMPLE_DIV)) fe (
        .clk(clk), .rst_n(rst_n), .run(run), .meas_valid(mv), .meas_idx(mi), .meas_mag(mm),
        .meas_phase(mp), .rom_done(rom_done), .overrun(overrun));
    pdw_detector dt (
        .clk(clk), .rst_n(rst_n), .meas_valid(mv), .meas_mag(mm), .meas_phase(mp),
        .pdw_valid(pv), .pdw_toa(t), .pdw_pw(w), .pdw_pa(a), .pdw_freq(f), .pdw_flags(g),
        .glitch_cnt(glitch_cnt));
    pdw_framer #(.CRC_INIT(CRC_INIT)) fr (
        .clk(clk), .rst_n(rst_n), .pdw_valid(pv), .pdw_toa(t), .pdw_pw(w), .pdw_pa(a),
        .pdw_freq(f), .pdw_flags(g), .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready),
        .m_last(m_last), .frame_cnt(frame_cnt), .pdw_drop(pdw_drop));
endmodule

//------------------------------------------------------------------------------
// 테스트벤치 본체
//------------------------------------------------------------------------------
module tb_pdw_framer;
    parameter READY_MODE = 0;
    parameter [15:0] CRC_INIT = 16'hFFFF;   // 시험용으로 바꿔 보는 값(기본은 계획서 값)

    reg clk = 1'b0, rst_n = 1'b0, run = 1'b0;
    always #10 clk = ~clk;

    wire        rdy1, rdy2, rdy3;
    wire [7:0]  d1, d2, d3;
    wire        v1, v2, v3, l1, l2, l3, rd1, rd2, rd3, ov1, ov2, ov3;
    wire [15:0] fc1, fc2, fc3, dr1, dr2, dr3, gl1, gl2, gl3;

    ready_gen #(.MODE(READY_MODE), .SEED(16'hACE1)) rg1 (.clk(clk), .rst_n(rst_n), .ready(rdy1));
    ready_gen #(.MODE(READY_MODE), .SEED(16'h1D2B)) rg2 (.clk(clk), .rst_n(rst_n), .ready(rdy2));
    ready_gen #(.MODE(READY_MODE), .SEED(16'h7F31)) rg3 (.clk(clk), .rst_n(rst_n), .ready(rdy3));

    pdw_chain #(.ROM_FILE("sim_data/iq_rom_s1.hex"), .CRC_INIT(CRC_INIT)) ch1 (
        .clk(clk), .rst_n(rst_n), .run(run), .m_ready(rdy1), .m_data(d1), .m_valid(v1), .m_last(l1),
        .rom_done(rd1), .overrun(ov1), .frame_cnt(fc1), .pdw_drop(dr1), .glitch_cnt(gl1));
    pdw_chain #(.ROM_FILE("sim_data/iq_rom_s2.hex"), .CRC_INIT(CRC_INIT)) ch2 (
        .clk(clk), .rst_n(rst_n), .run(run), .m_ready(rdy2), .m_data(d2), .m_valid(v2), .m_last(l2),
        .rom_done(rd2), .overrun(ov2), .frame_cnt(fc2), .pdw_drop(dr2), .glitch_cnt(gl2));
    pdw_chain #(.ROM_FILE("sim_data/iq_rom_s3.hex"), .CRC_INIT(CRC_INIT)) ch3 (
        .clk(clk), .rst_n(rst_n), .run(run), .m_ready(rdy3), .m_data(d3), .m_valid(v3), .m_last(l3),
        .rom_done(rd3), .overrun(ov3), .frame_cnt(fc3), .pdw_drop(dr3), .glitch_cnt(gl3));

    wire [31:0] n1, b1, c1, a1, q1, z1, h1;
    wire [31:0] n2, b2, c2, a2, q2, z2, h2;
    wire [31:0] n3, b3, c3, a3, q3, z3, h3;

    frame_chk #(.FRAME_FILE("sim_data/model_frame_s1.hex"), .N_EXP(80)) k1 (
        .clk(clk), .rst_n(rst_n), .m_data(d1), .m_valid(v1), .m_ready(rdy1), .m_last(l1),
        .n_frames(n1), .e_byte(b1), .e_crc(c1), .e_asm(a1), .e_cnt(q1), .e_len(z1), .e_hold(h1));
    frame_chk #(.FRAME_FILE("sim_data/model_frame_s2.hex"), .N_EXP(102)) k2 (
        .clk(clk), .rst_n(rst_n), .m_data(d2), .m_valid(v2), .m_ready(rdy2), .m_last(l2),
        .n_frames(n2), .e_byte(b2), .e_crc(c2), .e_asm(a2), .e_cnt(q2), .e_len(z2), .e_hold(h2));
    frame_chk #(.FRAME_FILE("sim_data/model_frame_s3.hex"), .N_EXP(9)) k3 (
        .clk(clk), .rst_n(rst_n), .m_data(d3), .m_valid(v3), .m_ready(rdy3), .m_last(l3),
        .n_frames(n3), .e_byte(b3), .e_crc(c3), .e_asm(a3), .e_cnt(q3), .e_len(z3), .e_hold(h3));

    // 프레임 하나에 걸린 클록 수(첫 바이트 valid ~ last 전달) 최댓값/최솟값, 시나리오 1 기준
    reg [31:0] fl_cnt, fl_max, fl_min;
    reg        in_fr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin fl_cnt <= 0; fl_max <= 0; fl_min <= 32'hFFFF_FFFF; in_fr <= 0; end
        else if (v1) begin
            if (v1 && rdy1 && l1) begin
                if (fl_cnt + 1 > fl_max) fl_max <= fl_cnt + 1;
                if (fl_cnt + 1 < fl_min) fl_min <= fl_cnt + 1;
                fl_cnt <= 0;
            end else fl_cnt <= fl_cnt + 1;
        end
    end

    integer pass_cnt;
    task report;
        input [8*2-1:0] name;
        input integer   n_exp;
        input [31:0]    nf, eb, ec, ea, eq, ez, eh;
        input [15:0]    drop;
        input           ovr;
        reg ok;
        begin
            ok = (nf == n_exp) && (eb == 0) && (ec == 0) && (ea == 0) && (eq == 0) && (ez == 0)
                 && (eh == 0) && (drop == 0) && !ovr;
            if (ok) pass_cnt = pass_cnt + 1;
            $display("[%s] %s frames=%0d(expect %0d) byte_mismatch=%0d crc_fail=%0d asm_err=%0d cnt_err=%0d len_err=%0d hold_err=%0d pdw_drop=%0d overrun=%0d",
                     ok ? "PASS" : "FAIL", name, nf, n_exp, eb, ec, ea, eq, ez, eh, drop, ovr);
        end
    endtask

    // CRC 함수 자체 확인: "123456789" -> 0x29B1
    function [15:0] crc16_byte_tb;
        input [15:0] crc_in;
        input [7:0]  data;
        integer      k;
        reg   [15:0] c;
        begin
            c = crc_in ^ {data, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_byte_tb = c;
        end
    endfunction

    reg [15:0] crc_chk;
    integer    j;

    initial begin
        pass_cnt = 0;
        crc_chk = 16'hFFFF;
        for (j = 0; j < 9; j = j + 1) crc_chk = crc16_byte_tb(crc_chk, 8'h31 + j);
        $display("CRC self-check \"123456789\" = 0x%04X (expect 0x29B1) %s",
                 crc_chk, (crc_chk == 16'h29B1) ? "OK" : "WRONG");
        $display("READY_MODE = %0d (0: always, 1: ~50%%, 2: ~1/8, 3: ~1/256 drop test)", READY_MODE);

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        run = 1'b1;
        wait (rd1 && rd2 && rd3);
        repeat (2000) @(posedge clk);    // 마지막 프레임 전송 대기 (강한 역압 포함)

        report("S1", 80,  n1, b1, c1, a1, q1, z1, h1, dr1, ov1);
        report("S2", 102, n2, b2, c2, a2, q2, z2, h2, dr2, ov2);
        report("S3", 9,   n3, b3, c3, a3, q3, z3, h3, dr3, ov3);
        $display("S1 frame time: min %0d / max %0d clocks (21 = no backpressure)", fl_min, fl_max);
        $display("----------------------------------------");
        $display("TOTAL: %0d / 3 scenarios PASS  (stage5 telemetry framer)", pass_cnt);
        $display("----------------------------------------");
        $finish;
    end

    initial begin
        #40_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
