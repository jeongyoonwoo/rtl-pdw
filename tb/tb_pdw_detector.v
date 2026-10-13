//==============================================================================
// tb_pdw_detector.v   (rtl-pdw 3~4단계 테스트벤치)
// pdw_frontend -> pdw_detector 전체 경로를 S1~S3에서 동시에 재생하고,
// 나온 PDW를 비트 단위 모델(model_pdw_sN.hex)과 필드별로 비교한다.
//
// 합격 조건 (시나리오마다)
//   - PDW 개수가 모델과 같다(S1 80, S2 102, S3 9)
//   - 모든 필드가 모델과 같다 (TOA, PW, PA, FREQ, FLAGS 각각 불일치 0)
//   - 버려진 글리치 수가 기대와 같다(S3만 1)
//   - frontend overrun 없음
// 참고로 정답표(expected_pdw_sN.hex) 대비 TOA/PW 오차 범위를 함께 출력한다
// (정답표 판정은 Python이 하고, 여기서는 눈으로 확인하는 용도).
//
// 단계 구분: 3단계 = TOA/PW/PA/FLAGS, 4단계 = FREQ. 결과 줄에 필드별로 따로 표시한다.
// 실행 위치: rtl-pdw 폴더(sim_data가 보이는 곳)
//==============================================================================
`timescale 1ns/1ps

module pdw_det_chk #(
    parameter MODEL_FILE = "sim_data/model_pdw_s1.hex",
    parameter EXP_FILE   = "sim_data/expected_pdw_s1.hex",
    parameter N_EXP      = 80
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,
    input  wire [31:0] toa,
    input  wire [15:0] pw, pa, freq, flags,
    output reg  [31:0] n_seen,
    output reg  [31:0] e_toa, e_pw, e_pa, e_freq, e_flags,   // 모델과의 필드별 불일치 수
    output reg signed [31:0] toa_err_min, toa_err_max,       // 정답표 대비 오차(검출 - 정답)
    output reg signed [31:0] pw_err_min,  pw_err_max
);
    reg [95:0] model_mem [0:N_EXP-1];
    reg [95:0] exp_mem   [0:N_EXP-1];
    initial begin
        $readmemh(MODEL_FILE, model_mem);
        $readmemh(EXP_FILE,   exp_mem);
    end

    reg [95:0] m, x;
    integer    d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            n_seen <= 0; e_toa <= 0; e_pw <= 0; e_pa <= 0; e_freq <= 0; e_flags <= 0;
            toa_err_min <= 1000; toa_err_max <= -1000; pw_err_min <= 1000; pw_err_max <= -1000;
        end else if (valid) begin
            if (n_seen < N_EXP) begin
                m = model_mem[n_seen];
                x = exp_mem[n_seen];
                if (toa   !== m[95:64]) e_toa   <= e_toa   + 1;
                if (pw    !== m[63:48]) e_pw    <= e_pw    + 1;
                if (pa    !== m[47:32]) e_pa    <= e_pa    + 1;
                if (freq  !== m[31:16]) e_freq  <= e_freq  + 1;
                if (flags !== m[15:0])  e_flags <= e_flags + 1;
                d = $signed(toa) - $signed(x[95:64]);
                if (d < toa_err_min) toa_err_min <= d;
                if (d > toa_err_max) toa_err_max <= d;
                d = $signed({1'b0, pw}) - $signed({1'b0, x[63:48]});
                if (d < pw_err_min) pw_err_min <= d;
                if (d > pw_err_max) pw_err_max <= d;
            end
            n_seen <= n_seen + 1;
        end
    end
endmodule

module tb_pdw_detector;
    parameter N_SAMPLES  = 8192;
    parameter SAMPLE_DIV = 50;
    parameter MIN_W      = 4;      // 시험용으로 바꿔 보는 값(기본은 계획서 값)
    parameter TH_HI      = 3000;

    reg clk = 1'b0, rst_n = 1'b0, run = 1'b0;
    always #10 clk = ~clk;

    // 시나리오 3개의 신호선
    wire        mv1, mv2, mv3;
    wire [12:0] mi1, mi2, mi3;
    wire signed [15:0] mm1, mm2, mm3, mp1, mp2, mp3;
    wire        rd1, rd2, rd3, ov1, ov2, ov3;
    wire        pv1, pv2, pv3;
    wire [31:0] t1, t2, t3;
    wire [15:0] w1, w2, w3, a1, a2, a3, f1, f2, f3, g1, g2, g3, gl1, gl2, gl3;

    pdw_frontend #(.ROM_FILE("sim_data/iq_rom_s1.hex"), .N_SAMPLES(N_SAMPLES), .SAMPLE_DIV(SAMPLE_DIV)) fe1 (
        .clk(clk), .rst_n(rst_n), .run(run), .meas_valid(mv1), .meas_idx(mi1), .meas_mag(mm1),
        .meas_phase(mp1), .rom_done(rd1), .overrun(ov1));
    pdw_frontend #(.ROM_FILE("sim_data/iq_rom_s2.hex"), .N_SAMPLES(N_SAMPLES), .SAMPLE_DIV(SAMPLE_DIV)) fe2 (
        .clk(clk), .rst_n(rst_n), .run(run), .meas_valid(mv2), .meas_idx(mi2), .meas_mag(mm2),
        .meas_phase(mp2), .rom_done(rd2), .overrun(ov2));
    pdw_frontend #(.ROM_FILE("sim_data/iq_rom_s3.hex"), .N_SAMPLES(N_SAMPLES), .SAMPLE_DIV(SAMPLE_DIV)) fe3 (
        .clk(clk), .rst_n(rst_n), .run(run), .meas_valid(mv3), .meas_idx(mi3), .meas_mag(mm3),
        .meas_phase(mp3), .rom_done(rd3), .overrun(ov3));

    pdw_detector #(.MIN_W(MIN_W), .TH_HI(TH_HI)) dt1 (
        .clk(clk), .rst_n(rst_n), .meas_valid(mv1), .meas_mag(mm1), .meas_phase(mp1),
        .pdw_valid(pv1), .pdw_toa(t1), .pdw_pw(w1), .pdw_pa(a1), .pdw_freq(f1), .pdw_flags(g1), .glitch_cnt(gl1));
    pdw_detector #(.MIN_W(MIN_W), .TH_HI(TH_HI)) dt2 (
        .clk(clk), .rst_n(rst_n), .meas_valid(mv2), .meas_mag(mm2), .meas_phase(mp2),
        .pdw_valid(pv2), .pdw_toa(t2), .pdw_pw(w2), .pdw_pa(a2), .pdw_freq(f2), .pdw_flags(g2), .glitch_cnt(gl2));
    pdw_detector #(.MIN_W(MIN_W), .TH_HI(TH_HI)) dt3 (
        .clk(clk), .rst_n(rst_n), .meas_valid(mv3), .meas_mag(mm3), .meas_phase(mp3),
        .pdw_valid(pv3), .pdw_toa(t3), .pdw_pw(w3), .pdw_pa(a3), .pdw_freq(f3), .pdw_flags(g3), .glitch_cnt(gl3));

    wire [31:0] s1, s2, s3, et1, ew1, ea1, ef1, eg1, et2, ew2, ea2, ef2, eg2, et3, ew3, ea3, ef3, eg3;
    wire signed [31:0] tmin1, tmax1, wmin1, wmax1, tmin2, tmax2, wmin2, wmax2, tmin3, tmax3, wmin3, wmax3;

    pdw_det_chk #(.MODEL_FILE("sim_data/model_pdw_s1.hex"), .EXP_FILE("sim_data/expected_pdw_s1.hex"), .N_EXP(80)) k1 (
        .clk(clk), .rst_n(rst_n), .valid(pv1), .toa(t1), .pw(w1), .pa(a1), .freq(f1), .flags(g1),
        .n_seen(s1), .e_toa(et1), .e_pw(ew1), .e_pa(ea1), .e_freq(ef1), .e_flags(eg1),
        .toa_err_min(tmin1), .toa_err_max(tmax1), .pw_err_min(wmin1), .pw_err_max(wmax1));
    pdw_det_chk #(.MODEL_FILE("sim_data/model_pdw_s2.hex"), .EXP_FILE("sim_data/expected_pdw_s2.hex"), .N_EXP(102)) k2 (
        .clk(clk), .rst_n(rst_n), .valid(pv2), .toa(t2), .pw(w2), .pa(a2), .freq(f2), .flags(g2),
        .n_seen(s2), .e_toa(et2), .e_pw(ew2), .e_pa(ea2), .e_freq(ef2), .e_flags(eg2),
        .toa_err_min(tmin2), .toa_err_max(tmax2), .pw_err_min(wmin2), .pw_err_max(wmax2));
    pdw_det_chk #(.MODEL_FILE("sim_data/model_pdw_s3.hex"), .EXP_FILE("sim_data/expected_pdw_s3.hex"), .N_EXP(9)) k3 (
        .clk(clk), .rst_n(rst_n), .valid(pv3), .toa(t3), .pw(w3), .pa(a3), .freq(f3), .flags(g3),
        .n_seen(s3), .e_toa(et3), .e_pw(ew3), .e_pa(ea3), .e_freq(ef3), .e_flags(eg3),
        .toa_err_min(tmin3), .toa_err_max(tmax3), .pw_err_min(wmin3), .pw_err_max(wmax3));

    integer pass_cnt;
    task report;
        input [8*2-1:0] name;
        input integer   n_exp, n_gl_exp;
        input [31:0]    seen, eto, epw, epa, efr, efl;
        input [15:0]    gl;
        input           ovr;
        input integer   tmn, tmx, wmn, wmx;
        reg ok3, ok4, ok;
        begin
            ok3 = (seen == n_exp) && (eto == 0) && (epw == 0) && (epa == 0) && (efl == 0);
            ok4 = (efr == 0);
            ok  = ok3 && ok4 && (gl == n_gl_exp) && !ovr;
            if (ok) pass_cnt = pass_cnt + 1;
            $display("[%s] %s PDW=%0d(expect %0d) mismatch: TOA=%0d PW=%0d PA=%0d FLAGS=%0d | FREQ=%0d | glitch=%0d(expect %0d) overrun=%0d",
                     ok ? "PASS" : "FAIL", name, seen, n_exp, eto, epw, epa, efl, efr, gl, n_gl_exp, ovr);
            $display("        vs truth (det-truth): TOA %0d~%0d samples, PW %0d~%0d samples", tmn, tmx, wmn, wmx);
        end
    endtask

    initial begin
        pass_cnt = 0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        run = 1'b1;
        wait (rd1 && rd2 && rd3);
        repeat (200) @(posedge clk);     // 마지막 펄스 종료와 출력 대기
        report("S1", 80, 0, s1, et1, ew1, ea1, ef1, eg1, gl1, ov1, tmin1, tmax1, wmin1, wmax1);
        report("S2", 102, 0, s2, et2, ew2, ea2, ef2, eg2, gl2, ov2, tmin2, tmax2, wmin2, wmax2);
        report("S3", 9, 1, s3, et3, ew3, ea3, ef3, eg3, gl3, ov3, tmin3, tmax3, wmin3, wmax3);
        $display("----------------------------------------");
        $display("TOTAL: %0d / 3 scenarios PASS  (stage3 TOA/PW/PA/FLAGS + stage4 FREQ)", pass_cnt);
        $display("----------------------------------------");
        $finish;
    end

    initial begin
        #40_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
