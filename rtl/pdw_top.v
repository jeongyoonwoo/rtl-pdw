//==============================================================================
// pdw_top.v   (rtl-pdw 6단계 최상위)
// 합성 IQ ROM -> CORDIC 크기·위상 -> 펄스 검출·PDW 측정 -> 텔레메트리 프레임 바이트 스트림
//
//   pdw_frontend : ROM 재생, 1MSPS 샘플 스트로브, cordic_engine(Vectoring)
//   pdw_detector : 히스테리시스 펄스 검출, TOA/PW/PA/FREQ/FLAGS
//   pdw_framer   : 21바이트 프레임(ASM, FRAME_CNT, N_PDW, PDW, CRC-16-CCITT)
//
// 상태 출력은 Signal Tap 관측과 7단계 보드 데모에서 쓴다.
//==============================================================================

module pdw_top #(
    parameter ROM_FILE   = "sim_data/iq_rom_s1.hex",
    parameter N_SAMPLES  = 8192,
    parameter SAMPLE_DIV = 50,
    parameter TH_HI      = 3000,
    parameter TH_LO      = 2000,
    parameter MIN_W      = 4,
    parameter END_LOW    = 2,
    parameter FREQ_SKIP  = 3,      // 1 이상이어야 함: 펄스 시작 샘플(상대 0)은 누산하지 않으므로 0이면 7개만 더해짐
    parameter [15:0] CRC_INIT = 16'hFFFF
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run,

    // 텔레메트리 바이트 스트림
    output wire [7:0]  m_data,
    output wire        m_valid,
    input  wire        m_ready,
    output wire        m_last,

    // 상태
    output wire        rom_done,
    output wire        overrun,       // 1이면 설계 오류 (CORDIC이 샘플 간격 안에 못 끝남)
    output wire [15:0] glitch_cnt,    // MIN_W보다 짧아서 버린 펄스 수
    output wire [15:0] frame_cnt,     // 보낸 프레임 수
    output wire [15:0] pdw_drop       // 인코더가 바빠서 버린 PDW 수
);

    wire               mv;
    wire [12:0]        mi;
    wire signed [15:0] mm, mp;
    wire               pv;
    wire [31:0]        p_toa;
    wire [15:0]        p_pw, p_pa, p_freq, p_flags;

    pdw_frontend #(
        .ROM_FILE(ROM_FILE), .N_SAMPLES(N_SAMPLES), .SAMPLE_DIV(SAMPLE_DIV)
    ) u_frontend (
        .clk(clk), .rst_n(rst_n), .run(run),
        .meas_valid(mv), .meas_idx(mi), .meas_mag(mm), .meas_phase(mp),
        .rom_done(rom_done), .overrun(overrun)
    );

    pdw_detector #(
        .TH_HI(TH_HI), .TH_LO(TH_LO), .MIN_W(MIN_W), .END_LOW(END_LOW), .FREQ_SKIP(FREQ_SKIP)
    ) u_detector (
        .clk(clk), .rst_n(rst_n),
        .meas_valid(mv), .meas_mag(mm), .meas_phase(mp),
        .pdw_valid(pv), .pdw_toa(p_toa), .pdw_pw(p_pw), .pdw_pa(p_pa),
        .pdw_freq(p_freq), .pdw_flags(p_flags), .glitch_cnt(glitch_cnt)
    );

    pdw_framer #(
        .CRC_INIT(CRC_INIT)
    ) u_framer (
        .clk(clk), .rst_n(rst_n),
        .pdw_valid(pv), .pdw_toa(p_toa), .pdw_pw(p_pw), .pdw_pa(p_pa),
        .pdw_freq(p_freq), .pdw_flags(p_flags),
        .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
        .frame_cnt(frame_cnt), .pdw_drop(pdw_drop)
    );

endmodule
