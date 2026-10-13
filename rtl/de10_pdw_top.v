//==============================================================================
// de10_pdw_top.v   (rtl-pdw 7단계, DE10-Standard 보드 최상위)
// 포트 이름은 Terasic DE10-Standard 기본 최상위(golden top / System Builder)와 같다.
// 핀 배정은 그 qsf의 CLOCK_50, KEY, SW, LEDR, HEX0~HEX5 줄을 그대로 쓰면 된다.
//
// 조작
//   KEY[0] : 리셋 (누르면 처음부터 다시)
//   SW[0]  : 1이면 재생 시작 (8,192샘플, 약 8.2ms에 끝남)
//   SW[1]  : 표시 선택. 0 = 정상 결과, 1 = 오류 카운터
//
// 표시 (SW[1] = 0)
//   HEX5~4 : 정상 프레임 수 하위 8비트 (S1 50, S2 66, S3 09 -- 16진)
//   HEX3~0 : 스트림 서명 (S1 23B3, S2 9829, S3 F352)
// 표시 (SW[1] = 1)
//   HEX5~4 : pdw_drop 하위 8비트 (00이어야 함)
//   HEX3~0 : 오류 프레임 수 (0000이어야 함)
// LED
//   LEDR[0] 재생 끝 / [1] overrun / [2] pdw_drop 있음 / [3] 오류 프레임 있음
//   LEDR[4] 합격: 정상 프레임 수와 서명이 기대값과 같고 오류·유실·overrun이 없음
//   LEDR[8] 동작 표시(약 1.5Hz 깜빡임) / LEDR[9] SW[0] 상태
//
// 시나리오는 합성할 때 SCEN 파라미터(1~3)로 고른다. ROM 파일은 프로젝트 폴더 기준 경로다.
// 출력 스트림의 ready는 1로 고정한다(받는 쪽이 항상 준비됨).
//==============================================================================

module de10_pdw_top #(
    parameter SCEN = 2
) (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [9:0]  SW,
    output wire [9:0]  LEDR,
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5
);

    localparam ROM_FILE   = (SCEN == 1) ? "sim_data/iq_rom_s1.hex" :
                            (SCEN == 3) ? "sim_data/iq_rom_s3.hex" : "sim_data/iq_rom_s2.hex";
    localparam [15:0] EXP_SIG    = (SCEN == 1) ? 16'h23B3 : (SCEN == 3) ? 16'hF352 : 16'h9829;
    localparam [15:0] EXP_FRAMES = (SCEN == 1) ? 16'd80   : (SCEN == 3) ? 16'd9    : 16'd102;

    wire clk = CLOCK_50;

    //--------------------------------------------------------------------
    // 리셋: 누르면 바로 리셋, 뗄 때는 클록에 맞춰 풀림 (2단 동기화)
    //--------------------------------------------------------------------
    reg [1:0] rst_sync;
    always @(posedge clk or negedge KEY[0])
        if (!KEY[0]) rst_sync <= 2'b00;
        else         rst_sync <= {rst_sync[0], 1'b1};
    wire rst_n = rst_sync[1];

    // 스위치 동기화
    reg [1:0] run_sync, sel_sync;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin run_sync <= 2'b00; sel_sync <= 2'b00; end
        else begin
            run_sync <= {run_sync[0], SW[0]};
            sel_sync <= {sel_sync[0], SW[1]};
        end
    wire run = run_sync[1];
    wire sel = sel_sync[1];

    //--------------------------------------------------------------------
    // 처리 경로와 수신 모니터
    //--------------------------------------------------------------------
    wire [7:0]  m_data;
    wire        m_valid, m_last;
    wire        m_ready = 1'b1;
    wire        rom_done, overrun;
    wire [15:0] glitch_cnt, frame_cnt, pdw_drop;
    wire [15:0] good_frames, bad_frames, signature;

    pdw_top #(.ROM_FILE(ROM_FILE)) u_pdw (
        .clk(clk), .rst_n(rst_n), .run(run),
        .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
        .rom_done(rom_done), .overrun(overrun), .glitch_cnt(glitch_cnt),
        .frame_cnt(frame_cnt), .pdw_drop(pdw_drop)
    );

    pdw_stream_mon u_mon (
        .clk(clk), .rst_n(rst_n),
        .s_data(m_data), .s_valid(m_valid), .s_ready(m_ready), .s_last(m_last),
        .good_frames(good_frames), .bad_frames(bad_frames), .signature(signature)
    );

    //--------------------------------------------------------------------
    // 표시
    //--------------------------------------------------------------------
    reg [24:0] hb;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) hb <= 25'd0;
        else        hb <= hb + 25'd1;

    wire pass = rom_done && !m_valid && (good_frames == EXP_FRAMES) && (signature == EXP_SIG)
                && (bad_frames == 16'd0) && (pdw_drop == 16'd0) && !overrun;

    assign LEDR[0] = rom_done;
    assign LEDR[1] = overrun;
    assign LEDR[2] = (pdw_drop != 16'd0);
    assign LEDR[3] = (bad_frames != 16'd0);
    assign LEDR[4] = pass;
    assign LEDR[7:5] = 3'b000;
    assign LEDR[8] = hb[24];
    assign LEDR[9] = run;

    wire [15:0] low16 = sel ? bad_frames    : signature;
    wire [7:0]  high8 = sel ? pdw_drop[7:0] : good_frames[7:0];

    seg7_hex u_h0 (.v(low16[3:0]),   .seg(HEX0));
    seg7_hex u_h1 (.v(low16[7:4]),   .seg(HEX1));
    seg7_hex u_h2 (.v(low16[11:8]),  .seg(HEX2));
    seg7_hex u_h3 (.v(low16[15:12]), .seg(HEX3));
    seg7_hex u_h4 (.v(high8[3:0]),   .seg(HEX4));
    seg7_hex u_h5 (.v(high8[7:4]),   .seg(HEX5));

endmodule

//------------------------------------------------------------------------------
// 16진 한 자리 -> 7세그먼트 (DE10 보드는 0일 때 켜짐, 비트 순서 {g,f,e,d,c,b,a})
//------------------------------------------------------------------------------
module seg7_hex (
    input  wire [3:0] v,
    output reg  [6:0] seg
);
    always @(*) begin
        case (v)
            4'h0: seg = 7'b1000000;
            4'h1: seg = 7'b1111001;
            4'h2: seg = 7'b0100100;
            4'h3: seg = 7'b0110000;
            4'h4: seg = 7'b0011001;
            4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010;
            4'h7: seg = 7'b1111000;
            4'h8: seg = 7'b0000000;
            4'h9: seg = 7'b0010000;
            4'hA: seg = 7'b0001000;
            4'hB: seg = 7'b0000011;
            4'hC: seg = 7'b1000110;
            4'hD: seg = 7'b0100001;
            4'hE: seg = 7'b0000110;
            default: seg = 7'b0001110;   // F
        endcase
    end
endmodule
