//==============================================================================
// pdw_framer.v   (rtl-pdw 5단계)
// PDW 한 건을 21바이트 텔레메트리 프레임 하나로 만들어 바이트 스트림으로 내보낸다.
//
// 프레임 (상위 바이트 먼저)
//   idx  0~3  ASM        0x1ACFFC1D (CCSDS 동기 마커)
//   idx  4~5  FRAME_CNT  프레임 순번 (65,536에서 0으로)
//   idx  6    N_PDW      1
//   idx  7~10 TOA
//   idx 11~12 PW
//   idx 13~14 PA
//   idx 15~16 FREQ
//   idx 17~18 FLAGS
//   idx 19~20 CRC        CRC-16-CCITT (0x1021, 초기값 CRC_INIT), idx 4~18의 15바이트
//
// 출력 인터페이스: m_valid / m_ready / m_last (m_valid && m_ready인 클록에 바이트 한 개 전달)
//   m_valid가 선 뒤 m_ready가 올 때까지 m_data, m_last는 바뀌지 않는다.
// PDW 보관 레지스터는 1개다. 프레임을 보내는 중에 새 PDW가 오면 버리고 pdw_drop을 올린다.
//   (마지막 바이트가 나가는 바로 그 클록에 온 PDW는 받는다.)
// CRC는 바이트가 실제로 나갈 때마다 1바이트씩 갱신하므로 idx 19에서 이미 최종값이다.
// ready가 항상 1이면 프레임 하나에 21클록이 걸린다.
//==============================================================================

module pdw_framer #(
    parameter [15:0] CRC_INIT = 16'hFFFF
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        pdw_valid,
    input  wire [31:0] pdw_toa,
    input  wire [15:0] pdw_pw,
    input  wire [15:0] pdw_pa,
    input  wire [15:0] pdw_freq,
    input  wire [15:0] pdw_flags,

    output wire [7:0]  m_data,
    output wire        m_valid,
    input  wire        m_ready,
    output wire        m_last,

    output reg  [15:0] frame_cnt,      // 다음 프레임에 들어갈 순번 = 지금까지 보낸 프레임 수
    output reg  [15:0] pdw_drop        // 버려진 PDW 수 (정상 시나리오에서 0)
);

    //--------------------------------------------------------------------
    // CRC-16-CCITT 1바이트 갱신 (비트 직렬 8회를 조합 논리로 펼침)
    //--------------------------------------------------------------------
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

    //--------------------------------------------------------------------
    // 상태
    //--------------------------------------------------------------------
    reg        busy;
    reg [4:0]  idx;          // 지금 내보내는 바이트 번호 0~20
    reg [15:0] cnt_hold;     // 이 프레임의 FRAME_CNT
    reg [31:0] toa_h;
    reg [15:0] pw_h, pa_h, freq_h, flags_h;
    reg [15:0] crc;

    wire fire = busy && m_ready;            // 이번 클록에 바이트 하나가 나감
    wire done = fire && (idx == 5'd20);     // 이번 클록에 프레임 마지막 바이트가 나감

    //--------------------------------------------------------------------
    // 바이트 선택 (모두 레지스터 값에서 고르므로 ready를 기다리는 동안 안정)
    //--------------------------------------------------------------------
    reg [7:0] byte_mux;
    always @(*) begin
        case (idx)
            5'd0:  byte_mux = 8'h1A;
            5'd1:  byte_mux = 8'hCF;
            5'd2:  byte_mux = 8'hFC;
            5'd3:  byte_mux = 8'h1D;
            5'd4:  byte_mux = cnt_hold[15:8];
            5'd5:  byte_mux = cnt_hold[7:0];
            5'd6:  byte_mux = 8'h01;
            5'd7:  byte_mux = toa_h[31:24];
            5'd8:  byte_mux = toa_h[23:16];
            5'd9:  byte_mux = toa_h[15:8];
            5'd10: byte_mux = toa_h[7:0];
            5'd11: byte_mux = pw_h[15:8];
            5'd12: byte_mux = pw_h[7:0];
            5'd13: byte_mux = pa_h[15:8];
            5'd14: byte_mux = pa_h[7:0];
            5'd15: byte_mux = freq_h[15:8];
            5'd16: byte_mux = freq_h[7:0];
            5'd17: byte_mux = flags_h[15:8];
            5'd18: byte_mux = flags_h[7:0];
            5'd19: byte_mux = crc[15:8];
            5'd20: byte_mux = crc[7:0];
            default: byte_mux = 8'h00;
        endcase
    end

    assign m_data  = byte_mux;
    assign m_valid = busy;
    assign m_last  = busy && (idx == 5'd20);

    //--------------------------------------------------------------------
    // 제어
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy      <= 1'b0;
            idx       <= 5'd0;
            cnt_hold  <= 16'd0;
            toa_h     <= 32'd0;
            pw_h      <= 16'd0;
            pa_h      <= 16'd0;
            freq_h    <= 16'd0;
            flags_h   <= 16'd0;
            crc       <= CRC_INIT;
            frame_cnt <= 16'd0;
            pdw_drop  <= 16'd0;
        end else begin
            // 바이트 전송
            if (fire) begin
                idx <= idx + 5'd1;
                if ((idx >= 5'd4) && (idx <= 5'd18))
                    crc <= crc16_byte(crc, byte_mux);
            end
            if (done) begin
                busy      <= 1'b0;
                frame_cnt <= frame_cnt + 16'd1;
            end

            // 새 PDW 받기: 비어 있거나 지금 막 프레임이 끝나는 클록이면 받는다
            if (pdw_valid) begin
                if (!busy || done) begin
                    busy     <= 1'b1;
                    idx      <= 5'd0;
                    crc      <= CRC_INIT;
                    cnt_hold <= done ? (frame_cnt + 16'd1) : frame_cnt;
                    toa_h    <= pdw_toa;
                    pw_h     <= pdw_pw;
                    pa_h     <= pdw_pa;
                    freq_h   <= pdw_freq;
                    flags_h  <= pdw_flags;
                end else begin
                    pdw_drop <= pdw_drop + 16'd1;
                end
            end
        end
    end

endmodule
