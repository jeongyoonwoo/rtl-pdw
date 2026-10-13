//==============================================================================
// pdw_stream_mon.v   (rtl-pdw 7단계, 보드 데모용 수신 모니터)
// 텔레메트리 바이트 스트림을 받아
//   - 프레임마다 ASM과 CRC-16-CCITT를 다시 확인해 정상/오류 프레임 수를 센다
//   - 스트림 전체 바이트에 대한 CRC-16-CCITT(초기값 0xFFFF)를 서명으로 낸다
// 서명이 시뮬레이션 기대값과 같으면 출력 바이트 전체가 비트 단위로 같다는 뜻이다.
//   기대 서명: S1 0x23B3 (80프레임), S2 0x9829 (102프레임), S3 0xF352 (9프레임)
// 보드에는 바이트를 받을 장치가 없으므로, 이 모니터가 프레임 경로를 합성에서 지워지지 않게 붙잡는 역할도 한다.
//==============================================================================

module pdw_stream_mon (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  s_data,
    input  wire        s_valid,
    input  wire        s_ready,
    input  wire        s_last,

    output reg  [15:0] good_frames,    // ASM과 CRC가 맞은 프레임 수
    output reg  [15:0] bad_frames,     // ASM, CRC, 길이 중 하나라도 틀린 프레임 수
    output reg  [15:0] signature       // 지금까지 받은 모든 바이트의 CRC-16
);

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

    wire       fire = s_valid && s_ready;
    reg  [4:0] pos;            // 프레임 안 바이트 위치 (0~20)
    reg [15:0] fcrc;           // 이 프레임의 CRC (바이트 4~18)
    reg  [7:0] crc_hi;         // 받은 CRC 상위 바이트
    reg        asm_ok;

    // 위치별 기대 ASM 바이트
    reg [7:0] asm_byte;
    always @(*) begin
        case (pos)
            5'd0:    asm_byte = 8'h1A;
            5'd1:    asm_byte = 8'hCF;
            5'd2:    asm_byte = 8'hFC;
            default: asm_byte = 8'h1D;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos         <= 5'd0;
            fcrc        <= 16'hFFFF;
            crc_hi      <= 8'd0;
            asm_ok      <= 1'b1;
            good_frames <= 16'd0;
            bad_frames  <= 16'd0;
            signature   <= 16'hFFFF;
        end else if (fire) begin
            signature <= crc16_byte(signature, s_data);

            if (pos <= 5'd3 && s_data != asm_byte) asm_ok <= 1'b0;
            if (pos >= 5'd4 && pos <= 5'd18)       fcrc   <= crc16_byte(fcrc, s_data);
            if (pos == 5'd19)                      crc_hi <= s_data;

            if (s_last) begin
                if ((pos == 5'd20) && asm_ok && ({crc_hi, s_data} == fcrc))
                    good_frames <= good_frames + 16'd1;
                else
                    bad_frames  <= bad_frames + 16'd1;
                pos    <= 5'd0;
                fcrc   <= 16'hFFFF;
                asm_ok <= 1'b1;
            end else if (pos != 5'd31) begin
                pos <= pos + 5'd1;
            end
        end
    end

endmodule
