`timescale 1ns/1ps

module cordic_engine (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire mode,
    input wire signed [15:0] x_in,
    input wire signed [15:0] y_in,
    input wire signed [15:0] z_in,
    output reg signed [15:0] x_out,
    output reg signed [15:0] y_out,
    output reg signed [15:0] z_out,
    output reg done,
    output reg busy
);

    localparam signed [15:0] CORDIC_K_16 = 16'sh4DBB;
    localparam signed [15:0] CORDIC_K_18 = 18'sh04DBB;
    localparam signed [15:0] POS_PI_Q15  = 16'sh7FFF;
    localparam signed [15:0] NEG_PI_Q15  = 16'sh8000;

    localparam [1:0] STATE_IDLE = 2'd0;
    localparam [1:0] STATE_CALC = 2'd1;
    localparam [1:0] STATE_DONE = 2'd2;

    reg [1:0] state;
    reg [3:0] iter_cnt;

    reg signed [17:0] x;
    reg signed [17:0] y;
    reg signed [15:0] z;
    reg [15:0] lut_angle;

    wire signed [17:0] x_shift = x >>> iter_cnt;
    wire signed [17:0] y_shift = y >>> iter_cnt;

    wire signed [17:0] x_ext = {{2{x_in[15]}}, x_in};
    wire signed [17:0] y_ext = {{2{y_in[15]}}, y_in};

    always @(*) begin
        case (iter_cnt)
            4'd0:    lut_angle = 16'h2000;
            4'd1:    lut_angle = 16'h12E4;
            4'd2:    lut_angle = 16'h09FB;
            4'd3:    lut_angle = 16'h0511;
            4'd4:    lut_angle = 16'h028B;
            4'd5:    lut_angle = 16'h0146;
            4'd6:    lut_angle = 16'h00A3;
            4'd7:    lut_angle = 16'h0051;
            4'd8:    lut_angle = 16'h0029;
            4'd9:    lut_angle = 16'h0014;
            4'd10:   lut_angle = 16'h000A;
            4'd11:   lut_angle = 16'h0005;
            4'd12:   lut_angle = 16'h0003;
            4'd13:   lut_angle = 16'h0001;
            4'd14:   lut_angle = 16'h0001;
            4'd15:   lut_angle = 16'h0000;
            default: lut_angle = 16'h0000;
        endcase
    end

    wire rot_dir = (mode == 1'b0) ? (z >= 0) : (y < 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= STATE_IDLE;
            iter_cnt <= 4'd0;
            x        <= 18'd0;
            y        <= 18'd0;
            z        <= 16'd0;
            x_out    <= 16'd0;
            y_out    <= 16'd0;
            z_out    <= 16'd0;
            done     <= 1'b0;
            busy     <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        iter_cnt <= 4'd0;
                        busy     <= 1'b1;
                        state    <= STATE_CALC;

                        if (mode == 1'b0) begin
                            // Mode 0: Rotation Mode (회전 모드)
                            //         입력 (1/K, 0)이면 각도 z_in에 대한 cos/sin을 출력
                            //         일반적으로는 (x_in, y_in)을 z_in만큼 회전
                            x <= x_ext;
                            y <= y_ext;
                            z <= z_in;
                        end else begin
                            // Mode 1: Vectoring Mode (벡터링 모드) - 크기 K*|s|와 위상 출력
                            // x_in < 0이면 먼저 pi만큼 돌려 오른쪽 반평면으로 옮긴다
                            if (x_in < 0) begin
                                x <= -x_ext;
                                y <= -y_ext;
                                z <= (y_in < 0) ? NEG_PI_Q15 : POS_PI_Q15;
                            end else begin
                                x <= x_ext;
                                y <= y_ext;
                                z <= 16'sh0000;
                            end
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                STATE_CALC: begin
                    if (rot_dir) begin
                        x <= x - y_shift;
                        y <= y + x_shift;
                        z <= z - $signed(lut_angle);
                    end else begin
                        x <= x + y_shift;
                        y <= y - x_shift;
                        z <= z + $signed(lut_angle);
                    end

                    if (iter_cnt == 4'd15) begin
                        state <= STATE_DONE;
                    end else begin
                        iter_cnt <= iter_cnt + 1'b1;
                    end
                end

                STATE_DONE: begin
                    if (mode == 1'b0) begin
                        x_out <= (x > 18'sh07FFF) ? 16'sh7FFF : ((x < -18'sh08000) ? -16'sh8000 : x[15:0]);
                        y_out <= (y > 18'sh07FFF) ? 16'sh7FFF : ((y < -18'sh08000) ? -16'sh8000 : y[15:0]);
                    end else begin
                        x_out <= (x > 18'sh07FFF) ? 16'sh7FFF : ((x < -18'sh08000) ? -16'sh8000 : x[15:0]);
                        y_out <= y[15:0];
                    end
                    z_out <= z;
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
