//==============================================================================
// tb_de10_pdw_top.v   (rtl-pdw 7단계 보드 최상위 테스트벤치)
// de10_pdw_top을 SCEN 1~3으로 3개 만들어 버튼·스위치를 보드처럼 조작하고,
// 7세그먼트와 LED에 보일 값을 그대로 읽어 출력한다. 보드에서 같은 값이 보여야 한다.
// 실행 위치: rtl-pdw 폴더(sim_data가 보이는 곳)
//==============================================================================
`timescale 1ns/1ps

module tb_de10_pdw_top;
    reg        clk = 1'b0;
    reg  [3:0] key = 4'b1111;
    reg  [9:0] sw  = 10'd0;
    always #10 clk = ~clk;

    wire [9:0] led1, led2, led3;
    wire [6:0] a0, a1, a2, a3, a4, a5;
    wire [6:0] b0, b1, b2, b3, b4, b5;
    wire [6:0] c0, c1, c2, c3, c4, c5;

    de10_pdw_top #(.SCEN(1)) u1 (.CLOCK_50(clk), .KEY(key), .SW(sw), .LEDR(led1),
        .HEX0(a0), .HEX1(a1), .HEX2(a2), .HEX3(a3), .HEX4(a4), .HEX5(a5));
    de10_pdw_top #(.SCEN(2)) u2 (.CLOCK_50(clk), .KEY(key), .SW(sw), .LEDR(led2),
        .HEX0(b0), .HEX1(b1), .HEX2(b2), .HEX3(b3), .HEX4(b4), .HEX5(b5));
    de10_pdw_top #(.SCEN(3)) u3 (.CLOCK_50(clk), .KEY(key), .SW(sw), .LEDR(led3),
        .HEX0(c0), .HEX1(c1), .HEX2(c2), .HEX3(c3), .HEX4(c4), .HEX5(c5));

    // 7세그먼트 패턴 -> 16진 문자
    function [7:0] dig;
        input [6:0] s;
        begin
            case (s)
                7'b1000000: dig = "0"; 7'b1111001: dig = "1"; 7'b0100100: dig = "2";
                7'b0110000: dig = "3"; 7'b0011001: dig = "4"; 7'b0010010: dig = "5";
                7'b0000010: dig = "6"; 7'b1111000: dig = "7"; 7'b0000000: dig = "8";
                7'b0010000: dig = "9"; 7'b0001000: dig = "A"; 7'b0000011: dig = "b";
                7'b1000110: dig = "C"; 7'b0100001: dig = "d"; 7'b0000110: dig = "E";
                7'b0001110: dig = "F"; default:    dig = "?";
            endcase
        end
    endfunction

    task show;
        input [8*2-1:0] name;
        input [9:0]     led;
        input [6:0]     h5, h4, h3, h2, h1, h0;
        begin
            $display("  %s  HEX5..0 = %c%c %c%c%c%c   LEDR[4:0] = %b  (pass=%b done=%b overrun=%b drop=%b bad=%b)",
                     name, dig(h5), dig(h4), dig(h3), dig(h2), dig(h1), dig(h0),
                     led[4:0], led[4], led[0], led[1], led[2], led[3]);
        end
    endtask

    initial begin
        // 전원 직후: 리셋 버튼 한 번
        repeat (3) @(posedge clk);
        key[0] = 1'b0;
        repeat (5) @(posedge clk);
        key[0] = 1'b1;
        repeat (20) @(posedge clk);

        // SW[0] 올려서 재생 시작
        sw[0] = 1'b1;
        #8_500_000;                      // 8,192샘플 x 1us + 여유

        $display("==== SW[1]=0 : HEX5-4 good frames, HEX3-0 signature ====");
        show("S1", led1, a5, a4, a3, a2, a1, a0);
        show("S2", led2, b5, b4, b3, b2, b1, b0);
        show("S3", led3, c5, c4, c3, c2, c1, c0);
        $display("  expect: S1 50 23B3, S2 66 9829, S3 09 F352, LEDR[4]=1");

        sw[1] = 1'b1;
        repeat (10) @(posedge clk);
        $display("==== SW[1]=1 : HEX5-4 pdw_drop, HEX3-0 bad frames ====");
        show("S1", led1, a5, a4, a3, a2, a1, a0);
        show("S2", led2, b5, b4, b3, b2, b1, b0);
        show("S3", led3, c5, c4, c3, c2, c1, c0);
        $display("  expect: 00 0000 for all");

        $display("----------------------------------------");
        $display("TOTAL: %0d / 3 boards PASS (LEDR[4])", {1'b0,led1[4]} + {1'b0,led2[4]} + {1'b0,led3[4]});
        $display("----------------------------------------");

        // 리셋 후 다시 재생해도 같은 결과인지
        sw[1] = 1'b0;
        key[0] = 1'b0; repeat (5) @(posedge clk); key[0] = 1'b1;
        repeat (5) @(posedge clk);
        if (led1[4] || led2[4] || led3[4]) $display("  reset check: FAIL (pass LED still on after reset)");
        else                               $display("  reset check: pass LED cleared");
        #8_500_000;
        $display("  replay after reset: %0d / 3 boards PASS", {1'b0,led1[4]} + {1'b0,led2[4]} + {1'b0,led3[4]});
        $finish;
    end
endmodule
