//==============================================================================
// pdw_frontend.v   (rtl-pdw 2단계, 7단계에서 합성용으로 ROM 읽기 구조만 변경)
// 합성 IQ ROM을 SAMPLE_DIV 클록마다 한 샘플씩 꺼내 cordic_engine(Vectoring)에 넣고,
// 샘플마다 크기(K배 스케일)와 위상을 내보낸다.
//
//   meas_valid : 1클록 펄스. 샘플 하나의 결과가 나왔다는 표시
//   meas_idx   : 샘플 번호 (0 ~ N_SAMPLES-1)
//   meas_mag   : K*|s|  (Q1.15, 16비트 포화) -- 입력 |s| > 1/K(~0.607)이면 32767로 포화
//   meas_phase : 위상   (Q1.15, pi = 32768, 16비트 모듈로)
//   rom_done   : 마지막 샘플까지 끝남
//   overrun    : 이전 샘플의 CORDIC 연산이 끝나기 전에 다음 스트로브가 온 적이 있음(1이면 설계 오류)
//
// 타이밍(50MHz, SAMPLE_DIV=50 -> 1MSPS): 스트로브 1클록 + ROM 래치 1클록 + CORDIC 18클록
//   이라서 샘플 간격 50클록 안에 충분히 끝난다.
//
// 7단계 변경: ROM 읽기 레지스터(rom_q)를 리셋 없는 별도 always 블록으로 옮겼다.
//   비동기 리셋이 걸린 레지스터로 읽으면 Quartus가 M10K로 추론하지 못하고 로직으로 펼칠 수 있다.
//   ROM은 매 클록 현재 주소를 읽고, CORDIC에 들어가는 값은 이전 구조와 클록 단위로 같다.
//   리셋 중 rom_q 값은 start_r이 0이라 쓰이지 않는다.
//   Quartus는 $readmemh 경로를 프로젝트 폴더 기준으로 찾는다.
//==============================================================================

module pdw_frontend #(
    parameter ROM_FILE   = "sim_data/iq_rom_s1.hex",
    parameter N_SAMPLES  = 8192,
    parameter SAMPLE_DIV = 50
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               run,
    output reg                meas_valid,
    output reg  [12:0]        meas_idx,
    output reg  signed [15:0] meas_mag,
    output reg  signed [15:0] meas_phase,
    output reg                rom_done,
    output reg                overrun
);

    //--------------------------------------------------------------------
    // 합성 신호 ROM: 32비트 = {I[15:0], Q[15:0]}  (M10K 추론 템플릿: 동기 읽기, 리셋 없음)
    //--------------------------------------------------------------------
    (* ramstyle = "M10K" *) reg [31:0] rom [0:N_SAMPLES-1];
    initial $readmemh(ROM_FILE, rom);

    reg [7:0]  div_cnt;      // 스트로브 분주기
    reg [13:0] rd_idx;       // 다음에 읽을 샘플 번호 (N_SAMPLES까지 셀 수 있게 14비트)
    reg [12:0] cur_idx;      // CORDIC에 들어간 샘플 번호
    reg [31:0] rom_q;        // ROM 읽기 결과
    reg        start_r;      // CORDIC start 펄스
    reg        busy_r;       // CORDIC 진행 중

    // 매 클록 현재 주소(rd_idx)를 읽는다. 스트로브 에지에서 rom_q = rom[k], rd_idx = k+1이 되고,
    // 다음 에지에서 CORDIC이 start와 함께 rom_q(= rom[k])를 가져간다. rom_q는 그 에지에 rom[k+1]로
    // 바뀌지만 CORDIC은 바뀌기 전 값을 쓴다. 읽기에 run 등 제어 신호가 끼지 않아 시뮬레이터 경쟁이 없다.
    always @(posedge clk) begin
        rom_q <= rom[rd_idx[12:0]];
    end

    wire               cordic_done;
    wire signed [15:0] cordic_x, cordic_z;

    cordic_engine u_cordic (
        .clk   (clk),
        .rst_n (rst_n),
        .start (start_r),
        .mode  (1'b1),                 // Vectoring
        .x_in  (rom_q[31:16]),         // I
        .y_in  (rom_q[15:0]),          // Q
        .z_in  (16'sd0),
        .x_out (cordic_x),             // K*|s|
        .y_out (),
        .z_out (cordic_z),             // 위상
        .done  (cordic_done),
        .busy  ()
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt    <= 8'd0;
            rd_idx     <= 14'd0;
            cur_idx    <= 13'd0;
            start_r    <= 1'b0;
            busy_r     <= 1'b0;
            meas_valid <= 1'b0;
            meas_idx   <= 13'd0;
            meas_mag   <= 16'sd0;
            meas_phase <= 16'sd0;
            rom_done   <= 1'b0;
            overrun    <= 1'b0;
        end else begin
            start_r    <= 1'b0;
            meas_valid <= 1'b0;

            // 샘플 스트로브: run이 켜져 있고 읽을 샘플이 남아 있는 동안 SAMPLE_DIV 클록 주기
            if (run && (rd_idx < N_SAMPLES)) begin
                if (div_cnt == SAMPLE_DIV - 1) div_cnt <= 8'd0;
                else                           div_cnt <= div_cnt + 8'd1;

                if (div_cnt == 8'd0) begin
                    cur_idx <= rd_idx[12:0];
                    rd_idx  <= rd_idx + 14'd1;
                    start_r <= 1'b1;          // 다음 클록에 CORDIC이 start를 받는다
                    if (busy_r) overrun <= 1'b1;
                end
            end

            // CORDIC 진행 표시와 결과 래치
            if (start_r) busy_r <= 1'b1;
            if (cordic_done) begin
                busy_r     <= 1'b0;
                meas_valid <= 1'b1;
                meas_idx   <= cur_idx;
                meas_mag   <= cordic_x;
                meas_phase <= cordic_z;
            end

            // 마지막 샘플까지 끝났는지
            if ((rd_idx == N_SAMPLES) && !busy_r && !start_r && !meas_valid)
                rom_done <= 1'b1;
        end
    end

endmodule
