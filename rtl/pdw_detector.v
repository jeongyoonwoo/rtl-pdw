//==============================================================================
// pdw_detector.v   (rtl-pdw 3~4단계)
// 샘플별 크기·위상(pdw_frontend 출력)에서 펄스를 검출하고 PDW를 만든다.
//
// 펄스 검출: 두 문턱(TH_HI 시작, TH_LO 유지) + 종료 유예(연속 END_LOW 샘플이 TH_LO 미만이면 종료)
//   PW >= MIN_W이면 PDW를 내고, 더 짧으면 글리치로 버린다(glitch_cnt 증가).
// 측정 (모두 정수 연산, 곱셈기·나눗셈기 없음)
//   TOA  : 처음 TH_HI를 넘은 샘플 번호 (샘플 단위)
//   PW   : (마지막으로 TH_LO 이상이던 샘플) - TOA + 1
//   PA   : PW >= MIN_FREQ_W이면 상대 인덱스 [FREQ_SKIP, FREQ_SKIP+8) 8샘플의 크기 평균,
//          그보다 짧으면 피크 (피크는 잡음 위의 최댓값이라 위로 치우친다)
//   FREQ : 같은 창에서 위상 차(16비트 모듈로)의 합 >>> 3. PW >= MIN_FREQ_W일 때만 유효
//          주파수 f = FREQ / 32768 * (Fs/2)
//   FLAGS: bit0 = 주파수 유효(PW >= MIN_FREQ_W), bit1 = 크기 포화(피크가 32767)
//
// 입력 meas_mag는 K배 스케일 크기(항상 0 이상), meas_phase는 Q1.15 위상이다.
// 알고리즘은 gen_pdw_signal.py의 비트 단위 참조 모델(run_model)과 같다.
// 한계: 펄스 폭 카운터가 16비트라 65,535샘플보다 긴 펄스는 감긴다.
//==============================================================================

module pdw_detector #(
    parameter TH_HI     = 3000,
    parameter TH_LO     = 2000,
    parameter MIN_W     = 4,
    parameter END_LOW   = 2,        // 1~3
    parameter FREQ_SKIP = 3,        // 창 시작 상대 인덱스 (가장자리 샘플을 피하려고 3샘플 건너뜀)
    parameter FREQ_WIN  = 8         // 8로 고정(3비트 시프트로 평균)
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               meas_valid,
    input  wire signed [15:0] meas_mag,
    input  wire signed [15:0] meas_phase,

    output reg                pdw_valid,     // 1클록 펄스
    output reg         [31:0] pdw_toa,
    output reg         [15:0] pdw_pw,
    output reg         [15:0] pdw_pa,
    output reg         [15:0] pdw_freq,
    output reg         [15:0] pdw_flags,
    output reg         [15:0] glitch_cnt     // 버려진 펄스 수
);

    localparam MIN_FREQ_W = FREQ_SKIP + FREQ_WIN;

    reg                in_pulse;
    reg         [31:0] sample_cnt;           // 지금 처리하는 샘플 번호
    reg         [31:0] toa;
    reg         [15:0] r;                    // 펄스 안에서 직전 샘플의 상대 인덱스
    reg         [15:0] last_hi_r;            // 마지막으로 TH_LO 이상이던 샘플의 상대 인덱스
    reg         [15:0] peak;
    reg         [1:0]  low;                  // 연속으로 TH_LO 미만인 샘플 수
    reg signed  [18:0] fsum;                 // 위상 차 8개 합 (|합| <= 262144)
    reg         [17:0] msum;                 // 크기 8개 합 (<= 8*32767)
    reg signed  [15:0] prev_phase;

    wire signed [15:0] dphi     = meas_phase - prev_phase;     // 16비트 모듈로 뺄셈
    wire        [15:0] mag_u    = meas_mag[15:0];
    wire        [15:0] cur_r    = r + 16'd1;                   // 지금 샘플의 상대 인덱스
    wire               is_hi    = (meas_mag >= TH_LO);
    wire        [1:0]  low_n    = is_hi ? 2'd0 : (low + 2'd1);
    wire               ending   = in_pulse && !is_hi && (low_n >= END_LOW);
    wire        [15:0] pw_c     = last_hi_r + 16'd1;
    wire               fvalid   = (pw_c >= MIN_FREQ_W);
    wire signed [18:0] fsum_avg = fsum >>> 3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_pulse   <= 1'b0;
            sample_cnt <= 32'd0;
            toa        <= 32'd0;
            r          <= 16'd0;
            last_hi_r  <= 16'd0;
            peak       <= 16'd0;
            low        <= 2'd0;
            fsum       <= 19'sd0;
            msum       <= 18'd0;
            prev_phase <= 16'sd0;
            pdw_valid  <= 1'b0;
            pdw_toa    <= 32'd0;
            pdw_pw     <= 16'd0;
            pdw_pa     <= 16'd0;
            pdw_freq   <= 16'd0;
            pdw_flags  <= 16'd0;
            glitch_cnt <= 16'd0;
        end else begin
            pdw_valid <= 1'b0;

            if (meas_valid) begin
                prev_phase <= meas_phase;
                sample_cnt <= sample_cnt + 32'd1;

                if (!in_pulse) begin
                    // IDLE: 문턱을 넘으면 펄스 시작
                    if (meas_mag >= TH_HI) begin
                        in_pulse  <= 1'b1;
                        toa       <= sample_cnt;
                        r         <= 16'd0;
                        last_hi_r <= 16'd0;
                        peak      <= mag_u;
                        low       <= 2'd0;
                        fsum      <= 19'sd0;
                        msum      <= 18'd0;
                    end
                end else begin
                    // IN_PULSE
                    r <= cur_r;
                    if (is_hi) begin
                        last_hi_r <= cur_r;
                        low       <= 2'd0;
                        if (mag_u > peak) peak <= mag_u;
                    end else begin
                        low <= low_n;
                    end

                    // 주파수·PA 창: 상대 인덱스 [FREQ_SKIP, FREQ_SKIP + 8)
                    if ((cur_r >= FREQ_SKIP) && (cur_r < FREQ_SKIP + FREQ_WIN)) begin
                        fsum <= fsum + {{3{dphi[15]}}, dphi};
                        msum <= msum + {2'b00, mag_u};
                    end

                    // 종료: PDW 확정 또는 글리치 폐기
                    if (ending) begin
                        in_pulse <= 1'b0;
                        if (pw_c >= MIN_W) begin
                            pdw_valid <= 1'b1;
                            pdw_toa   <= toa;
                            pdw_pw    <= pw_c;
                            pdw_pa    <= fvalid ? {1'b0, msum[17:3]} : peak;
                            pdw_freq  <= fvalid ? fsum_avg[15:0] : 16'd0;
                            pdw_flags <= {14'd0, (peak >= 16'd32767), fvalid};
                        end else begin
                            glitch_cnt <= glitch_cnt + 16'd1;
                        end
                    end
                end
            end
        end
    end

endmodule
