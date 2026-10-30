// KAL-003 MVP: runtime-configurable, two-state Kalman filter with one measurement.
// Numeric contract:
//   state, measurement, gain: signed 16-bit, 15 fractional bits (raw * 2^-15)
//   F and H coefficients:     signed 16-bit, 14 fractional bits (raw * 2^-14)
//   P, Q, R covariance:       signed 40-bit, 30 fractional bits (raw * 2^-30)
//   Signed products are kept in 64-bit intermediates; rescaling rounds nearest,
//   ties away from zero. State/gain saturate to signed 16-bit; covariance to 40-bit.
//   This MVP uses P_new = P_pred - K*(H*P_pred), algebraically equivalent to
//   (I-KH)P_pred for exact arithmetic. Joseph form is a planned robustness upgrade.
//
// Scope: MEAS_DIM=1 only in this first deadline-driven implementation.
// Configuration is latched when start is accepted. init_valid loads initial state/P
// while idle. A transaction is committed only after all calculations succeed.
// This file should be compiled and tested with the accompanying testbench before
// integration; it is a first implementation milestone, not yet board-validated.

module kalman_2state_ip (
    input  logic clk,
    input  logic rst_n,

    input  logic init_valid,
    input  logic signed [15:0] init_x0,
    input  logic signed [15:0] init_x1,
    input  logic signed [39:0] init_p00,
    input  logic signed [39:0] init_p01,
    input  logic signed [39:0] init_p11,

    input  logic start,
    output logic busy,
    output logic done,
    output logic error_status,
    output logic overflow_seen,

    // F coefficients: Q2.14
    input  logic signed [15:0] cfg_f00, cfg_f01, cfg_f10, cfg_f11,
    // Q covariance: Q10.30
    input  logic signed [39:0] cfg_q00, cfg_q01, cfg_q11,
    // H coefficients: Q2.14; single measurement row [h0 h1]
    input  logic signed [15:0] cfg_h0, cfg_h1,
    // Scalar measurement-noise variance R: Q10.30
    input  logic signed [39:0] cfg_r,
    // Measurement: Q1.15
    input  logic signed [15:0] measurement,

    output logic signed [15:0] state0,
    output logic signed [15:0] state1,
    output logic signed [39:0] p00,
    output logic signed [39:0] p01,
    output logic signed [39:0] p11
);

    typedef enum logic [3:0] {
        ST_IDLE, ST_PRED_X, ST_PRED_P1, ST_PRED_P2,
        ST_INNOV, ST_INNOV_COV, ST_GAIN, ST_UPDATE_X,
        ST_UPDATE_P, ST_COMMIT, ST_ERROR
    } state_t;

    state_t state;

    logic initialized;
    logic signed [15:0] f00_r, f01_r, f10_r, f11_r;
    logic signed [39:0] q00_r, q01_r, q11_r, r_r;
    logic signed [15:0] h0_r, h1_r, z_r;

    logic signed [15:0] x0_pred, x1_pred, x0_next, x1_next;
    logic signed [39:0] a00, a01, a10, a11;
    logic signed [39:0] pp00, pp01, pp11;
    logic signed [39:0] hp0, hp1, s_r;
    logic signed [15:0] innov_r;
    logic signed [15:0] k0_r, k1_r;
    logic signed [39:0] p00_next, p01_next, p11_next;

    logic signed [63:0] t0, t1, t2, t3, wide_sum;
    // Explicitly widen covariance numerators before the Q30-to-Q15 gain shift.
    logic signed [63:0] gain_num0, gain_num1;
    logic local_overflow;
    // Diagnostic only: identifies the last overflow condition detected.
    logic [7:0] overflow_stage;

    function automatic logic signed [63:0] round_shift(
        input logic signed [63:0] value,
        input integer shift
    );
        logic signed [63:0] mag;
        logic signed [63:0] rounded_mag;
        begin
            if (shift <= 0) begin
                round_shift = value;
            end else if (value < 0) begin
                mag = -value;
                rounded_mag = (mag + (64'sd1 <<< (shift-1))) >>> shift;
                round_shift = -rounded_mag;
            end else begin
                round_shift = (value + (64'sd1 <<< (shift-1))) >>> shift;
            end
        end
    endfunction

    function automatic logic signed [15:0] sat16(
        input logic signed [63:0] value
    );
        begin
            if (value > 64'sd32767) sat16 = 16'sd32767;
            else if (value < -64'sd32768) sat16 = -16'sd32768;
            else sat16 = value[15:0];
        end
    endfunction

    function automatic logic signed [39:0] sat40(
        input logic signed [63:0] value
    );
        begin
            if (value > 64'sd549755813887) sat40 = 40'sh7FFFFFFFFF;
            else if (value < -64'sd549755813888) sat40 = 40'sh8000000000;
            else sat40 = value[39:0];
        end
    endfunction

    function automatic logic fits40(input logic signed [63:0] value);
        begin
            fits40 = (value <= 64'sd549755813887) &&
                     (value >= -64'sd549755813888);
        end
    endfunction

    always_comb begin
        busy = (state != ST_IDLE);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            initialized    <= 1'b0;
            done           <= 1'b0;
            error_status   <= 1'b0;
            overflow_seen  <= 1'b0;
            overflow_stage <= 8'd0;
            state0         <= '0;
            state1         <= '0;
            p00            <= '0;
            p01            <= '0;
            p11            <= '0;
            x0_pred        <= '0;
            x1_pred        <= '0;
            x0_next        <= '0;
            x1_next        <= '0;
            a00            <= '0;
            a01            <= '0;
            a10            <= '0;
            a11            <= '0;
            pp00           <= '0;
            pp01           <= '0;
            pp11           <= '0;
            hp0             <= '0;
            hp1             <= '0;
            s_r             <= '0;
            innov_r         <= '0;
            k0_r            <= '0;
            k1_r            <= '0;
            p00_next        <= '0;
            p01_next        <= '0;
            p11_next        <= '0;
            f00_r <= '0; f01_r <= '0; f10_r <= '0; f11_r <= '0;
            q00_r <= '0; q01_r <= '0; q11_r <= '0; r_r <= '0;
            h0_r <= '0; h1_r <= '0; z_r <= '0;
            gain_num0 <= '0;
            gain_num1 <= '0;
        end else begin
            done <= 1'b0;
            if (state == ST_IDLE) begin
                if (init_valid) begin
                    state0      <= init_x0;
                    state1      <= init_x1;
                    p00         <= init_p00;
                    p01         <= init_p01;
                    p11         <= init_p11;
                    initialized <= 1'b1;
                    error_status <= 1'b0;
                    overflow_seen <= 1'b0;
                end else if (start && initialized) begin
                    f00_r <= cfg_f00; f01_r <= cfg_f01;
                    f10_r <= cfg_f10; f11_r <= cfg_f11;
                    q00_r <= cfg_q00; q01_r <= cfg_q01; q11_r <= cfg_q11;
                    h0_r <= cfg_h0; h1_r <= cfg_h1;
                    r_r <= cfg_r;
                    z_r <= measurement;
                    error_status <= 1'b0;
                    overflow_seen <= 1'b0;
                    overflow_stage <= 8'd0;
                    state <= ST_PRED_X;
                end else if (start && !initialized) begin
                    error_status <= 1'b1;
                    done <= 1'b1;
                end
            end else begin
                case (state)
                    ST_PRED_X: begin
                        t0 = 64'($signed(f00_r)) * 64'($signed(state0));
                        t1 = 64'($signed(f01_r)) * 64'($signed(state1));
                        wide_sum = round_shift(t0 + t1, 14);
                        x0_pred <= sat16(wide_sum);
                        if (!((wide_sum <= 32767) && (wide_sum >= -32768)))
                            begin overflow_seen <= 1'b1; overflow_stage <= 8'd1; end

                        t2 = 64'($signed(f10_r)) * 64'($signed(state0));
                        t3 = 64'($signed(f11_r)) * 64'($signed(state1));
                        wide_sum = round_shift(t2 + t3, 14);
                        x1_pred <= sat16(wide_sum);
                        if (!((wide_sum <= 32767) && (wide_sum >= -32768)))
                            begin overflow_seen <= 1'b1; overflow_stage <= 8'd2; end
                        state <= ST_PRED_P1;
                    end

                    ST_PRED_P1: begin
                        // A = F*P. Inputs P are Q30, F is Q14; result returns to Q30.
                        t0 = round_shift(64'($signed(f00_r)) * 64'($signed(p00)), 14);
                        t1 = round_shift(64'($signed(f01_r)) * 64'($signed(p01)), 14);
                        wide_sum = t0 + t1;
                        a00 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd3; end

                        t0 = round_shift(64'($signed(f00_r)) * 64'($signed(p01)), 14);
                        t1 = round_shift(64'($signed(f01_r)) * 64'($signed(p11)), 14);
                        wide_sum = t0 + t1;
                        a01 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd4; end

                        t0 = round_shift(64'($signed(f10_r)) * 64'($signed(p00)), 14);
                        t1 = round_shift(64'($signed(f11_r)) * 64'($signed(p01)), 14);
                        wide_sum = t0 + t1;
                        a10 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd5; end

                        t0 = round_shift(64'($signed(f10_r)) * 64'($signed(p01)), 14);
                        t1 = round_shift(64'($signed(f11_r)) * 64'($signed(p11)), 14);
                        wide_sum = t0 + t1;
                        a11 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd6; end
                        state <= ST_PRED_P2;
                    end

                    ST_PRED_P2: begin
                        // Ppred = A*F^T + Q
                        t0 = round_shift(64'($signed(a00)) * 64'($signed(f00_r)), 14);
                        t1 = round_shift(64'($signed(a01)) * 64'($signed(f01_r)), 14);
                        wide_sum = t0 + t1 + $signed(q00_r);
                        pp00 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd7; end

                        t0 = round_shift(64'($signed(a00)) * 64'($signed(f10_r)), 14);
                        t1 = round_shift(64'($signed(a01)) * 64'($signed(f11_r)), 14);
                        wide_sum = t0 + t1 + $signed(q01_r);
                        pp01 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd8; end

                        t0 = round_shift(64'($signed(a10)) * 64'($signed(f10_r)), 14);
                        t1 = round_shift(64'($signed(a11)) * 64'($signed(f11_r)), 14);
                        wide_sum = t0 + t1 + $signed(q11_r);
                        pp11 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd9; end
                        state <= ST_INNOV;
                    end

                    ST_INNOV: begin
                        // predicted measurement = H*xpred, Q14*Q15 -> Q15 after >>14
                        t0 = 64'($signed(h0_r)) * 64'($signed(x0_pred));
                        t1 = 64'($signed(h1_r)) * 64'($signed(x1_pred));
                        wide_sum = round_shift(t0 + t1, 14);
                        wide_sum = $signed(z_r) - wide_sum;
                        innov_r <= sat16(wide_sum);
                        if (!((wide_sum <= 32767) && (wide_sum >= -32768)))
                            begin overflow_seen <= 1'b1; overflow_stage <= 8'd10; end

                        // HP = H*Ppred; return to Q30.
                        t0 = round_shift(64'($signed(h0_r)) * 64'($signed(pp00)), 14);
                        t1 = round_shift(64'($signed(h1_r)) * 64'($signed(pp01)), 14);
                        wide_sum = t0 + t1;
                        hp0 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd11; end

                        t0 = round_shift(64'($signed(h0_r)) * 64'($signed(pp01)), 14);
                        t1 = round_shift(64'($signed(h1_r)) * 64'($signed(pp11)), 14);
                        wide_sum = t0 + t1;
                        hp1 <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd12; end
                        state <= ST_INNOV_COV;
                    end

                    ST_INNOV_COV: begin
                        // S = H*Ppred*H^T + R; Q14*Q30 -> Q30 after >>14.
                        t0 = round_shift(64'($signed(h0_r)) * 64'($signed(hp0)), 14);
                        t1 = round_shift(64'($signed(h1_r)) * 64'($signed(hp1)), 14);
                        wide_sum = t0 + t1 + $signed(r_r);
                        s_r <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd13; end
                        if (wide_sum <= 0) begin
                            error_status <= 1'b1;
                            state <= ST_ERROR;
                        end else begin
                            state <= ST_GAIN;
                        end
                    end

                    ST_GAIN: begin
                        // K raw Q15 = (PH^T raw Q30 * 2^15) / S raw Q30.
                        // PH^T is [hp0 hp1]^T because P is symmetric.
                        // Widen BEFORE shifting: a shift keeps the left operand's width.
                        gain_num0 = {{24{hp0[39]}}, hp0};
                        gain_num1 = {{24{hp1[39]}}, hp1};
                        gain_num0 = gain_num0 <<< 15;
                        gain_num1 = gain_num1 <<< 15;

                        t0 = gain_num0 / $signed(s_r);
                        t1 = gain_num1 / $signed(s_r);
                        k0_r <= sat16(t0);
                        k1_r <= sat16(t1);
                        if (!((t0 <= 32767) && (t0 >= -32768)) ||
                            !((t1 <= 32767) && (t1 >= -32768)))
                            begin overflow_seen <= 1'b1; overflow_stage <= 8'd14; end
                        state <= ST_UPDATE_X;
                    end

                    ST_UPDATE_X: begin
                        // xnew = xpred + K*innovation; Q15*Q15 -> Q30, >>15 to Q15.
                        t0 = round_shift(64'($signed(k0_r)) * 64'($signed(innov_r)), 15);
                        wide_sum = $signed(x0_pred) + t0;
                        x0_next <= sat16(wide_sum);
                        if (!((wide_sum <= 32767) && (wide_sum >= -32768)))
                            begin overflow_seen <= 1'b1; overflow_stage <= 8'd15; end

                        t1 = round_shift(64'($signed(k1_r)) * 64'($signed(innov_r)), 15);
                        wide_sum = $signed(x1_pred) + t1;
                        x1_next <= sat16(wide_sum);
                        if (!((wide_sum <= 32767) && (wide_sum >= -32768)))
                            begin overflow_seen <= 1'b1; overflow_stage <= 8'd16; end
                        state <= ST_UPDATE_P;
                    end

                    ST_UPDATE_P: begin
                        // Pnew = Ppred - K*(H*Ppred). Q15*Q30 -> Q45, >>15 to Q30.
                        t0 = round_shift(64'($signed(k0_r)) * 64'($signed(hp0)), 15);
                        wide_sum = $signed(pp00) - t0;
                        p00_next <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd17; end

                        t1 = round_shift(64'($signed(k0_r)) * 64'($signed(hp1)), 15);
                        wide_sum = $signed(pp01) - t1;
                        p01_next <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd18; end

                        t2 = round_shift(64'($signed(k1_r)) * 64'($signed(hp1)), 15);
                        wide_sum = $signed(pp11) - t2;
                        p11_next <= sat40(wide_sum);
                        if (!fits40(wide_sum)) begin overflow_seen <= 1'b1; overflow_stage <= 8'd19; end
                        state <= ST_COMMIT;
                    end

                    ST_COMMIT: begin
                        state0 <= x0_next;
                        state1 <= x1_next;
                        p00 <= p00_next;
                        p01 <= p01_next;
                        p11 <= p11_next;
                        done <= 1'b1;
                        state <= ST_IDLE;
                    end

                    ST_ERROR: begin
                        // Preserve the last valid state/covariance on failed update.
                        done <= 1'b1;
                        state <= ST_IDLE;
                    end

                    default: begin
                        error_status <= 1'b1;
                        done <= 1'b1;
                        state <= ST_IDLE;
                    end
                endcase
            end
        end
    end
endmodule
