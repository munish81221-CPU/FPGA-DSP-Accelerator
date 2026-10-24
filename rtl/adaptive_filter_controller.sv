`timescale 1ns/1ps

// Shared four-tap LMS/NLMS adaptive-filter controller.
// Samples, desired, mu and weights are signed Q1.15.
// The engine accumulator is signed Q*.30. NLMS energy is kept in Q*.30
// and epsilon is supplied in the same raw Q30 scale (34-bit signed).
// NLMS uses one division per tap to avoid requiring a 32x16 multiplier:
// correction_q15 = (mu_error_times_x)_q15 / energy_q30, scaled by 2^30.
module adaptive_filter_controller (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic mode,                 // 0 = LMS, 1 = NLMS; latched per iteration
    input  logic signed [15:0] x0, x1, x2, x3,
    input  logic signed [15:0] desired,
    input  logic signed [15:0] mu,
    input  logic signed [33:0] epsilon_q30, // positive regularization, raw Q30

    output logic done,
    output logic busy,
    output logic signed [15:0] y_out,
    output logic signed [15:0] error_out,
    output logic signed [15:0] w0, w1, w2, w3,
    output logic overflow_seen,
    output logic error_status,

    // Shared arithmetic-engine command channel
    output logic cmd_valid,
    input  logic cmd_ready,
    output logic [2:0] opcode,
    output logic signed [15:0] operand_a,
    output logic signed [15:0] operand_b,
    input  logic result_valid,
    input  logic signed [15:0] result,
    input  logic engine_overflow,
    input  logic signed [39:0] accumulator_value
);
    localparam logic MODE_LMS  = 1'b0;
    localparam logic MODE_NLMS = 1'b1;
    localparam logic [2:0] OP_MUL       = 3'b000;
    localparam logic [2:0] OP_MAC       = 3'b001;
    localparam logic [2:0] OP_ADD       = 3'b010;
    localparam logic [2:0] OP_SUB       = 3'b011;
    localparam logic [2:0] OP_ACC_CLEAR = 3'b100;
    localparam logic [2:0] OP_ACC_READ  = 3'b101;

    typedef enum logic [5:0] {
        ST_IDLE,
        ST_ISSUE_CLEAR_Y, ST_WAIT_CLEAR_Y,
        ST_ISSUE_MAC_Y, ST_WAIT_MAC_Y,
        ST_ISSUE_READ_Y, ST_WAIT_READ_Y,
        ST_ISSUE_ERROR, ST_WAIT_ERROR,
        ST_ISSUE_MUE, ST_WAIT_MUE,
        ST_ISSUE_CLEAR_E, ST_WAIT_CLEAR_E,
        ST_ISSUE_MAC_E, ST_WAIT_MAC_E,
        ST_ISSUE_READ_E, ST_WAIT_READ_E,
        ST_ISSUE_CORR, ST_WAIT_CORR,
        ST_ISSUE_DIV, ST_WAIT_DIV,
        ST_ISSUE_WADD, ST_WAIT_WADD,
        ST_FINISH
    } state_t;

    state_t state;
    logic mode_q;
    logic signed [15:0] x_q [0:3];
    logic signed [15:0] desired_q, mu_q, error_q, mu_error_q, correction_q;
    logic signed [15:0] weights [0:3];
    logic [1:0] tap_idx;
    logic signed [39:0] energy_raw_q;
    logic signed [34:0] energy_plus_epsilon;
    logic signed [33:0] denominator_q30;
    logic signed [15:0] div_quotient;
    logic div_start, div_busy, div_done, div_by_zero, div_overflow;
    logic signed [15:0] div_numerator;
    logic signed [33:0] div_denominator;
    integer i;

    assign w0 = weights[0];
    assign w1 = weights[1];
    assign w2 = weights[2];
    assign w3 = weights[3];

    // Sum in 35 bits so epsilon addition cannot silently wrap.
    always_comb begin
        energy_plus_epsilon = $signed({1'b0, accumulator_value[33:0]}) +
                              $signed({epsilon_q30[33], epsilon_q30});
        div_numerator   = correction_q;
        div_denominator = denominator_q30;
    end

    divider_unit #(
    .NUM_W(16),
    .DEN_W(34),
    .QUOT_W(16),
    .SCALE_SHIFT(30)

    ) u_nlms_divider (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .numerator(div_numerator), .denominator(div_denominator),
        .busy(div_busy), .done(div_done), .quotient(div_quotient),
        .div_by_zero(div_by_zero), .overflow(div_overflow)
    );

    always_comb begin
        cmd_valid = 1'b0;
        opcode    = OP_ACC_CLEAR;
        operand_a = '0;
        operand_b = '0;
        div_start = 1'b0;

        case (state)
            ST_ISSUE_CLEAR_Y, ST_ISSUE_CLEAR_E: begin
                cmd_valid = 1'b1;
                opcode = OP_ACC_CLEAR;
            end
            ST_ISSUE_MAC_Y: begin
                cmd_valid = 1'b1;
                opcode = OP_MAC;
                operand_a = weights[tap_idx];
                operand_b = x_q[tap_idx];
            end
            ST_ISSUE_MAC_E: begin
                cmd_valid = 1'b1;
                opcode = OP_MAC;
                operand_a = x_q[tap_idx];
                operand_b = x_q[tap_idx];
            end
            ST_ISSUE_READ_Y, ST_ISSUE_READ_E: begin
                cmd_valid = 1'b1;
                opcode = OP_ACC_READ;
            end
            ST_ISSUE_ERROR: begin
                cmd_valid = 1'b1;
                opcode = OP_SUB;
                operand_a = desired_q;
                operand_b = y_out;
            end
            ST_ISSUE_MUE: begin
                cmd_valid = 1'b1;
                opcode = OP_MUL;
                operand_a = mu_q;
                operand_b = error_q;
            end
            ST_ISSUE_CORR: begin
                cmd_valid = 1'b1;
                opcode = OP_MUL;
                operand_a = mu_error_q;
                operand_b = x_q[tap_idx];
            end
            ST_ISSUE_WADD: begin
                cmd_valid = 1'b1;
                opcode = OP_ADD;
                operand_a = weights[tap_idx];
                operand_b = correction_q;
            end
            ST_ISSUE_DIV: begin
                if (!div_busy) div_start = 1'b1;
            end
            default: ;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            mode_q <= MODE_LMS;
            busy <= 1'b0;
            done <= 1'b0;
            y_out <= '0;
            error_out <= '0;
            desired_q <= '0;
            mu_q <= '0;
            error_q <= '0;
            mu_error_q <= '0;
            correction_q <= '0;
            tap_idx <= '0;
            energy_raw_q <= '0;
            denominator_q30 <= '0;
            overflow_seen <= 1'b0;
            error_status <= 1'b0;
            for (i = 0; i < 4; i = i + 1) begin
                x_q[i] <= '0;
                weights[i] <= '0;
            end
        end else begin
            done <= 1'b0;
            // During energy accumulation/read, the engine's 16-bit result view may
            // saturate even though the 40-bit accumulator remains valid. Ignore that
            // expected conversion saturation; use accumulator_value for NLMS energy.
            if (result_valid && engine_overflow &&
                (state != ST_WAIT_MAC_E) && (state != ST_WAIT_READ_E))
                overflow_seen <= 1'b1;
            if (div_done && div_overflow) overflow_seen <= 1'b1;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        mode_q <= mode;
                        x_q[0] <= x0; x_q[1] <= x1; x_q[2] <= x2; x_q[3] <= x3;
                        desired_q <= desired;
                        mu_q <= mu;
                        y_out <= '0;
                        error_out <= '0;
                        error_q <= '0;
                        mu_error_q <= '0;
                        correction_q <= '0;
                        tap_idx <= 0;
                        energy_raw_q <= '0;
                        denominator_q30 <= '0;
                        overflow_seen <= 1'b0;
                        error_status <= 1'b0;
                        busy <= 1'b1;
                        state <= ST_ISSUE_CLEAR_Y;
                    end
                end
                ST_ISSUE_CLEAR_Y: if (cmd_ready) state <= ST_WAIT_CLEAR_Y;
                ST_WAIT_CLEAR_Y: if (result_valid) begin tap_idx <= 0; state <= ST_ISSUE_MAC_Y; end
                ST_ISSUE_MAC_Y: if (cmd_ready) state <= ST_WAIT_MAC_Y;
                ST_WAIT_MAC_Y: if (result_valid) begin
                    if (tap_idx == 3) state <= ST_ISSUE_READ_Y;
                    else begin tap_idx <= tap_idx + 1'b1; state <= ST_ISSUE_MAC_Y; end
                end
                ST_ISSUE_READ_Y: if (cmd_ready) state <= ST_WAIT_READ_Y;
                ST_WAIT_READ_Y: if (result_valid) begin y_out <= result; state <= ST_ISSUE_ERROR; end
                ST_ISSUE_ERROR: if (cmd_ready) state <= ST_WAIT_ERROR;
                ST_WAIT_ERROR: if (result_valid) begin
                    error_q <= result; error_out <= result; state <= ST_ISSUE_MUE;
                end
                ST_ISSUE_MUE: if (cmd_ready) state <= ST_WAIT_MUE;
                ST_WAIT_MUE: if (result_valid) begin
                    mu_error_q <= result;
                    tap_idx <= 0;
                    if (mode_q == MODE_NLMS) state <= ST_ISSUE_CLEAR_E;
                    else state <= ST_ISSUE_CORR;
                end
                ST_ISSUE_CLEAR_E: if (cmd_ready) state <= ST_WAIT_CLEAR_E;
                ST_WAIT_CLEAR_E: if (result_valid) begin tap_idx <= 0; state <= ST_ISSUE_MAC_E; end
                ST_ISSUE_MAC_E: if (cmd_ready) state <= ST_WAIT_MAC_E;
                ST_WAIT_MAC_E: if (result_valid) begin
                    if (tap_idx == 3) state <= ST_ISSUE_READ_E;
                    else begin tap_idx <= tap_idx + 1'b1; state <= ST_ISSUE_MAC_E; end
                end
                ST_ISSUE_READ_E: if (cmd_ready) state <= ST_WAIT_READ_E;
                ST_WAIT_READ_E: if (result_valid) begin
                    energy_raw_q <= accumulator_value;

                    if (epsilon_q30 <= 0) begin
                        error_status <= 1'b1;
                        state <= ST_FINISH;
                    end else begin
                        if (energy_plus_epsilon[34] || energy_plus_epsilon[33]) begin
                            denominator_q30 <= {1'b0, {33{1'b1}}};
                            overflow_seen <= 1'b1;
                        end else begin
                            denominator_q30 <= energy_plus_epsilon[33:0];
                        end

                        // Energy accumulation finished at tap 3.
                        // Restart the correction loop at tap 0.
                        tap_idx <= 2'd0;

                        state <= ST_ISSUE_CORR;
                    end
                end
                ST_ISSUE_CORR: if (cmd_ready) state <= ST_WAIT_CORR;
                ST_WAIT_CORR: if (result_valid) begin
                    correction_q <= result;
                    if (mode_q == MODE_NLMS) state <= ST_ISSUE_DIV;
                    else state <= ST_ISSUE_WADD;
                end
                ST_ISSUE_DIV: if (!div_busy) state <= ST_WAIT_DIV;
                ST_WAIT_DIV: if (div_done) begin
                    if (div_by_zero) begin error_status <= 1'b1; correction_q <= '0; end
                    else correction_q <= div_quotient;
                    state <= ST_ISSUE_WADD;
                end
                ST_ISSUE_WADD: if (cmd_ready) state <= ST_WAIT_WADD;
                ST_WAIT_WADD: if (result_valid) begin
                    weights[tap_idx] <= result;
                    if (tap_idx == 3) state <= ST_FINISH;
                    else begin tap_idx <= tap_idx + 1'b1; state <= ST_ISSUE_CORR; end
                end
                ST_FINISH: begin busy <= 1'b0; done <= 1'b1; state <= ST_IDLE; end
                default: begin state <= ST_IDLE; busy <= 1'b0; error_status <= 1'b1; end
            endcase
        end
    end
endmodule
