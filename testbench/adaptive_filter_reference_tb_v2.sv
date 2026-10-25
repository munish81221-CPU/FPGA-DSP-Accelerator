`timescale 1ns/1ps

// Independent, bit-accurate reference testbench for the shared LMS/NLMS controller.
//
// The model below does NOT read internal DUT state. It maintains its own weights
// and independently models the documented arithmetic sequence:
//   1) Q30 MAC -> round-to-nearest/ties-away-from-zero -> saturate to Q1.15
//   2) desired - y -> signed Q1.15 saturation
//   3) mu * error -> Q30-to-Q15 rounding/saturation
//   4) mu_error * x[i] -> Q30-to-Q15 rounding/saturation
//   5) LMS: add correction and saturate
//      NLMS: divide correction_raw * 2^30 by energy_raw + epsilon_raw,
//            truncate toward zero, saturate quotient to signed 16 bits,
//            then add/saturate the weight.
//
// Samples, weights, desired, mu and controller outputs are raw signed Q1.15.
// Input energy and epsilon are raw Q30. The divider's FRACTION_BITS is 30.

module adaptive_filter_reference_tb;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic mode = 1'b0;

    logic signed [15:0] x0, x1, x2, x3, desired, mu;
    logic signed [33:0] epsilon_q30;

    logic done, busy;
    logic signed [15:0] y_out, error_out, w0, w1, w2, w3;
    logic overflow_seen, error_status;

    logic cmd_valid, cmd_ready;
    logic [2:0] opcode;
    logic signed [15:0] operand_a, operand_b, result;
    logic result_valid, engine_overflow, engine_busy;
    logic signed [39:0] accumulator_value;

    logic signed [15:0] ref_weights [0:3];
    logic signed [15:0] sample_vec [0:3];
    logic signed [15:0] exp_weights [0:3];
    logic signed [15:0] exp_y, exp_error;
    logic signed [15:0] exp_mu_error;
    logic signed [15:0] exp_correction;
    longint signed ref_energy;
    longint signed ref_denominator;
    longint signed ref_quotient;

    integer total_vectors = 0;
    integer total_checks = 0;
    logic [31:0] rng_state = 32'h51A7_C0DE;

    always #5 clk = ~clk;

    arithmetic_engine u_engine (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),
        .opcode(opcode),
        .operand_a(operand_a),
        .operand_b(operand_b),
        .result(result),
        .result_valid(result_valid),
        .busy(engine_busy),
        .overflow(engine_overflow),
        .accumulator_value(accumulator_value)
    );

    adaptive_filter_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .mode(mode),
        .x0(x0), .x1(x1), .x2(x2), .x3(x3),
        .desired(desired),
        .mu(mu),
        .epsilon_q30(epsilon_q30),
        .done(done),
        .busy(busy),
        .y_out(y_out),
        .error_out(error_out),
        .w0(w0), .w1(w1), .w2(w2), .w3(w3),
        .overflow_seen(overflow_seen),
        .error_status(error_status),
        .cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),
        .opcode(opcode),
        .operand_a(operand_a),
        .operand_b(operand_b),
        .result_valid(result_valid),
        .result(result),
        .engine_overflow(engine_overflow),
        .accumulator_value(accumulator_value)
    );

    // Q30 -> Q1.15 conversion: round to nearest, exact half-LSB ties
    // away from zero, then saturate. Use magnitude rounding for negative
    // values; subtracting half an LSB and then arithmetic-shifting would
    // incorrectly round exact negative integers one unit too negative.
    function automatic logic signed [15:0] ref_q30_to_q15(input longint signed value);
        longint signed magnitude;
        longint signed scaled_magnitude;
        longint signed scaled;
        begin
            if (value < 0) begin
                magnitude = -value;
                scaled_magnitude = (magnitude + 64'sd16384) >>> 15;
                scaled = -scaled_magnitude;
            end else begin
                scaled = (value + 64'sd16384) >>> 15;
            end

            if (scaled > 32767)
                ref_q30_to_q15 = 16'sd32767;
            else if (scaled < -32768)
                ref_q30_to_q15 = -16'sd32768;
            else
                ref_q30_to_q15 = scaled[15:0];
        end
    endfunction

    function automatic logic signed [15:0] ref_sat_q15(input longint signed value);
        begin
            if (value > 32767)
                ref_sat_q15 = 16'sd32767;
            else if (value < -32768)
                ref_sat_q15 = -16'sd32768;
            else
                ref_sat_q15 = value[15:0];
        end
    endfunction

    function automatic logic [31:0] lcg_next(input logic [31:0] state_in);
        // Deterministic 32-bit LCG; no simulator-dependent $urandom behavior.
        lcg_next = state_in * 32'd1664525 + 32'd1013904223;
    endfunction

    task automatic collect_samples;
        begin
            sample_vec[0] = x0;
            sample_vec[1] = x1;
            sample_vec[2] = x2;
            sample_vec[3] = x3;
        end
    endtask

    // Compute expected results using only testbench-owned state and input values.
    task automatic calculate_reference(input logic selected_mode);
        longint signed sum_q30;
        longint signed product_q30;
        longint signed correction_raw;
        longint signed weight_sum;
        integer k;
        begin
            collect_samples();

            // Filter output: sum of raw Q1.15 x Q1.15 products is raw Q30.
            sum_q30 = 0;
            for (k = 0; k < 4; k = k + 1)
                sum_q30 = sum_q30 +
                          (longint'($signed(ref_weights[k])) *
                           longint'($signed(sample_vec[k])));
            exp_y = ref_q30_to_q15(sum_q30);

            // Error subtraction uses a 17-bit conceptual result then Q1.15 saturation.
            exp_error = ref_sat_q15(longint'($signed(desired)) -
                                    longint'($signed(exp_y)));

            // Engine MUL: mu * error (Q30), rounded and saturated to Q1.15.
            product_q30 = longint'($signed(mu)) *
                          longint'($signed(exp_error));
            exp_mu_error = ref_q30_to_q15(product_q30);

            // Energy is accumulated without intermediate rounding in raw Q30.
            ref_energy = 0;
            for (k = 0; k < 4; k = k + 1)
                ref_energy = ref_energy +
                             (longint'($signed(sample_vec[k])) *
                              longint'($signed(sample_vec[k])));
            ref_denominator = ref_energy + longint'($signed(epsilon_q30));

            for (k = 0; k < 4; k = k + 1) begin
                // Engine MUL: mu_error * x[k], Q30 -> rounded/saturated Q1.15.
                product_q30 = longint'($signed(exp_mu_error)) *
                              longint'($signed(sample_vec[k]));
                correction_raw = longint'($signed(ref_q30_to_q15(product_q30)));

                if (selected_mode) begin
                    if (ref_denominator <= 0)
                        $fatal(1, "Reference setup error: NLMS denominator must be positive");
                    // Divider computes (signed numerator * 2^30) / denominator,
                    // with integer division truncating toward zero.
                    ref_quotient = (correction_raw * (64'sd1 << 30)) /
                                   ref_denominator;
                    if (ref_quotient > 32767)
                        correction_raw = 32767;
                    else if (ref_quotient < -32768)
                        correction_raw = -32768;
                    else
                        correction_raw = ref_quotient;
                end

                exp_correction = correction_raw[15:0];
                weight_sum = longint'($signed(ref_weights[k])) +
                             longint'($signed(exp_correction));
                exp_weights[k] = ref_sat_q15(weight_sum);
            end

            // Advance independent model state for the next iteration.
            for (k = 0; k < 4; k = k + 1)
                ref_weights[k] = exp_weights[k];
        end
    endtask

    task automatic run_and_check(input logic selected_mode, input string label);
        integer cycles;
        begin
            calculate_reference(selected_mode);

            @(negedge clk);
            mode = selected_mode;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            cycles = 0;
            while ((done !== 1'b1) && (cycles < 800)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end

            if (done !== 1'b1)
                $fatal(1, "TIMEOUT: %s mode=%0d", label, selected_mode);
            if (busy !== 1'b0)
                $fatal(1, "%s: busy high when done asserted", label);
            if (error_status !== 1'b0)
                $fatal(1, "%s: unexpected error_status=1", label);

            if (y_out !== exp_y)
                $fatal(1, "%s: y mismatch actual=%0d expected=%0d",
                       label, y_out, exp_y);
            if (error_out !== exp_error)
                $fatal(1, "%s: error mismatch actual=%0d expected=%0d",
                       label, error_out, exp_error);
            if (w0 !== exp_weights[0] || w1 !== exp_weights[1] ||
                w2 !== exp_weights[2] || w3 !== exp_weights[3])
                $fatal(1, "%s: weights actual={%0d,%0d,%0d,%0d} expected={%0d,%0d,%0d,%0d}",
                       label, w0, w1, w2, w3,
                       exp_weights[0], exp_weights[1], exp_weights[2], exp_weights[3]);

            total_vectors = total_vectors + 1;
            total_checks = total_checks + 3;
            $display("PASS REF %-22s mode=%s cycles=%0d y=%0d e=%0d weights={%0d,%0d,%0d,%0d}",
                     label, selected_mode ? "NLMS" : "LMS", cycles,
                     y_out, error_out, w0, w1, w2, w3);

            // Check completion is a pulse, not a level that remains asserted.
            @(negedge clk);
            if (done !== 1'b0)
                $fatal(1, "%s: done was not a one-cycle pulse", label);
            total_checks = total_checks + 1;
        end
    endtask

    task automatic reset_all;
        integer k;
        begin
            @(negedge clk);
            start = 1'b0;
            rst_n = 1'b0;
            repeat (3) @(negedge clk);
            if (busy !== 1'b0 || done !== 1'b0)
                $fatal(1, "Reset did not clear busy/done");
            for (k = 0; k < 4; k = k + 1)
                ref_weights[k] = 0;
            if ({w0,w1,w2,w3} !== 64'b0)
                $fatal(1, "Reset did not clear DUT weights");
            rst_n = 1'b1;
            @(negedge clk);
        end
    endtask

    integer i;
    integer temp_value;
    initial begin
        x0 = 0; x1 = 0; x2 = 0; x3 = 0;
        desired = 0;
        mu = 0;
        epsilon_q30 = 34'sd10737418; // approximately 0.01 in raw Q30
        for (i = 0; i < 4; i = i + 1)
            ref_weights[i] = 0;

        reset_all();

        // Directed vectors: original LMS baseline, then multiple NLMS/LMS cases.
        x0 = 16'sd8192; x1 = 16'sd16384; x2 = -16'sd8192; x3 = 16'sd4096;
        desired = 16'sd16384; mu = 16'sd8192;
        run_and_check(1'b0, "directed LMS 1");
        run_and_check(1'b0, "directed LMS 2");

        reset_all();
        x0 = 16'sd8192; x1 = 16'sd16384; x2 = -16'sd8192; x3 = 16'sd4096;
        desired = 16'sd16384; mu = 16'sd8192;
        run_and_check(1'b1, "directed NLMS mixed");
        run_and_check(1'b1, "repeated NLMS");

        // Zero-energy NLMS with positive epsilon must leave all weights unchanged.
        reset_all();
        x0 = 0; x1 = 0; x2 = 0; x3 = 0;
        desired = 16'sd12000; mu = 16'sd4096;
        run_and_check(1'b1, "NLMS zero energy");

        // Negative and asymmetric inputs.
        reset_all();
        x0 = -16'sd8192; x1 = -16'sd4096; x2 = -16'sd12000; x3 = -16'sd2048;
        desired = 16'sd10000; mu = 16'sd2048;
        run_and_check(1'b1, "NLMS all negative");
        x0 = 16'sd3000; x1 = -16'sd9000; x2 = 16'sd15000; x3 = -16'sd2500;
        desired = -16'sd7000; mu = 16'sd1536;
        run_and_check(1'b0, "LMS mixed signs");
        run_and_check(1'b1, "NLMS mixed signs 2");

        // Reproducible bounded pseudo-random sequence. Inputs are deliberately
        // bounded away from full scale to avoid making this first model test a
        // saturation-only test; dedicated saturation tests should follow later.
        reset_all();
        epsilon_q30 = 34'sd10737418;
        rng_state = 32'h51A7_C0DE;
        for (i = 0; i < 24; i = i + 1) begin
            rng_state = lcg_next(rng_state);
            temp_value = integer'(rng_state[14:0] % 24001) - 12000;
            x0 = temp_value;

            rng_state = lcg_next(rng_state);
            temp_value = integer'(rng_state[14:0] % 24001) - 12000;
            x1 = temp_value;

            rng_state = lcg_next(rng_state);
            temp_value = integer'(rng_state[14:0] % 24001) - 12000;
            x2 = temp_value;

            rng_state = lcg_next(rng_state);
            temp_value = integer'(rng_state[14:0] % 24001) - 12000;
            x3 = temp_value;

            rng_state = lcg_next(rng_state);
            temp_value = integer'(rng_state[14:0] % 28001) - 14000;
            desired = temp_value;

            rng_state = lcg_next(rng_state);
            mu = 512 + (rng_state[11:0] % 3585);

            run_and_check((i % 2) != 0,
                          (i % 2) != 0 ? "seeded random NLMS" : "seeded random LMS");
        end

        $display("");
        $display("ALL INDEPENDENT REFERENCE CHECKS PASSED: vectors=%0d checks=%0d",
                 total_vectors, total_checks);
        $finish;
    end

    initial begin
        #500000;
        $fatal(1, "GLOBAL WATCHDOG: simulation exceeded 500 us");
    end
endmodule
