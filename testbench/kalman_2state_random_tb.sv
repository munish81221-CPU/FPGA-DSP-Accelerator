`timescale 1ns/1ps

// KAL-004F: Seeded, constrained-random stress regression for kalman_2state_ip.
//
// Purpose:
//   Exercise many bounded combinations of transition/observation coefficients,
//   process noise, measurement noise, initial covariance, and measurements.
//   Check transaction completion, status flags, bounded outputs, nonnegative
//   covariance diagonals, and covariance PSD determinant.
//
// This is constrained-random invariant testing, not a bit-exact reference
// comparison and not exhaustive proof. Stimulus is kept inside a moderate
// operating envelope to avoid intentionally provoking the saturation cases
// already covered by KAL-004B.
//
// Fixed-point formats:
//   state/measurement Q1.15; F/H Q2.14; P/Q/R Q10.30.
//
// Run from the project root:
// iverilog -g2012 -s kalman_2state_random_tb -o kalman_random rtl/kalman_2state_ip.sv testbench/kalman_2state_random_tb.sv
// vvp kalman_random

module kalman_2state_random_tb;
    logic clk = 1'b0;
    logic rst_n = 1'b0;

    logic init_valid = 1'b0;
    logic signed [15:0] init_x0 = '0, init_x1 = '0;
    logic signed [39:0] init_p00 = '0, init_p01 = '0, init_p11 = '0;

    logic start = 1'b0;
    logic busy, done, error_status, overflow_seen;

    logic signed [15:0] cfg_f00 = 16'sd16384;
    logic signed [15:0] cfg_f01 = '0, cfg_f10 = '0, cfg_f11 = 16'sd16384;
    logic signed [39:0] cfg_q00 = '0, cfg_q01 = '0, cfg_q11 = '0;
    logic signed [15:0] cfg_h0 = 16'sd16384, cfg_h1 = '0;
    logic signed [39:0] cfg_r = 40'sd10737418;
    logic signed [15:0] measurement = '0;

    logic signed [15:0] state0, state1;
    logic signed [39:0] p00, p01, p11;

    integer checks = 0;
    integer failures = 0;
    integer scenario_idx, step_idx;
    integer transactions = 0;
    integer timeout;
    integer rand_a, rand_b, rand_c, rand_d;
    integer signed_f01, signed_f10, signed_h1;
    integer seed = 32'h51A7F00D;
    real rp00, rp01, rp11, determinant;
    real psd_tolerance;

    localparam logic signed [39:0] Q30_ONE       = 40'sd1073741824;
    localparam logic signed [39:0] Q30_TENTH     = 40'sd107374182;
    localparam logic signed [39:0] Q30_HUNDREDTH = 40'sd10737418;
    localparam logic signed [39:0] Q30_005       = 40'sd5368709;
    localparam logic signed [39:0] Q30_001       = 40'sd1073742;
    localparam logic signed [39:0] Q30_0005      = 40'sd536871;

    always #5 clk = ~clk;

    kalman_2state_ip dut (
        .clk(clk), .rst_n(rst_n),
        .init_valid(init_valid),
        .init_x0(init_x0), .init_x1(init_x1),
        .init_p00(init_p00), .init_p01(init_p01), .init_p11(init_p11),
        .start(start), .busy(busy), .done(done),
        .error_status(error_status), .overflow_seen(overflow_seen),
        .cfg_f00(cfg_f00), .cfg_f01(cfg_f01), .cfg_f10(cfg_f10), .cfg_f11(cfg_f11),
        .cfg_q00(cfg_q00), .cfg_q01(cfg_q01), .cfg_q11(cfg_q11),
        .cfg_h0(cfg_h0), .cfg_h1(cfg_h1), .cfg_r(cfg_r),
        .measurement(measurement),
        .state0(state0), .state1(state1),
        .p00(p00), .p01(p01), .p11(p11)
    );

    task automatic check(input logic condition, input string label);
        begin
            checks = checks + 1;
            if (condition === 1'b1) $display("PASS %s", label);
            else begin
                failures = failures + 1;
                $display("FAIL %s", label);
            end
        end
    endtask

    task automatic reset_dut;
        begin
            @(negedge clk);
            rst_n = 1'b0;
            start = 1'b0;
            init_valid = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);
        end
    endtask

    task automatic initialize_random_case;
        begin
            // F diagonals are 0.75..1.00. Off-diagonals are approximately
            // -0.0625..+0.0625. This keeps the transition matrix moderate.
            rand_a = $urandom(seed) & 32'h7fffffff; seed = rand_a;
            rand_b = $urandom(seed) & 32'h7fffffff; seed = rand_b;
            rand_c = $urandom(seed) & 32'h7fffffff; seed = rand_c;
            rand_d = $urandom(seed) & 32'h7fffffff; seed = rand_d;

            cfg_f00 = 16'sd12288 + (rand_a % 4097);
            cfg_f11 = 16'sd12288 + (rand_b % 4097);
            signed_f01 = (rand_c % 2049) - 1024;
            signed_f10 = (rand_d % 2049) - 1024;
            cfg_f01 = signed_f01;
            cfg_f10 = signed_f10;

            // Positive diagonal process noise, no cross-correlation.
            rand_a = $urandom(seed) & 32'h7fffffff; seed = rand_a;
            rand_b = $urandom(seed) & 32'h7fffffff; seed = rand_b;
            cfg_q00 = Q30_0005 + (rand_a % 5368709);
            cfg_q01 = '0;
            cfg_q11 = Q30_0005 + (rand_b % 5368709);

            // H0 stays positive and moderate; H1 spans approximately
            // -0.25..+0.25. Positive R prevents a zero/negative S case.
            rand_c = $urandom(seed) & 32'h7fffffff; seed = rand_c;
            rand_d = $urandom(seed) & 32'h7fffffff; seed = rand_d;
            cfg_h0 = 16'sd12288 + (rand_c % 8193);
            signed_h1 = (rand_d % 8193) - 4096;
            cfg_h1 = signed_h1;
            cfg_r = Q30_HUNDREDTH + ((rand_a % 4) * Q30_001);

            // Initial P is diagonal and strictly positive, approximately
            // 0.02..0.12 on each diagonal. This is valid PSD by construction.
            rand_a = $urandom(seed) & 32'h7fffffff; seed = rand_a;
            rand_b = $urandom(seed) & 32'h7fffffff; seed = rand_b;
            init_p00 = Q30_TENTH / 5 + (rand_a % (Q30_TENTH + 1));
            init_p01 = '0;
            init_p11 = Q30_TENTH / 5 + (rand_b % (Q30_TENTH + 1));

            rand_c = $urandom(seed) & 32'h7fffffff; seed = rand_c;
            rand_d = $urandom(seed) & 32'h7fffffff; seed = rand_d;
            init_x0 = (rand_c % 12001) - 6000;
            init_x1 = (rand_d % 12001) - 6000;
        end
    endtask

    task automatic apply_initialization;
        begin
            @(negedge clk);
            init_valid = 1'b1;
            @(negedge clk);
            init_valid = 1'b0;
            @(negedge clk);
        end
    endtask

    task automatic run_transaction(input logic signed [15:0] z,
                                   input integer sc, input integer st);
        begin
            @(negedge clk);
            measurement = z;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            timeout = 0;
            while ((done !== 1'b1) && (timeout < 30)) begin
                @(negedge clk);
                timeout = timeout + 1;
            end

            check(done === 1'b1, $sformatf("case %0d step %0d completes", sc, st));
            check(busy === 1'b0, $sformatf("case %0d step %0d busy clears", sc, st));
            check(error_status === 1'b0, $sformatf("case %0d step %0d no error", sc, st));
            check(overflow_seen === 1'b0, $sformatf("case %0d step %0d no overflow", sc, st));

            if (done === 1'b1) begin
                transactions = transactions + 1;
                check_covariance(sc, st);
                check((state0 <= 16'sh7fff) && (state0 >= -16'sh8000),
                      $sformatf("case %0d step %0d state0 is representable", sc, st));
                check((state1 <= 16'sh7fff) && (state1 >= -16'sh8000),
                      $sformatf("case %0d step %0d state1 is representable", sc, st));
            end
        end
    endtask

    task automatic check_covariance(input integer sc, input integer st);
        begin
            rp00 = $itor($signed(p00)) / 1073741824.0;
            rp01 = $itor($signed(p01)) / 1073741824.0;
            rp11 = $itor($signed(p11)) / 1073741824.0;
            determinant = (rp00 * rp11) - (rp01 * rp01);
            psd_tolerance = 0.000001;

            check(p00 >= 0, $sformatf("case %0d step %0d P00 nonnegative", sc, st));
            check(p11 >= 0, $sformatf("case %0d step %0d P11 nonnegative", sc, st));
            check(determinant >= -psd_tolerance,
                  $sformatf("case %0d step %0d covariance PSD", sc, st));
            $display("      random[%0d,%0d] P=(%0.7f,%0.7f,%0.7f), det=%0.9f",
                     sc, st, rp00, rp01, rp11, determinant);
        end
    endtask

    initial begin
        $display("KAL-004F: constrained-random Kalman robustness regression");
        $display("Seed initialization = 0x%08h", seed);
        reset_dut();

        // 30 deterministic pseudo-random configurations, three measurements
        // per configuration. Every configuration is reset and initialized
        // independently so one scenario cannot contaminate the next.
        for (scenario_idx = 0; scenario_idx < 30; scenario_idx = scenario_idx + 1) begin
            reset_dut();
            initialize_random_case();
            apply_initialization();

            check(p00 >= 0, $sformatf("case %0d initialized P00 nonnegative", scenario_idx));
            check(p11 >= 0, $sformatf("case %0d initialized P11 nonnegative", scenario_idx));
            check_covariance(scenario_idx, -1);

            for (step_idx = 0; step_idx < 3; step_idx = step_idx + 1) begin
                rand_a = $urandom(seed) & 32'h7fffffff; seed = rand_a;
                // Measurement remains bounded to roughly +/-0.18 in Q1.15.
                measurement = (rand_a % 12001) - 6000;
                run_transaction(measurement, scenario_idx, step_idx);
            end
        end

        $display("");
        $display("KAL-004F SUMMARY: scenarios=30 transactions=%0d checks=%0d failures=%0d",
                 transactions, checks, failures);
        if ((failures == 0) && (transactions == 90))
            $display("ALL KAL-004F CONSTRAINED-RANDOM CHECKS PASSED");
        else
            $display("KAL-004F FAILED: inspect failing scenario and covariance trace");
        $finish;
    end
endmodule
