`timescale 1ns/1ps

// KAL-004E: Covariance integrity and numerical robustness regression.
//
// DUT convention:
//   state/measurement: signed Q1.15
//   F/H coefficients: signed Q2.14
//   covariance Q/R/P: signed Q10.30
//
// Checks:
//   * P00 and P11 remain nonnegative for valid, bounded test cases.
//   * P remains positive semidefinite (PSD): P00*P11 - P01^2 >= -tolerance.
//   * No error or saturation/overflow in these bounded scenarios.
//   * Repeated updates with diagonal and correlated covariance, and with
//     nonzero process noise.
//   * A final scenario uses a non-identity transition matrix.
//
// The determinant is calculated after converting Q30 covariance values to
// real values. This avoids overflowing a 64-bit integer while calculating
// products of 40-bit covariance values. This is an invariant test, not a
// bit-exact reference model.
//
// Run from the project root:
// iverilog -g2012 -s kalman_2state_covariance_tb -o kalman_covariance rtl/kalman_2state_ip.sv testbench/kalman_2state_covariance_tb.sv
// vvp kalman_covariance

module kalman_2state_covariance_tb;
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
    integer i, scenario;
    integer transaction_count = 0;
    real rp00, rp01, rp11, determinant;
    real psd_tolerance;

    localparam logic signed [39:0] Q30_ONE       = 40'sd1073741824;
    localparam logic signed [39:0] Q30_HALF      = 40'sd536870912;
    localparam logic signed [39:0] Q30_QUARTER   = 40'sd268435456;
    localparam logic signed [39:0] Q30_TENTH     = 40'sd107374182;
    localparam logic signed [39:0] Q30_HUNDREDTH = 40'sd10737418;
    localparam logic signed [39:0] Q30_TWO_HUND  = 40'sd214748365;
    localparam logic signed [39:0] Q30_FIVE_THOU = 40'sd5368709;
    localparam logic signed [39:0] Q30_ONE_THOU  = 40'sd1073742;

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

    task automatic initialize_state(
        input logic signed [15:0] x0,
        input logic signed [15:0] x1,
        input logic signed [39:0] ip00,
        input logic signed [39:0] ip01,
        input logic signed [39:0] ip11
    );
        begin
            @(negedge clk);
            init_x0 = x0;
            init_x1 = x1;
            init_p00 = ip00;
            init_p01 = ip01;
            init_p11 = ip11;
            init_valid = 1'b1;
            @(negedge clk);
            init_valid = 1'b0;
            @(negedge clk);
        end
    endtask

    task automatic run_transaction(input logic signed [15:0] z);
        integer timeout;
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
            check(done === 1'b1, "transaction completes before timeout");
            check(busy === 1'b0, "busy deasserts at transaction completion");
            check(error_status === 1'b0, "valid covariance scenario has no error");
            check(overflow_seen === 1'b0, "bounded covariance scenario has no overflow");
            if (done === 1'b1) transaction_count = transaction_count + 1;
        end
    endtask

    task automatic check_covariance(input string label);
        begin
            rp00 = $itor($signed(p00)) / 1073741824.0;
            rp01 = $itor($signed(p01)) / 1073741824.0;
            rp11 = $itor($signed(p11)) / 1073741824.0;
            determinant = (rp00 * rp11) - (rp01 * rp01);
            // A small allowance accounts for finite fixed-point rounding.
            // This is intentionally much smaller than the test covariances.
            psd_tolerance = 0.0000005;

            check(p00 >= 0, {label, ": P00 nonnegative"});
            check(p11 >= 0, {label, ": P11 nonnegative"});
            check(determinant >= -psd_tolerance,
                  {label, ": covariance PSD determinant within rounding tolerance"});
            $display("      %s: P=(%0.8f,%0.8f,%0.8f), det=%0.10f",
                     label, rp00, rp01, rp11, determinant);
        end
    endtask

    initial begin
        $display("KAL-004E: covariance integrity and numerical robustness");
        reset_dut();

        // Scenario E1: diagonal covariance, identity transition, scalar
        // observation of state 0. Repeated updates should preserve PSD.
        scenario = 1;
        cfg_f00 = 16'sd16384; cfg_f01 = '0;
        cfg_f10 = '0; cfg_f11 = 16'sd16384;
        cfg_q00 = '0; cfg_q01 = '0; cfg_q11 = '0;
        cfg_h0 = 16'sd16384; cfg_h1 = '0;
        cfg_r = Q30_HUNDREDTH;
        initialize_state(16'sd8192, -16'sd4096,
                         Q30_QUARTER, '0, Q30_TENTH);
        check_covariance("E1 initial diagonal covariance");
        for (i = 0; i < 12; i = i + 1) begin
            run_transaction(16'sd12288);
            check_covariance("E1 repeated update");
        end

        // Scenario E2: valid correlated covariance and correlated process
        // noise. Initial covariance determinant is positive:
        // 0.25*0.10 - 0.05*0.05 > 0.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = '0;
        cfg_f10 = '0; cfg_f11 = 16'sd16384;
        cfg_q00 = Q30_ONE_THOU;
        cfg_q01 = Q30_ONE_THOU / 2;
        cfg_q11 = Q30_ONE_THOU;
        cfg_h0 = 16'sd16384; cfg_h1 = 16'sd8192;
        cfg_r = Q30_HUNDREDTH;
        initialize_state(-16'sd4096, 16'sd6144,
                         Q30_QUARTER, Q30_FIVE_THOU, Q30_TENTH);
        check_covariance("E2 initial correlated covariance");
        for (i = 0; i < 10; i = i + 1) begin
            run_transaction((i % 2 == 0) ? 16'sd4096 : -16'sd2048);
            check_covariance("E2 correlated update");
        end

        // Scenario E3: non-identity F with moderate coupling and nonzero
        // diagonal process noise. The state/covariance remain in a bounded
        // range so saturation is not expected.
        reset_dut();
        cfg_f00 = 16'sd16384;
        cfg_f01 = 16'sd2048;  // +0.125
        cfg_f10 = -16'sd1024; // -0.0625
        cfg_f11 = 16'sd15360; // +0.9375
        cfg_q00 = Q30_ONE_THOU;
        cfg_q01 = '0;
        cfg_q11 = Q30_ONE_THOU;
        cfg_h0 = 16'sd16384; cfg_h1 = -16'sd4096; // [1,-0.25]
        cfg_r = Q30_TWO_HUND;
        initialize_state(16'sd2048, -16'sd1024,
                         Q30_TENTH, Q30_ONE_THOU, Q30_TENTH);
        check_covariance("E3 initial covariance");
        for (i = 0; i < 12; i = i + 1) begin
            run_transaction((i < 6) ? 16'sd2048 : -16'sd1024);
            check_covariance("E3 coupled-model update");
        end

        // Scenario E4: longer repeated-update stress with moderate process
        // noise. This is still an invariant test, not a proof for all inputs.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = '0;
        cfg_f10 = '0; cfg_f11 = 16'sd16384;
        cfg_q00 = Q30_ONE_THOU;
        cfg_q01 = '0;
        cfg_q11 = Q30_ONE_THOU;
        cfg_h0 = 16'sd16384; cfg_h1 = '0;
        cfg_r = Q30_TWO_HUND;
        initialize_state('0, '0, Q30_HUNDREDTH, '0, Q30_HUNDREDTH);
        check_covariance("E4 initial covariance");
        for (i = 0; i < 20; i = i + 1) begin
            run_transaction((i % 4 < 2) ? 16'sd1024 : -16'sd1024);
            check_covariance("E4 stress update");
        end

        $display("");
        $display("KAL-004E SUMMARY: transactions=%0d checks=%0d failures=%0d",
                 transaction_count, checks, failures);
        if (failures == 0)
            $display("ALL KAL-004E COVARIANCE INTEGRITY CHECKS PASSED");
        else
            $display("KAL-004E FAILED: inspect covariance trace and determinant");
        $finish;
    end
endmodule
