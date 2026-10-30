`timescale 1ns/1ps

// KAL-004C: Numerical stability regression for the two-state, single-measurement
// Kalman IP. One self-contained testbench; no staged/phase-specific tests.
//
// Coverage:
//   1) Repeated scalar measurements with identity F and H=[1,0].
//      Checks state convergence, non-increasing P00 when Q=0, and invariant P11.
//   2) Repeated updates from correlated initial covariance.
//      Checks nonnegative diagonal covariance, finite/bounded values, and no errors.
//   3) Repeated updates with nonzero process noise.
//      Checks covariance remains valid and settles within a broad bounded envelope.
//   4) Change measurement values across consecutive transactions.
//      Checks the estimate responds in the expected direction without overflow.
//
// Important: these are numerical-stability/invariant checks, not a bit-accurate
// equation-by-equation reference model. The DUT exposes only P00, P01, and P11;
// symmetry is structurally represented by a single P01 output.
//
// Run from project root:
// iverilog -g2012 -s kalman_2state_stability_tb -o kalman_stability rtl/kalman_2state_ip.sv testbench/kalman_2state_stability_tb.sv
// vvp kalman_stability

module kalman_2state_stability_tb;
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
    integer i;
    integer prev_abs_error;
    integer current_abs_error;
    logic signed [39:0] prev_p00;
    logic signed [15:0] prev_state0;
    logic signed [39:0] scenario2_initial_p00;
    logic signed [39:0] scenario2_initial_p11;
    logic signed [39:0] scenario3_first_p00;
    logic signed [39:0] scenario3_last_p00;

    localparam logic signed [39:0] Q30_ONE       = 40'sd1073741824;
    localparam logic signed [39:0] Q30_HALF      = 40'sd536870912;
    localparam logic signed [39:0] Q30_QUARTER   = 40'sd268435456;
    localparam logic signed [39:0] Q30_TENTH     = 40'sd107374182;
    localparam logic signed [39:0] Q30_HUNDREDTH = 40'sd10737418;
    localparam logic signed [39:0] Q30_THOU      = 40'sd1073742;
    localparam logic signed [39:0] Q30_TWO_THOU  = 40'sd2147484;
    localparam logic signed [39:0] Q30_FIVE_HUND = 40'sd536871;

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
                $display("FAIL %s (condition=%b)", label, condition);
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

    task automatic initialize(
        input logic signed [15:0] x0,
        input logic signed [15:0] x1,
        input logic signed [39:0] ip00,
        input logic signed [39:0] ip01,
        input logic signed [39:0] ip11
    );
        begin
            @(negedge clk);
            init_x0 = x0; init_x1 = x1;
            init_p00 = ip00; init_p01 = ip01; init_p11 = ip11;
            init_valid = 1'b1;
            @(negedge clk);
            init_valid = 1'b0;
        end
    endtask

    task automatic transact;
        integer cycles;
        begin
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            cycles = 0;
            while ((done !== 1'b1) && (cycles < 40)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end
            if (cycles >= 40) begin
                failures = failures + 1;
                checks = checks + 1;
                $display("FAIL transaction timeout");
            end
            else begin
                checks = checks + 1;
                if (error_status === 1'b0)
                    $display("PASS transaction completed (%0d wait cycles)", cycles);
                else begin
                    failures = failures + 1;
                    $display("FAIL transaction completed with error_status=1");
                end
            end
            @(negedge clk);
        end
    endtask

    function automatic integer abs_int(input integer value);
        begin
            if (value < 0) abs_int = -value;
            else abs_int = value;
        end
    endfunction

    initial begin
        // Common safe configuration: identity transition, H=[1,0].
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_h0 = 16'sd16384; cfg_h1 = 0;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_r = Q30_HUNDREDTH;
        measurement = 16'sd16384; // +0.5

        // -----------------------------------------------------------------
        // Scenario 1: 20 repeated measurements, Q=0, initially uncorrelated P.
        // P00 should not increase; x0 should approach z; x1 and P11 stay fixed.
        // -----------------------------------------------------------------
        reset_dut();
        initialize(16'sd0, -16'sd4096, Q30_QUARTER, 0, Q30_TENTH);
        prev_p00 = p00;
        prev_state0 = state0;
        prev_abs_error = abs_int(16384 - $signed(state0));

        for (i = 0; i < 20; i = i + 1) begin
            transact();
            check(p00 >= 0, $sformatf("S1 iter %0d: P00 nonnegative", i+1));
            check(p11 == Q30_TENTH, $sformatf("S1 iter %0d: unobserved P11 invariant", i+1));
            check(p01 == 0, $sformatf("S1 iter %0d: zero cross-covariance invariant", i+1));
            check(p00 <= prev_p00, $sformatf("S1 iter %0d: P00 non-increasing for Q=0", i+1));
            current_abs_error = abs_int(16384 - $signed(state0));
            check(current_abs_error <= prev_abs_error,
                  $sformatf("S1 iter %0d: estimate approaches fixed measurement", i+1));
            check(overflow_seen == 1'b0, $sformatf("S1 iter %0d: no overflow", i+1));
            prev_p00 = p00;
            prev_state0 = state0;
            prev_abs_error = current_abs_error;
        end
        check(state0 > 15000 && state0 <= 16384,
              "S1 final state is close to measurement without overshoot");
        check(p00 < Q30_QUARTER, "S1 final P00 lower than initial P00");

        // -----------------------------------------------------------------
        // Scenario 2: correlated initial covariance; repeated observations.
        // Check covariance diagonal remains nonnegative and no errors/overflow.
        // P01 is nonzero and may be positive or negative after updates.
        // -----------------------------------------------------------------
        reset_dut();
        cfg_r = Q30_HUNDREDTH;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        initialize(-16'sd8192, 16'sd6144, Q30_QUARTER, Q30_FIVE_HUND, Q30_TENTH);
        scenario2_initial_p00 = Q30_QUARTER;
        scenario2_initial_p11 = Q30_TENTH;
        for (i = 0; i < 15; i = i + 1) begin
            measurement = (i < 7) ? 16'sd8192 : -16'sd4096;
            transact();
            check(p00 >= 0, $sformatf("S2 iter %0d: P00 nonnegative", i+1));
            check(p11 >= 0, $sformatf("S2 iter %0d: P11 nonnegative", i+1));
            check(overflow_seen == 1'b0, $sformatf("S2 iter %0d: no overflow", i+1));
            check(error_status == 1'b0, $sformatf("S2 iter %0d: no error", i+1));
        end
        check(p00 <= scenario2_initial_p00, "S2 final P00 bounded by initial value");
        check(p11 <= scenario2_initial_p11, "S2 final P11 bounded by initial value");

        // -----------------------------------------------------------------
        // Scenario 3: nonzero process noise. P00 can rise due to Q, so do not
        // assert monotonic decrease; instead require valid bounded covariance.
        // -----------------------------------------------------------------
        reset_dut();
        cfg_r = Q30_TENTH;
        cfg_q00 = Q30_THOU;
        cfg_q01 = 0;
        cfg_q11 = Q30_TWO_THOU;
        measurement = 16'sd4096; // +0.125
        initialize(16'sd0, 16'sd2048, Q30_HUNDREDTH, 0, Q30_TENTH);
        for (i = 0; i < 25; i = i + 1) begin
            transact();
            check(p00 >= 0 && p00 < Q30_HALF,
                  $sformatf("S3 iter %0d: P00 remains in bounded range", i+1));
            check(p11 >= 0 && p11 < Q30_HALF,
                  $sformatf("S3 iter %0d: P11 remains in bounded range", i+1));
            check(overflow_seen == 1'b0, $sformatf("S3 iter %0d: no overflow", i+1));
            check(error_status == 1'b0, $sformatf("S3 iter %0d: no error", i+1));
        end
        scenario3_last_p00 = p00;
        check(scenario3_last_p00 > 0, "S3 final P00 remains positive with process noise");
        check(p11 > Q30_TENTH, "S3 P11 reflects accumulated process noise");

        // -----------------------------------------------------------------
        // Scenario 4: measurement direction changes over a sequence.
        // The estimate should first move upward, then downward after the
        // measurement changes. Use fresh covariance so gain is not near zero.
        // -----------------------------------------------------------------
        reset_dut();
        cfg_q00 = Q30_HUNDREDTH;
        cfg_q01 = 0; cfg_q11 = Q30_HUNDREDTH;
        cfg_r = Q30_HUNDREDTH;
        initialize(16'sd0, 16'sd0, Q30_QUARTER, 0, Q30_QUARTER);

        measurement = 16'sd16384; // +0.5
        transact();
        scenario3_first_p00 = p00;
        check(state0 > 0, "S4 first positive measurement moves state upward");
        prev_state0 = state0;

        measurement = -16'sd16384; // -0.5
        transact();
        check(state0 < prev_state0, "S4 negative measurement moves state downward");
        check(p00 >= 0 && p11 >= 0, "S4 covariance diagonals remain nonnegative");
        check(overflow_seen == 1'b0, "S4 no overflow after measurement reversal");
        check(error_status == 1'b0, "S4 no error after measurement reversal");

        // Final status and useful state snapshot.
        $display("\\nFinal snapshot: x=(%0d,%0d) P=(%0d,%0d,%0d)",
                 $signed(state0), $signed(state1), $signed(p00), $signed(p01), $signed(p11));
        $display("KAL-004C SUMMARY: checks=%0d passed=%0d failed=%0d",
                 checks, checks-failures, failures);

        if (failures != 0) $fatal(1, "KAL-004C numerical stability verification failed");
        $display("ALL KAL-004C NUMERICAL STABILITY CHECKS PASSED");
        $finish;
    end

    initial begin
        #200000;
        $fatal(1, "TIMEOUT: KAL-004C testbench did not finish");
    end
endmodule
