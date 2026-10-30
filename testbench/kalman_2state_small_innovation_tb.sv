`timescale 1ns/1ps

// KAL-004H: Small positive innovation-covariance and gain-saturation stress test.
// Run against kalman_2state_controller_gain18_diag.sv.
// Formats: state/z Q1.15, F/H Q2.14, covariance/noise Q10.30,
// internal gain Q3.15.
//
// Case A: S is small but positive and K remains representable.
// Case B: S remains positive but the mathematical gain is just above +4;
//         the controller must saturate the gain and flag overflow stage 14,
//         without hanging or incorrectly marking the transaction as an S error.

module kalman_2state_small_innovation_tb;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n, init_valid, start;
    logic signed [15:0] init_x0, init_x1;
    logic signed [39:0] init_p00, init_p01, init_p11;
    logic busy, done, error_status, overflow_seen;

    logic signed [15:0] cfg_f00, cfg_f01, cfg_f10, cfg_f11;
    logic signed [39:0] cfg_q00, cfg_q01, cfg_q11;
    logic signed [15:0] cfg_h0, cfg_h1;
    logic signed [39:0] cfg_r;
    logic signed [15:0] measurement;

    logic signed [15:0] state0, state1;
    logic signed [39:0] p00, p01, p11;

    integer checks = 0;
    integer failures = 0;
    integer cycles;

    localparam logic [3:0] ST_UPDATE_X_CODE = 4'd7;

    kalman_2state_ip dut (
        .clk(clk),
        .rst_n(rst_n),
        .init_valid(init_valid),
        .init_x0(init_x0),
        .init_x1(init_x1),
        .init_p00(init_p00),
        .init_p01(init_p01),
        .init_p11(init_p11),
        .start(start),
        .busy(busy),
        .done(done),
        .error_status(error_status),
        .overflow_seen(overflow_seen),
        .cfg_f00(cfg_f00),
        .cfg_f01(cfg_f01),
        .cfg_f10(cfg_f10),
        .cfg_f11(cfg_f11),
        .cfg_q00(cfg_q00),
        .cfg_q01(cfg_q01),
        .cfg_q11(cfg_q11),
        .cfg_h0(cfg_h0),
        .cfg_h1(cfg_h1),
        .cfg_r(cfg_r),
        .measurement(measurement),
        .state0(state0),
        .state1(state1),
        .p00(p00),
        .p01(p01),
        .p11(p11)
    );

    task automatic check(input logic condition, input string message);
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                failures = failures + 1;
                $display("FAIL: %s", message);
            end else begin
                $display("PASS: %s", message);
            end
        end
    endtask

    task automatic reset_and_initialize(
        input logic signed [39:0] p00_initial,
        input logic signed [39:0] p11_initial
    );
        begin
            rst_n = 1'b0;
            init_valid = 1'b0;
            start = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);

            init_x0 = 16'sd0;
            init_x1 = 16'sd0;
            init_p00 = p00_initial;
            init_p01 = 40'sd0;
            init_p11 = p11_initial;
            init_valid = 1'b1;
            @(negedge clk);
            init_valid = 1'b0;
            @(negedge clk);
            check(!busy, "controller is idle after initialization");
        end
    endtask

    task automatic launch_and_wait;
        begin
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            cycles = 0;
            while ((done !== 1'b1) && (cycles < 100)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end
            check(done === 1'b1, "transaction completes before timeout");
            check(!busy, "busy is low after transaction");
        end
    endtask

    initial begin
        rst_n = 1'b0;
        init_valid = 1'b0;
        start = 1'b0;
        init_x0 = '0;
        init_x1 = '0;
        init_p00 = '0;
        init_p01 = '0;
        init_p11 = '0;

        // Identity F, zero Q.
        cfg_f00 = 16'sd16384;
        cfg_f01 = 16'sd0;
        cfg_f10 = 16'sd0;
        cfg_f11 = 16'sd16384;
        cfg_q00 = 40'sd0;
        cfg_q01 = 40'sd0;
        cfg_q11 = 40'sd0;

        cfg_h0 = 16'sd16384;  // Case A H0 = 1.0
        cfg_h1 = 16'sd0;
        cfg_r = 40'sd1074;    // approximately 1e-6
        measurement = 16'sd1638; // approximately 0.05

        // -----------------------------------------------------------------
        // Case A: small positive S, gain remains representable.
        // P00 ~= 1e-4, H0 = 1, R ~= 1e-6 => K0 ~= 0.990.
        // -----------------------------------------------------------------
        $display("");
        $display("KAL-004H CASE A: small positive S, representable gain");
        reset_and_initialize(40'sd107374, 40'sd107374182);

        cfg_h0 = 16'sd16384;
        cfg_r = 40'sd1074;
        launch_and_wait();

       

        check(error_status == 1'b0, "small positive S is not treated as an error");
        check(overflow_seen == 1'b0, "representable gain does not set overflow");
        check(dut.s_r > 0 && dut.s_r < 40'sd200000,
              "innovation covariance is positive and small");
        check(dut.k0_r > 16'sd30000 && dut.k0_r < 18'sd32768,
              "gain is close to but below +1.0");
        check(state0 > 16'sd1500 && state0 < 16'sd1700,
              "state update follows the small-S reference expectation");
        check(p00 >= 0, "updated observed-state variance is nonnegative");

        // -----------------------------------------------------------------
        // Case B: tiny positive R and small P; mathematical gain is > 4.
        // P00 ~= 1e-4, H0 = 0.25, R = one Q30 LSB => K0 is just above 4.
        // -----------------------------------------------------------------
        $display("");
        $display("KAL-004H CASE B: positive S but gain exceeds Q3.15 range");
        reset_and_initialize(40'sd107374, 40'sd107374182);

        cfg_h0 = 16'sd4048; // 0.25
        cfg_r = 40'sd1;     // smallest positive Q10.30 value
        measurement = 16'sd0;
        launch_and_wait();

        $display("");
        $display("===== KAL-004H CASE B INTERNAL DIAGNOSTICS =====");
        $display("H0 raw             = %0d", $signed(cfg_h0));
        $display("R raw              = %0d", $signed(cfg_r));
        $display("P00 predicted raw  = %0d", $signed(dut.pp00));
        $display("HP0 raw            = %0d", $signed(dut.hp0));
        $display("Innovation S raw   = %0d", $signed(dut.s_r));
        $display("Gain numerator raw = %0d", $signed(dut.gain_num0));
        $display("Unsaturated gain   = %0d", $signed(dut.t0));
        $display("Stored K0 raw      = %0d", $signed(dut.k0_r));
        $display("Overflow seen      = %b", dut.overflow_seen);
        $display("Overflow stage     = %0d", dut.overflow_stage);
        $display("Error status       = %b", dut.error_status);
        $display("================================================");

        check(dut.s_r > 0, "innovation covariance remains strictly positive");
        check(error_status == 1'b0, "gain overflow is distinct from S-domain error");
        check(overflow_seen == 1'b1, "unrepresentable gain sets overflow_seen");
        check(dut.overflow_stage == 8'd14, "overflow is attributed to gain range check");
        check(dut.k0_r == 18'sd131071,
              "positive gain saturates at maximum signed Q3.15 value");
        check(!busy, "controller returns to idle after saturated-gain transaction");

        $display("");
        $display("KAL-004H SMALL-INNOVATION SUMMARY: checks=%0d failures=%0d",
                 checks, failures);
        if (failures == 0)
            $display("ALL KAL-004H SMALL-INNOVATION CHECKS PASSED");
        else
            $display("KAL-004H SMALL-INNOVATION TEST FAILED");
        $finish;
    end

endmodule
