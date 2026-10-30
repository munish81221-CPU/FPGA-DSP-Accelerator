`timescale 1ns/1ps

// KAL-004J: Fixed-Point Arithmetic and Overflow Audit
// DUT: kalman_2state_ip in rtl/kalman_2state_controller_gain18_diag.sv
//
// Formats used by the DUT:
//   state/measurement : signed Q1.15 (16-bit)
//   F/H coefficients  : signed Q2.14 (16-bit)
//   P/Q/R covariance  : signed Q10.30 (40-bit)
//   Kalman gain       : signed Q3.15 (18-bit)
//
// This audit focuses on directed tests for:
//   1. A nominal transaction that should not overflow.
//   2. State-prediction saturation.
//   3. Innovation saturation.
//   4. Covariance-prediction saturation.
//   5. State-update saturation.
//   6. Covariance saturation / overflow reporting.
//
// Internal stage codes are checked only where the stimulus isolates a stage.
// Since overflow_stage records the LAST detected overflow, multi-stage
// overflow cases should check overflow_seen and the saturated architectural
// output, not assume the stage code identifies the first overflow.

module kalman_2state_fixed_point_audit_tb;

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

    localparam logic signed [15:0] Q14_ONE = 16'sd16384;
    localparam logic signed [15:0] Q14_TWO = 16'sd32767;
    localparam logic signed [39:0] Q30_ONE_TENTH = 40'sd107374182;
    localparam logic signed [39:0] Q30_TWO_HUNDRED = 40'sd214748364800;
    localparam logic signed [39:0] Q30_ONE = 40'sd1073741824;

    kalman_2state_ip dut (
        .clk(clk), .rst_n(rst_n),
        .init_valid(init_valid),
        .init_x0(init_x0), .init_x1(init_x1),
        .init_p00(init_p00), .init_p01(init_p01), .init_p11(init_p11),
        .start(start), .busy(busy), .done(done),
        .error_status(error_status), .overflow_seen(overflow_seen),
        .cfg_f00(cfg_f00), .cfg_f01(cfg_f01),
        .cfg_f10(cfg_f10), .cfg_f11(cfg_f11),
        .cfg_q00(cfg_q00), .cfg_q01(cfg_q01), .cfg_q11(cfg_q11),
        .cfg_h0(cfg_h0), .cfg_h1(cfg_h1), .cfg_r(cfg_r),
        .measurement(measurement),
        .state0(state0), .state1(state1),
        .p00(p00), .p01(p01), .p11(p11)
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

    task automatic set_identity_model;
        begin
            cfg_f00 = Q14_ONE; cfg_f01 = 16'sd0;
            cfg_f10 = 16'sd0;  cfg_f11 = Q14_ONE;
            cfg_q00 = 40'sd0; cfg_q01 = 40'sd0; cfg_q11 = 40'sd0;
            cfg_h0 = Q14_ONE; cfg_h1 = 16'sd0;
            cfg_r = Q30_ONE_TENTH;
            measurement = 16'sd0;
        end
    endtask

    task automatic reset_and_initialize(
        input logic signed [15:0] x0_initial,
        input logic signed [15:0] x1_initial,
        input logic signed [39:0] p00_initial,
        input logic signed [39:0] p01_initial,
        input logic signed [39:0] p11_initial
    );
        begin
            rst_n = 1'b0;
            init_valid = 1'b0;
            start = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);
            init_x0 = x0_initial;
            init_x1 = x1_initial;
            init_p00 = p00_initial;
            init_p01 = p01_initial;
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
            check(!busy, "controller returns to idle");
        end
    endtask

    initial begin
        rst_n = 1'b0;
        init_valid = 1'b0;
        start = 1'b0;
        init_x0 = '0; init_x1 = '0;
        init_p00 = '0; init_p01 = '0; init_p11 = '0;
        set_identity_model();

        // ---------------------------------------------------------------
        // CASE A: nominal arithmetic; no overflow expected.
        // ---------------------------------------------------------------
        $display("\nKAL-004J CASE A: nominal fixed-point transaction");
        set_identity_model();
        cfg_r = Q30_ONE_TENTH;
        reset_and_initialize(16'sd3277, -16'sd1638,
                             Q30_ONE_TENTH, 40'sd0, Q30_ONE_TENTH);
        launch_and_wait();
        check(error_status == 1'b0, "nominal transaction has no domain error");
        check(overflow_seen == 1'b0, "nominal transaction does not overflow");
        check(state0 > 16'sd1500 && state0 < 16'sd1800,
              "nominal state0 matches expected measurement correction");
        check(state1 == -16'sd1638,
              "identity transition preserves unobserved state1");
        check(p00 >= 0 && p11 >= 0,
              "nominal covariance diagonals remain nonnegative");

        // ---------------------------------------------------------------
        // CASE B: state prediction exceeds signed Q1.15 positive range.
        // F00 ~= 2.0 and x0 ~= 0.9 => predicted x0 > +1.0.
        // H=0 means this prediction overflow remains the only expected
        // overflow source; positive R keeps S valid.
        // ---------------------------------------------------------------
        $display("\nKAL-004J CASE B: state-prediction saturation");
        set_identity_model();
        cfg_f00 = Q14_TWO;
        cfg_h0 = 16'sd0;
        cfg_h1 = 16'sd0;
        cfg_r = Q30_ONE_TENTH;
        reset_and_initialize(16'sd29491, 16'sd0,
                             Q30_ONE_TENTH, 40'sd0, Q30_ONE_TENTH);
        launch_and_wait();
        check(error_status == 1'b0, "state saturation does not create S-domain error");
        check(overflow_seen == 1'b1, "state-prediction overflow is reported");
        check(state0 == 16'sd32767,
              "positive predicted state saturates at +32767");
        check(p00 >= 0, "state-prediction saturation completes with valid covariance sign");

        // ---------------------------------------------------------------
        // CASE C: innovation subtraction exceeds signed Q1.15 range.
        // H=1, xpred near -1, z near +1 => innovation > +1.
        // ---------------------------------------------------------------
        $display("\nKAL-004J CASE C: innovation saturation");
        set_identity_model();
        cfg_h0 = Q14_ONE;
        cfg_h1 = 16'sd0;
        cfg_r = Q30_ONE_TENTH;
        measurement = 16'sd32767;
        reset_and_initialize(-16'sd32768, 16'sd0,
                             Q30_ONE_TENTH, 40'sd0, Q30_ONE_TENTH);
        launch_and_wait();
        check(error_status == 1'b0, "innovation saturation does not create domain error");
        check(overflow_seen == 1'b1, "innovation overflow is reported");
        check(state0 > -16'sd32768 && state0 < 16'sd0,
              "corrected state remains within signed Q1.15 range");

        // ---------------------------------------------------------------
        // CASE D: covariance prediction overflow.
        // P00=200.0 and F00 ~= 2 => predicted P00 ~= 800, above Q10.30
        // maximum (just below 512). H=0 avoids gain/covariance update.
        // ---------------------------------------------------------------
        $display("\nKAL-004J CASE D: covariance-prediction saturation");
        set_identity_model();
        cfg_f00 = Q14_TWO;
        cfg_h0 = 16'sd0;
        cfg_h1 = 16'sd0;
        cfg_r = Q30_ONE_TENTH;
        reset_and_initialize(16'sd0, 16'sd0,
                             Q30_TWO_HUNDRED, 40'sd0, Q30_ONE_TENTH);
        launch_and_wait();
        check(error_status == 1'b0, "covariance saturation does not create domain error");
        check(overflow_seen == 1'b1, "covariance-prediction overflow is reported");
        check(p00 == 40'sh7FFFFFFFFF,
              "positive predicted covariance saturates at signed 40-bit maximum");

        // ---------------------------------------------------------------
        // CASE E: state-update saturation.
        // Start with xpred near positive max; measurement asks for a large
        // positive correction. K should be positive and innovation positive.
        // ---------------------------------------------------------------
        $display("\nKAL-004J CASE E: state-update saturation");
        set_identity_model();
        cfg_h0 = 16'sd4096; // 0.25; permits K near +4
        cfg_h1 = 16'sd0;
        cfg_r = 40'sd1;
        measurement = 16'sd32767;
        reset_and_initialize(16'sd30000, 16'sd0,
                             Q30_ONE, 40'sd0, Q30_ONE_TENTH);
        launch_and_wait();
        check(error_status == 1'b0, "state-update saturation does not create domain error");
        check(overflow_seen == 1'b1, "state-update overflow is reported");
        check(state0 == 16'sd32767,
              "positive corrected state saturates at +32767");

        // ---------------------------------------------------------------
        // CASE F: adversarial covariance-update overflow.
        // Deliberately use a non-PSD initial covariance to stress arithmetic
        // saturation in the covariance update (this is NOT a physically valid
        // covariance model). P00=0.1 keeps S positive; P01=500 and P11 near
        // the positive 40-bit limit make K1*HP1 exceed the signed covariance
        // range, so P11_new must saturate at the negative 40-bit limit.
        // ---------------------------------------------------------------
        $display("\\nKAL-004J CASE F: adversarial covariance-update overflow");
        set_identity_model();
        cfg_h0 = Q14_ONE;
        cfg_h1 = 16'sd0;
        cfg_r = 40'sd1;
        measurement = 16'sd0;
        reset_and_initialize(16'sd0, 16'sd0,
                             Q30_ONE_TENTH,
                             40'sd536870912000, // 500.0 in Q10.30
                             40'sd549755813887); // max positive signed 40-bit
        launch_and_wait();
        check(error_status == 1'b0, "positive S avoids the S-domain error path");
        check(overflow_seen == 1'b1, "adversarial covariance arithmetic reports overflow");
        check(p11 == 40'sh8000000000,
              "underflowing covariance update saturates at signed 40-bit minimum");
        check(dut.overflow_stage == 8'd19,
              "final covariance-update overflow is attributed to stage 19");
        check(!busy, "controller remains recoverable after overflow transaction");

        $display("");
        $display("KAL-004J SUMMARY: checks=%0d failures=%0d", checks, failures);
        if (failures == 0)
            $display("ALL KAL-004J FIXED-POINT AUDIT CHECKS PASSED");
        else
            $display("KAL-004J FIXED-POINT AUDIT FAILED");
        $finish;
    end

    initial begin
        #200000;
        $display("FAIL: global timeout");
        $finish;
    end

endmodule
