`timescale 1ns/1ps

// KAL-004K: Covariance-update robustness and Joseph-form evaluation.
//
// This is an evaluation/reference testbench, not a Joseph-form RTL implementation.
// It runs the existing simplified covariance update:
//     P_new = P_pred - K*(H*P_pred)
// and independently evaluates the Joseph expression in real arithmetic:
//     P_J = (I-KH)*P_pred*(I-KH)^T + K*R*K^T
//
// The reference uses the DUT's quantized gain and predicted covariance, so the
// comparison isolates the covariance-update form and fixed-point rounding,
// rather than mixing in differences from an independent Kalman gain calculation.
//
// DUT module: kalman_2state_ip
// Compile against: rtl/kalman_2state_controller_gain18_diag.sv
//
// Tolerances are in real covariance units, after converting Q10.30 raw values.
// This checks numerical behavior over representative PSD initial covariance
// cases. It is not a proof for all inputs and does not replace formal verification.

module kalman_2state_joseph_eval_tb;

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
    integer case_id;

    real pred00, pred01, pred11;
    real gain0, gain1, h0_real, h1_real, r_real;
    real a00, a01, a10, a11;
    real joseph00, joseph01, joseph11;
    real actual00, actual01, actual11;
    real det_actual, det_joseph;
    real tol;

    localparam real Q30_SCALE = 1073741824.0;
    localparam real Q15_SCALE = 32768.0;
    localparam real Q14_SCALE = 16384.0;
    localparam real COV_TOL = 0.00002;

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

    task automatic reset_and_initialize(
        input logic signed [15:0] x0_i,
        input logic signed [15:0] x1_i,
        input logic signed [39:0] p00_i,
        input logic signed [39:0] p01_i,
        input logic signed [39:0] p11_i
    );
        begin
            rst_n = 1'b0;
            init_valid = 1'b0;
            start = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);
            init_x0 = x0_i;
            init_x1 = x1_i;
            init_p00 = p00_i;
            init_p01 = p01_i;
            init_p11 = p11_i;
            init_valid = 1'b1;
            @(negedge clk);
            init_valid = 1'b0;
            @(negedge clk);
            check(!busy, "controller idle after initialization");
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
            check(done === 1'b1, "measurement transaction completes");
            check(!busy, "controller returns to idle");
        end
    endtask

    // Evaluate the Joseph form using the actual predicted covariance and the
    // gain/H/R values latched by the DUT. Conversion happens only in the TB.
    task automatic evaluate_joseph_reference;
        real kh00, kh01, kh10, kh11;
        real tmp00, tmp01, tmp10, tmp11;
        begin
            pred00 = q30_to_real(dut.pp00);
            pred01 = q30_to_real(dut.pp01);
            pred11 = q30_to_real(dut.pp11);
            gain0  = $itor($signed(dut.k0_r)) / Q15_SCALE;
            gain1  = $itor($signed(dut.k1_r)) / Q15_SCALE;
            h0_real = $itor($signed(dut.h0_r)) / Q14_SCALE;
            h1_real = $itor($signed(dut.h1_r)) / Q14_SCALE;
            r_real = q30_to_real(dut.r_r);

            // A = I - K*H.
            a00 = 1.0 - gain0*h0_real;
            a01 =       - gain0*h1_real;
            a10 =       - gain1*h0_real;
            a11 = 1.0 - gain1*h1_real;

            // tmp = A * Ppred.
            tmp00 = a00*pred00 + a01*pred01;
            tmp01 = a00*pred01 + a01*pred11;
            tmp10 = a10*pred00 + a11*pred01;
            tmp11 = a10*pred01 + a11*pred11;

            // Joseph: Pj = tmp*A^T + K*R*K^T.
            joseph00 = tmp00*a00 + tmp01*a01 + gain0*r_real*gain0;
            joseph01 = tmp00*a10 + tmp01*a11 + gain0*r_real*gain1;
            joseph11 = tmp10*a10 + tmp11*a11 + gain1*r_real*gain1;

            actual00 = q30_to_real(p00);
            actual01 = q30_to_real(p01);
            actual11 = q30_to_real(p11);

            det_actual = actual00*actual11 - actual01*actual01;
            det_joseph = joseph00*joseph11 - joseph01*joseph01;
        end
    endtask

    task automatic report_and_check_case(input string case_name);
        begin
            evaluate_joseph_reference();
            $display("");
            $display("---- %s ----", case_name);
            $display("Ppred = [[%0.9f, %0.9f], [%0.9f, %0.9f]]",
                     pred00, pred01, pred01, pred11);
            $display("K = [%0.9f, %0.9f], H = [%0.9f, %0.9f], R = %0.9f",
                     gain0, gain1, h0_real, h1_real, r_real);
            $display("Simplified RTL P = [[%0.9f, %0.9f], [%0.9f, %0.9f]]",
                     actual00, actual01, actual01, actual11);
            $display("Joseph reference P = [[%0.9f, %0.9f], [%0.9f, %0.9f]]",
                     joseph00, joseph01, joseph01, joseph11);
            $display("Errors |dP00|=%0.9f |dP01|=%0.9f |dP11|=%0.9f",
                     (actual00 >= joseph00) ? actual00-joseph00 : joseph00-actual00,
                     (actual01 >= joseph01) ? actual01-joseph01 : joseph01-actual01,
                     (actual11 >= joseph11) ? actual11-joseph11 : joseph11-actual11);
            $display("det(P simplified)=%0.12f det(P Joseph)=%0.12f",
                     det_actual, det_joseph);

            check(error_status == 1'b0, "valid positive innovation covariance avoids domain error");
            check(overflow_seen == 1'b0, "nominal Joseph comparison case has no arithmetic overflow");
            check(actual00 >= -COV_TOL && actual11 >= -COV_TOL,
                  "simplified update has nonnegative covariance diagonals within tolerance");
            check(joseph00 >= -COV_TOL && joseph11 >= -COV_TOL,
                  "Joseph reference has nonnegative covariance diagonals within tolerance");
            check(det_joseph >= -COV_TOL,
                  "Joseph reference covariance is positive semidefinite within tolerance");

            tol = COV_TOL;
            check(((actual00-joseph00 <= tol) && (joseph00-actual00 <= tol)),
                  "simplified P00 agrees with Joseph reference within tolerance");
            check(((actual01-joseph01 <= tol) && (joseph01-actual01 <= tol)),
                  "simplified P01 agrees with Joseph reference within tolerance");
            check(((actual11-joseph11 <= tol) && (joseph11-actual11 <= tol)),
                  "simplified P11 agrees with Joseph reference within tolerance");
        end
    endtask

    initial begin
        rst_n = 1'b0;
        init_valid = 1'b0;
        start = 1'b0;
        init_x0 = '0; init_x1 = '0;
        init_p00 = '0; init_p01 = '0; init_p11 = '0;
        cfg_f00 = 16'sd16384; cfg_f01 = 16'sd0;
        cfg_f10 = 16'sd0; cfg_f11 = 16'sd16384;
        cfg_q00 = 40'sd0; cfg_q01 = 40'sd0; cfg_q11 = 40'sd0;
        cfg_h0 = 16'sd16384; cfg_h1 = 16'sd0;
        cfg_r = 40'sd53687091; // approximately 0.05
        measurement = 16'sd0;

        // CASE A: diagonal prior, measurement observes state 0.
        $display("\nKAL-004K CASE A: diagonal covariance, H=[1,0]");
        cfg_f00 = 16'sd16384; cfg_f01 = 16'sd0;
        cfg_f10 = 16'sd0; cfg_f11 = 16'sd16384;
        cfg_q00 = 40'sd0; cfg_q01 = 40'sd0; cfg_q11 = 40'sd0;
        cfg_h0 = 16'sd16384; cfg_h1 = 16'sd0;
        cfg_r = 40'sd53687091;
        reset_and_initialize(16'sd0, 16'sd0,
                             40'sd107374182, 40'sd0, 40'sd214748365);
        launch_and_wait();
        report_and_check_case("CASE A: diagonal prior");

        // CASE B: correlated positive-semidefinite prior and mixed H.
        $display("\nKAL-004K CASE B: correlated covariance, mixed measurement");
        cfg_f00 = 16'sd16384; cfg_f01 = 16'sd0;
        cfg_f10 = 16'sd0; cfg_f11 = 16'sd16384;
        cfg_q00 = 40'sd0; cfg_q01 = 40'sd0; cfg_q11 = 40'sd0;
        cfg_h0 = 16'sd8192;  // 0.5
        cfg_h1 = 16'sd4096;  // 0.25
        cfg_r = 40'sd21474836; // approximately 0.02
        reset_and_initialize(16'sd0, 16'sd0,
                             40'sd107374182, 40'sd32212255, 40'sd214748365);
        launch_and_wait();
        report_and_check_case("CASE B: correlated prior");

        // CASE C: non-identity transition and nonzero process noise.
        $display("\nKAL-004K CASE C: dynamic prediction with process noise");
        cfg_f00 = 16'sd16384; cfg_f01 = 16'sd1638; // about 0.1
        cfg_f10 = 16'sd0; cfg_f11 = 16'sd16384;
        cfg_q00 = 40'sd1073742; // about 0.001
        cfg_q01 = 40'sd0;
        cfg_q11 = 40'sd2147484; // about 0.002
        cfg_h0 = 16'sd12288;   // 0.75
        cfg_h1 = -16'sd4096;   // -0.25
        cfg_r = 40'sd10737418; // about 0.01
        reset_and_initialize(16'sd0, 16'sd0,
                             40'sd53687091, 40'sd10737418, 40'sd107374182);
        launch_and_wait();
        report_and_check_case("CASE C: dynamic model");

        // CASE D: observe state 1 with negative cross covariance.
        $display("\nKAL-004K CASE D: second-state observation and negative correlation");
        cfg_f00 = 16'sd16384; cfg_f01 = 16'sd0;
        cfg_f10 = 16'sd0; cfg_f11 = 16'sd16384;
        cfg_q00 = 40'sd0; cfg_q01 = 40'sd0; cfg_q11 = 40'sd0;
        cfg_h0 = 16'sd0; cfg_h1 = 16'sd16384;
        cfg_r = 40'sd16106127; // about 0.015
        reset_and_initialize(16'sd0, 16'sd0,
                             40'sd214748365, -40'sd32212255, 40'sd85899346);
        launch_and_wait();
        report_and_check_case("CASE D: negative cross covariance");

        $display("");
        $display("KAL-004K SUMMARY: checks=%0d failures=%0d", checks, failures);
        if (failures == 0)
            $display("ALL KAL-004K JOSEPH-FORM EVALUATION CHECKS PASSED");
        else
            $display("KAL-004K JOSEPH-FORM EVALUATION FAILED");
        $finish;
    end

    initial begin
        #300000;
        $display("FAIL: global timeout");
        $finish;
    end

endmodule
