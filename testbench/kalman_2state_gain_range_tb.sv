`timescale 1ns/1ps

// KAL-004G: Directed regression for the widened Q3.15 Kalman gain.
// Purpose: prove that a valid gain greater than +1 is represented without
// false overflow/saturation, while the state and covariance update complete.
//
// DUT contract:
//   state/measurement = Q1.15
//   F/H               = Q2.14
//   P/Q/R             = Q10.30
//   Kalman gain       = internal signed Q3.15 (18-bit)
//
// Directed case:
//   F = I, Q = 0
//   P = diag(0.1, 0.1)
//   H = [0.25, 0]
//   R = 0.001
// This gives K0 approximately 0.025 / (0.00625 + 0.001) = 3.448,
// which is > 1 but fits in signed Q3.15. K1 should be zero.

module kalman_2state_gain_range_tb;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n;
    logic init_valid;
    logic signed [15:0] init_x0, init_x1;
    logic signed [39:0] init_p00, init_p01, init_p11;
    logic start;
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

    task automatic wait_done;
        begin
            cycles = 0;
            while ((done !== 1'b1) && (cycles < 100)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end
            check(done === 1'b1, "transaction completes before timeout");
        end
    endtask

    initial begin
        rst_n = 1'b0;
        init_valid = 1'b0;
        init_x0 = 0;
        init_x1 = 0;
        init_p00 = 0;
        init_p01 = 0;
        init_p11 = 0;
        start = 1'b0;

        // F = identity, encoded as Q2.14.
        cfg_f00 = 16'sd16384;
        cfg_f01 = 16'sd0;
        cfg_f10 = 16'sd0;
        cfg_f11 = 16'sd16384;

        // Q = 0.
        cfg_q00 = 40'sd0;
        cfg_q01 = 40'sd0;
        cfg_q11 = 40'sd0;

        // H = [0.25, 0], encoded in Q2.14.
        cfg_h0 = 16'sd4096;
        cfg_h1 = 16'sd0;

        // R = approximately 0.001 in Q10.30.
        cfg_r = 40'sd1073742;

        // Measurement = approximately 0.1 in Q1.15.
        measurement = 16'sd3277;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        // Initial state is zero; P = diag(0.1, 0.1), Q30 encoding.
        init_x0 = 16'sd0;
        init_x1 = 16'sd0;
        init_p00 = 40'sd107374182;
        init_p01 = 40'sd0;
        init_p11 = 40'sd107374182;
        init_valid = 1'b1;
        @(negedge clk);
        init_valid = 1'b0;

        check(state0 == 0 && state1 == 0, "initial state loaded");
        check(p00 == 40'sd107374182 && p01 == 0 &&
              p11 == 40'sd107374182, "initial covariance loaded");

        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // ST_UPDATE_X is enum value 7. k0_r is registered in ST_GAIN,
        // so inspect after the FSM enters ST_UPDATE_X.
        cycles = 0;
        while ((dut.state != 4'd7) && (cycles < 100)) begin
            @(negedge clk);
            cycles = cycles + 1;
        end
        check(dut.state == 4'd7, "FSM reaches state-update stage");
        check($signed(dut.k0_r) > 32767,
              "K0 is greater than +1.0 in Q3.15");
        check($signed(dut.k0_r) < 131072,
              "K0 remains within the positive Q3.15 representable range");
        check($signed(dut.k1_r) >= -1 && $signed(dut.k1_r) <= 1,
              "K1 is approximately zero for H1=0");

        wait_done();

        check(error_status == 1'b0, "no transaction error");
        check(overflow_seen == 1'b0, "valid gain greater than +1 causes no overflow");
        check(!busy, "busy deasserts after completion");

        // Expected x0 ~= K0 * 0.1 ~= 0.3448, raw Q1.15 ~= 11300.
        check(state0 > 10500 && state0 < 12200,
              "updated state0 is consistent with gain greater than +1");
        check(state1 >= -2 && state1 <= 2,
              "state1 remains approximately zero");

        // Expected P00 ~= 0.1 - 3.448*0.025 ~= 0.0138 (Q30 raw ~14.8M).
        check(p00 > 40'sd12000000 && p00 < 40'sd18000000,
              "updated P00 is in the expected positive range");
        check(p01 >= -2 && p01 <= 2,
              "P01 remains approximately zero");
        check(p11 > 40'sd100000000 && p11 < 40'sd115000000,
              "unobserved-state variance remains approximately 0.1");
        check(p00 > 0 && p11 > 0, "covariance diagonal remains positive");

        $display("");
        $display("KAL-004G GAIN-RANGE SUMMARY: checks=%0d failures=%0d",
                 checks, failures);
        if (failures == 0)
            $display("ALL KAL-004G GAIN-RANGE CHECKS PASSED");
        else
            $display("KAL-004G GAIN-RANGE TEST FAILED");
        $finish;
    end

endmodule
