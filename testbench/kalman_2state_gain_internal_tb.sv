`timescale 1ns/1ps

// KAL-004I: Gain calculation internal-consistency and saturation verification.
// DUT: kalman_2state_ip in rtl/kalman_2state_controller_gain18_diag.sv
//
// Numeric formats:
//   H: Q2.14
//   P, HP, S: Q10.30
//   K: signed 18-bit Q3.15
//
// This testbench samples the DUT precisely while its FSM is in ST_GAIN.
// It independently recomputes K = (HP * 2^15) / S using 64-bit signed
// arithmetic, then checks the DUT intermediate, stored gain, and overflow
// reporting. It covers a representable positive gain, positive saturation,
// and negative saturation.

module kalman_2state_gain_internal_tb;

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

    // Captured during the actual ST_GAIN operation, before later FSM states
    // can reuse the DUT's shared temporary t0.
    logic gain_captured;
    logic signed [63:0] captured_hp0;
    logic signed [63:0] captured_s;
    logic signed [63:0] captured_num0;
    logic signed [63:0] captured_expected_k0;
    logic signed [63:0] captured_dut_t0;
    logic signed [17:0] captured_k0;
    logic captured_overflow;
    logic [7:0] captured_overflow_stage;
    logic captured_error;

    logic signed [63:0] ref_num;
    logic signed [63:0] ref_gain;
    logic signed [17:0] expected_stored_gain;
    logic expected_overflow;

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

    // Capture values at the ST_GAIN edge. In this RTL enum ST_GAIN is 4'd6.
    // hp0 and s_r are the values used by the gain division in that state.
    always @(posedge clk) begin
        if (rst_n && (dut.state == 4'd6)) begin
            captured_hp0 = {{24{dut.hp0[39]}}, dut.hp0};
            captured_s = {{24{dut.s_r[39]}}, dut.s_r};
            captured_num0 = captured_hp0 <<< 15;
            captured_expected_k0 = captured_num0 / captured_s;
            #1;
            captured_dut_t0 = dut.t0;
            captured_k0 = dut.k0_r;
            captured_overflow = dut.overflow_seen;
            captured_overflow_stage = dut.overflow_stage;
            captured_error = dut.error_status;
            gain_captured = 1'b1;
        end
    end

    task automatic reset_and_initialize;
        begin
            rst_n = 1'b0;
            init_valid = 1'b0;
            start = 1'b0;
            gain_captured = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);

            init_x0 = 16'sd0;
            init_x1 = 16'sd0;
            init_p00 = 40'sd107374;       // approximately 1e-4
            init_p01 = 40'sd0;
            init_p11 = 40'sd107374182;    // approximately 0.1
            init_valid = 1'b1;
            @(negedge clk);
            init_valid = 1'b0;
            @(negedge clk);
            check(!busy, "controller is idle after initialization");
        end
    endtask

    task automatic launch_and_wait;
        begin
            gain_captured = 1'b0;
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
            check(gain_captured === 1'b1, "gain arithmetic was captured in ST_GAIN");
        end
    endtask

    task automatic verify_gain(input logic should_overflow,
                               input logic signed [17:0] expected_limit,
                               input string case_name);
        begin
            $display("");
            $display("---- %s ----", case_name);
            $display("Captured HP0       = %0d", captured_hp0);
            $display("Captured S         = %0d", captured_s);
            $display("Independent num    = %0d", captured_num0);
            $display("Independent K      = %0d", captured_expected_k0);
            $display("DUT t0 at ST_GAIN  = %0d", captured_dut_t0);
            $display("Stored K0          = %0d", captured_k0);
            $display("Overflow/stage     = %b/%0d",
                     captured_overflow, captured_overflow_stage);

            check(captured_s > 0, "innovation covariance S is strictly positive");
            check(captured_dut_t0 == captured_expected_k0,
                  "DUT unsaturated gain exactly matches independent 64-bit reference");

            if (should_overflow) begin
                check(captured_expected_k0 > 131071 ||
                      captured_expected_k0 < -131072,
                      "reference gain is outside signed Q3.15 range");
                check(captured_k0 == expected_limit,
                      "stored gain saturates to the correct signed endpoint");
                check(captured_overflow == 1'b1,
                      "gain range overflow is reported");
                check(captured_overflow_stage == 8'd14,
                      "overflow stage identifies the gain range check");
                check(captured_error == 1'b0,
                      "gain saturation does not become an S-domain error");
            end else begin
                check(captured_expected_k0 <= 131071 &&
                      captured_expected_k0 >= -131072,
                      "reference gain is inside signed Q3.15 range");
                check(captured_k0 == captured_expected_k0[17:0],
                      "representable gain is stored without saturation");
                check(captured_overflow == 1'b0,
                      "representable gain does not set overflow");
                check(captured_overflow_stage == 8'd0,
                      "no overflow stage is recorded for representable gain");
                check(captured_error == 1'b0,
                      "representable positive S does not produce an error");
            end
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
        cfg_r = 40'sd1074;
        measurement = 16'sd0;

        // CASE A: H=1, P~=1e-4, R~=1e-6 => K~=0.99, representable.
        $display("\nKAL-004I CASE A: representable positive gain");
        cfg_h0 = 16'sd16384;
        cfg_r = 40'sd1074;
        reset_and_initialize();
        launch_and_wait();
        verify_gain(1'b0, 18'sd0, "CASE A: positive gain in range");

        // CASE B: small positive H and tiny R => K exceeds +3.9999695.
        $display("\nKAL-004I CASE B: positive gain saturation");
        cfg_h0 = 16'sd2048;  // 0.125 in Q2.14
        cfg_r = 40'sd1;      // one Q10.30 LSB
        reset_and_initialize();
        launch_and_wait();
        verify_gain(1'b1, 18'sd131071, "CASE B: positive gain overflow");

        // CASE C: negative H makes HP negative while S=H*P*H^T+R stays positive.
        // Expected K is below -4 and must saturate to -131072.
        $display("\nKAL-004I CASE C: negative gain saturation");
        cfg_h0 = -16'sd2048; // -0.125 in Q2.14
        cfg_r = 40'sd1;
        reset_and_initialize();
        launch_and_wait();
        verify_gain(1'b1, -18'sd131072, "CASE C: negative gain overflow");

        $display("");
        $display("KAL-004I SUMMARY: checks=%0d failures=%0d", checks, failures);
        if (failures == 0)
            $display("ALL KAL-004I GAIN-CALCULATION CHECKS PASSED");
        else
            $display("KAL-004I GAIN-CALCULATION TEST FAILED");
        $finish;
    end

    initial begin
        #200000;
        $display("FAIL: global timeout");
        $finish;
    end

endmodule
