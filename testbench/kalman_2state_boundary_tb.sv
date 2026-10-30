`timescale 1ns/1ps

// KAL-004B Phase 2: boundary-focused tests for kalman_2state_ip.
// Tests the observable RTL contract: state saturation/overflow reporting,
// covariance saturation/overflow reporting, and invalid innovation covariance
// handling with preservation of the previous committed state/covariance.
//
// Run from the project root:
// iverilog -g2012 -s kalman_2state_boundary_tb -o kalman_boundary rtl/kalman_2state_ip.sv testbench/kalman_2state_boundary_tb.sv
// vvp kalman_boundary

module kalman_2state_boundary_tb;
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
    logic signed [15:0] cfg_h0 = '0, cfg_h1 = '0;
    logic signed [39:0] cfg_r = 40'sd10737418; // approximately 0.01 in Q30
    logic signed [15:0] measurement = '0;

    logic signed [15:0] state0, state1;
    logic signed [39:0] p00, p01, p11;

    integer checks = 0;
    integer failures = 0;

    localparam logic signed [39:0] P40_MAX = 40'sh7FFFFFFFFF;
    localparam logic signed [39:0] P40_MIN = 40'sh8000000000;

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
            if (condition) $display("PASS %s", label);
            else begin
                failures = failures + 1;
                $display("FAIL %s", label);
            end
        end
    endtask

    task automatic reset_dut;
        begin
            @(negedge clk); rst_n = 1'b0; start = 1'b0; init_valid = 1'b0;
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

    task automatic run_transaction;
        integer cycles;
        begin
            @(negedge clk); start = 1'b1;
            @(negedge clk); start = 1'b0;
            cycles = 0;
            while ((done !== 1'b1) && (cycles < 30)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end
            if (cycles >= 30) begin
                failures = failures + 1;
                checks = checks + 1;
                $display("FAIL transaction timeout");
            end else begin
                checks = checks + 1;
                $display("PASS transaction completed in %0d wait cycles", cycles);
            end
            @(negedge clk); // let done return low before next operation
        end
    endtask

    initial begin
        // ---------- B07: state saturation ----------
        reset_dut();
        cfg_f00 = 16'sd32767; // approximately +2.0 in Q2.14
        cfg_f01 = 0; cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 0; cfg_h1 = 0; cfg_r = 40'sd10737418; // positive S
        measurement = 0;
        initialize(16'sd24576, 16'sd4096, 40'sd10737418, 0, 40'sd10737418);
        run_transaction();
        check(state0 == 16'sd32767, "B07 positive state clamps to +32767");
        check(overflow_seen == 1'b1, "B07 state saturation sets overflow_seen");
        check(error_status == 1'b0, "B07 saturation does not set error_status");
        check(done == 1'b0, "B07 done is a pulse, not permanently asserted");

        // ---------- B08: covariance saturation ----------
        // P00 begins at max Q10.30 and F00 is approximately 2.0, forcing
        // intermediate covariance products beyond signed 40-bit range.
        reset_dut();
        cfg_f00 = 16'sd32767; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 0; cfg_h1 = 0; cfg_r = 40'sd10737418;
        initialize(0, 0, P40_MAX, 0, 40'sd10737418);
        run_transaction();
        check(p00 == P40_MAX, "B08 covariance clamps to signed 40-bit maximum");
        check(overflow_seen == 1'b1, "B08 covariance overflow sets overflow_seen");
        check(error_status == 1'b0, "B08 covariance saturation does not set error_status");

        // ---------- B09: zero innovation covariance ----------
        // H=0 and R=0 gives S=0. The transaction must fail and preserve the
        // previously committed state and covariance.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 0; cfg_h1 = 0; cfg_r = 0;
        initialize(16'sd4096, -16'sd2048, 40'sd53687091, 40'sd0, 40'sd107374182);
        run_transaction();
        check(error_status == 1'b1, "B09 zero S raises error_status");
        check(state0 == 16'sd4096 && state1 == -16'sd2048,
              "B09 failed update preserves prior state");
        check(p00 == 40'sd53687091 && p01 == 0 && p11 == 40'sd107374182,
              "B09 failed update preserves prior covariance");
        check(overflow_seen == 1'b0, "B09 zero S alone does not report arithmetic overflow");

        // ---------- B10: negative innovation covariance ----------
        // Invalid negative R forces S<0 with H=0. This is an invalid
        // configuration, used only to exercise the RTL's defensive error path.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 0; cfg_h1 = 0; cfg_r = -40'sd1;
        initialize(-16'sd3000, 16'sd7000, 40'sd53687091, 0, 40'sd107374182);
        run_transaction();
        check(error_status == 1'b1, "B10 negative S raises error_status");
        check(state0 == -16'sd3000 && state1 == 16'sd7000,
              "B10 negative-S failure preserves prior state");
        check(p00 == 40'sd53687091 && p01 == 0 && p11 == 40'sd107374182,
              "B10 negative-S failure preserves prior covariance");

        // ---------- B11: start before initialization ----------
        reset_dut();
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        check(done == 1'b1, "B11 uninitialized start returns done pulse");
        check(error_status == 1'b1, "B11 uninitialized start raises error_status");
        check(state0 == 0 && state1 == 0 && p00 == 0 && p01 == 0 && p11 == 0,
              "B11 uninitialized start does not modify outputs");

        if (failures == 0)
            $display("\\nKAL-004B BOUNDARY SUMMARY: passed=%0d failed=%0d", checks, failures);
        else
            $display("\\nKAL-004B BOUNDARY SUMMARY: passed=%0d failed=%0d", checks-failures, failures);

        if (failures != 0) $fatal(1, "KAL-004B boundary verification failed");
        $display("ALL KAL-004B BOUNDARY CHECKS PASSED");
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "TIMEOUT: boundary testbench did not finish");
    end
endmodule
