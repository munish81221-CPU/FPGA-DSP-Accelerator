`timescale 1ns/1ps

// KAL-004D: Sequential protocol and transaction-control verification.
// Covers reset, initialization, start acceptance, busy/done behavior,
// start-while-busy rejection, configuration snapshotting, repeated transactions,
// init-while-busy behavior, and reset during an active transaction.
//
// Run from project root:
// iverilog -g2012 -s kalman_2state_sequential_tb -o kalman_sequential rtl/kalman_2state_ip.sv testbench/kalman_2state_sequential_tb.sv
// vvp kalman_sequential

module kalman_2state_sequential_tb;
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
    integer done_pulses = 0;
    integer cycles;
    logic signed [15:0] snapshot_state0;
    logic signed [39:0] snapshot_p00;

    localparam logic signed [39:0] Q30_QUARTER   = 40'sd268435456;
    localparam logic signed [39:0] Q30_TENTH     = 40'sd107374182;
    localparam logic signed [39:0] Q30_HUNDREDTH = 40'sd10737418;

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

    // Count done pulses at the active clock edge for transaction-count checks.
    always @(posedge clk) begin
        if (!rst_n) done_pulses <= 0;
        else if (done) done_pulses <= done_pulses + 1;
    end

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
            @(negedge clk);
        end
    endtask

    task automatic launch;
        begin
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
        end
    endtask

    task automatic wait_for_done(input integer max_cycles);
        integer waited;
        begin
            waited = 0;
            while ((done !== 1'b1) && (waited < max_cycles)) begin
                @(negedge clk);
                waited = waited + 1;
            end
            check(done === 1'b1, $sformatf("transaction completes within %0d cycles", max_cycles));
            if (done === 1'b1)
                $display("      observed completion after %0d wait cycles", waited);
            @(negedge clk);
        end
    endtask

    initial begin
        // Default safe configuration.
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 16'sd16384; cfg_h1 = 0;
        cfg_r = Q30_HUNDREDTH;
        measurement = 16'sd16384;

        // D01: reset establishes idle outputs and clears status.
        reset_dut();
        check(busy === 1'b0, "D01 reset leaves busy low");
        check(done === 1'b0, "D01 reset leaves done low");
        check(error_status === 1'b0, "D01 reset clears error_status");
        check(overflow_seen === 1'b0, "D01 reset clears overflow_seen");

        // D02: start before initialization is rejected and reports an error.
        launch();
        check(done === 1'b1, "D02 rejected uninitialized start emits done pulse");
        check(error_status === 1'b1, "D02 rejected uninitialized start sets error_status");
        check(busy === 1'b0, "D02 rejected start does not enter busy state");
        @(negedge clk);
        check(done === 1'b0, "D02 done pulse clears on following cycle");

        // D03: initialize and verify state/covariance are loaded.
        initialize(16'sd0, -16'sd2048, Q30_QUARTER, 0, Q30_TENTH);
        check(state0 == 0 && state1 == -16'sd2048, "D03 initialization loads state");
        check(p00 == Q30_QUARTER && p01 == 0 && p11 == Q30_TENTH,
              "D03 initialization loads covariance");
        check(error_status === 1'b0, "D03 initialization clears prior error");

        // D04: accepted start raises busy; busy remains asserted until completion.
        launch();
        check(busy === 1'b1, "D04 accepted transaction asserts busy");
        check(done === 1'b0, "D04 done is low while transaction is running");
        wait_for_done(20);
        check(busy === 1'b0, "D04 busy clears after completion");
        check(done === 1'b0, "D04 done returns low after its pulse");
        check(error_status === 1'b0, "D04 normal transaction has no error");
        check(overflow_seen === 1'b0, "D04 normal transaction has no overflow");

        // D05: config and measurement are snapshotted at start. Change them while
        // busy; this transaction should still use the original +0.5 measurement.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 16'sd16384; cfg_h1 = 0;
        cfg_r = Q30_HUNDREDTH;
        measurement = 16'sd16384;
        initialize(0, 0, Q30_QUARTER, 0, Q30_QUARTER);
        launch();
        check(busy === 1'b1, "D05 transaction entered busy before input changes");
        measurement = -16'sd16384;
        cfg_r = Q30_TENTH;
        cfg_h0 = 0;
        repeat (2) @(negedge clk);
        wait_for_done(20);
        check(state0 > 0, "D05 in-flight transaction uses latched positive measurement/config");
        check(error_status === 1'b0, "D05 config changes while busy do not corrupt transaction");

        // D06: a start pulse while busy must not create a second transaction.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 16'sd16384; cfg_h1 = 0;
        cfg_r = Q30_HUNDREDTH;
        measurement = 16'sd8192;
        initialize(0, 0, Q30_QUARTER, 0, Q30_QUARTER);
        done_pulses = 0;
        launch();
        check(busy === 1'b1, "D06 first transaction is busy");
        // Assert a second start while the first is still active.
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        wait_for_done(20);
        repeat (3) @(negedge clk);
        check(done_pulses == 1, "D06 start-while-busy does not queue a second transaction");
        check(busy === 1'b0, "D06 controller returns to idle after single transaction");

        // D07: initialization while busy is ignored; current transaction completes
        // using its previously committed state, not the attempted new initial state.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 16'sd16384; cfg_h1 = 0;
        cfg_r = Q30_HUNDREDTH;
        measurement = 16'sd8192;
        initialize(0, 0, Q30_QUARTER, 0, Q30_QUARTER);
        launch();
        check(busy === 1'b1, "D07 transaction busy before init_valid pulse");
        @(negedge clk);
        init_x0 = 16'sd12000; init_x1 = -16'sd12000;
        init_p00 = Q30_TENTH; init_p01 = 0; init_p11 = Q30_TENTH;
        init_valid = 1'b1;
        @(negedge clk); init_valid = 1'b0;
        wait_for_done(20);
        check(state0 != 16'sd12000 && state1 != -16'sd12000,
              "D07 init_valid while busy does not overwrite committed result");

        // D08: reset during active transaction aborts it and clears protocol/status.
        reset_dut();
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 16'sd16384; cfg_h1 = 0;
        cfg_r = Q30_HUNDREDTH;
        measurement = 16'sd16384;
        initialize(0, 0, Q30_QUARTER, 0, Q30_QUARTER);
        launch();
        check(busy === 1'b1, "D08 transaction active before reset abort");
        @(negedge clk); rst_n = 1'b0; start = 1'b0; init_valid = 1'b0;
        repeat (2) @(negedge clk);
        check(busy === 1'b0, "D08 reset abort clears busy");
        check(done === 1'b0, "D08 reset abort clears done");
        check(error_status === 1'b0, "D08 reset abort clears error status");
        check(overflow_seen === 1'b0, "D08 reset abort clears overflow status");
        check(state0 == 0 && state1 == 0 && p00 == 0 && p01 == 0 && p11 == 0,
              "D08 reset abort clears outputs");
        rst_n = 1'b1;
        @(negedge clk);
        check(busy === 1'b0 && done === 1'b0, "D08 controller remains idle after reset release");

        // D09: repeated sequential transactions work without reset between them.
        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384;
        cfg_q00 = 0; cfg_q01 = 0; cfg_q11 = 0;
        cfg_h0 = 16'sd16384; cfg_h1 = 0;
        cfg_r = Q30_HUNDREDTH;
        measurement = 16'sd4096;
        initialize(0, 0, Q30_QUARTER, 0, Q30_QUARTER);
        done_pulses = 0;
        launch();
        wait_for_done(20);
        snapshot_state0 = state0;
        snapshot_p00 = p00;
        check(done === 1'b0 && busy === 1'b0, "D09 first transaction returns to idle");
        measurement = 16'sd8192;
        launch();
        wait_for_done(20);
        check(done_pulses == 2, "D09 exactly two completion pulses observed");
        check(state0 > snapshot_state0, "D09 second measurement updates retained state");
        check(p00 <= snapshot_p00, "D09 covariance persists and updates between transactions");
        check(error_status === 1'b0, "D09 repeated transactions finish without error");

        $display("\\nKAL-004D SUMMARY: checks=%0d passed=%0d failed=%0d",
                 checks, checks-failures, failures);
        if (failures != 0) $fatal(1, "KAL-004D sequential verification failed");
        $display("ALL KAL-004D SEQUENTIAL CHECKS PASSED");
        $finish;
    end

    initial begin
        #200000;
        $fatal(1, "TIMEOUT: KAL-004D testbench did not finish");
    end
endmodule
