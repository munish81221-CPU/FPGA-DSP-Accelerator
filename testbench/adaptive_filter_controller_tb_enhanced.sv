`timescale 1ns/1ps

module adaptive_filter_controller_tb;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic mode = 1'b0;

    logic signed [15:0] x0, x1, x2, x3, desired, mu;
    logic signed [33:0] epsilon_q30;

    logic done, busy;
    logic signed [15:0] y_out, error_out, w0, w1, w2, w3;
    logic overflow_seen, error_status;

    logic cmd_valid, cmd_ready;
    logic [2:0] opcode;
    logic signed [15:0] operand_a, operand_b, result;
    logic result_valid, engine_overflow, engine_busy;
    logic signed [39:0] accumulator_value;

    integer total_checks = 0;

    localparam logic [33:0] EPSILON_001_Q30 = 34'sd10737418;

    always #5 clk = ~clk;

    arithmetic_engine u_engine (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),
        .opcode(opcode),
        .operand_a(operand_a),
        .operand_b(operand_b),
        .result(result),
        .result_valid(result_valid),
        .busy(engine_busy),
        .overflow(engine_overflow),
        .accumulator_value(accumulator_value)
    );

    adaptive_filter_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .mode(mode),
        .x0(x0), .x1(x1), .x2(x2), .x3(x3),
        .desired(desired),
        .mu(mu),
        .epsilon_q30(epsilon_q30),
        .done(done),
        .busy(busy),
        .y_out(y_out),
        .error_out(error_out),
        .w0(w0), .w1(w1), .w2(w2), .w3(w3),
        .overflow_seen(overflow_seen),
        .error_status(error_status),
        .cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),
        .opcode(opcode),
        .operand_a(operand_a),
        .operand_b(operand_b),
        .result_valid(result_valid),
        .result(result),
        .engine_overflow(engine_overflow),
        .accumulator_value(accumulator_value)
    );

    task automatic set_reference_vectors;
        begin
            // Q1.15 samples: {+0.25, +0.50, -0.25, +0.125}
            x0 = 16'sd8192;
            x1 = 16'sd16384;
            x2 = -16'sd8192;
            x3 = 16'sd4096;
            desired = 16'sd16384; // +0.5
            mu = 16'sd8192;       // +0.25
            epsilon_q30 = EPSILON_001_Q30;
        end
    endtask

    task automatic reset_dut;
        begin
            @(negedge clk);
            start = 1'b0;
            rst_n = 1'b0;
            repeat (2) @(negedge clk);

            if (busy !== 1'b0 || done !== 1'b0)
                $fatal(1, "RESET CHECK FAILED: busy=%b done=%b", busy, done);
            if ({w0,w1,w2,w3} !== 64'sd0)
                $fatal(1, "RESET CHECK FAILED: weights were not cleared");
            if (error_status !== 1'b0 || overflow_seen !== 1'b0)
                $fatal(1, "RESET CHECK FAILED: status outputs not cleared");

            rst_n = 1'b1;
            @(negedge clk);
            total_checks = total_checks + 1;
            $display("PASS reset: weights/status/control cleared");
        end
    endtask

    task automatic wait_for_completion(
        input logic expect_error,
        input integer timeout_cycles,
        input [8*32-1:0] test_name
    );
        integer cycles;
        begin
            cycles = 0;
            while ((done !== 1'b1) && (cycles < timeout_cycles)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end

            if (done !== 1'b1)
                $fatal(1, "TIMEOUT: %0s after %0d cycles", test_name, timeout_cycles);
            if (busy !== 1'b0)
                $fatal(1, "%0s: busy was high when done asserted", test_name);
            if (error_status !== expect_error)
                $fatal(1, "%0s: error_status=%b expected=%b",
                       test_name, error_status, expect_error);

            total_checks = total_checks + 1;
            $display("PASS %-24s cycles=%0d y=%0d e=%0d weights={%0d,%0d,%0d,%0d} overflow=%b error=%b",
                     test_name, cycles, y_out, error_out, w0, w1, w2, w3,
                     overflow_seen, error_status);

            // done is required to be a one-cycle pulse.
            @(negedge clk);
            if (done !== 1'b0)
                $fatal(1, "%0s: done did not return low after one cycle", test_name);
        end
    endtask

    task automatic run_iteration(
        input logic selected_mode,
        input logic expect_error,
        input [8*32-1:0] test_name
    );
        begin
            @(negedge clk);
            mode = selected_mode;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            wait_for_completion(expect_error, 700, test_name);
        end
    endtask

    task automatic check_weights(
        input logic signed [15:0] exp_w0,
        input logic signed [15:0] exp_w1,
        input logic signed [15:0] exp_w2,
        input logic signed [15:0] exp_w3,
        input [8*32-1:0] test_name
    );
        begin
            if (w0 !== exp_w0 || w1 !== exp_w1 ||
                w2 !== exp_w2 || w3 !== exp_w3)
                $fatal(1, "%0s: weights mismatch, got {%0d,%0d,%0d,%0d}, expected {%0d,%0d,%0d,%0d}",
                       test_name, w0, w1, w2, w3,
                       exp_w0, exp_w1, exp_w2, exp_w3);
            total_checks = total_checks + 1;
            $display("PASS %-24s weights={%0d,%0d,%0d,%0d}",
                     test_name, w0, w1, w2, w3);
        end
    endtask

    initial begin
        set_reference_vectors();
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        // TEST 1: Original LMS iteration 1 regression.
        run_iteration(1'b0, 1'b0, "LMS iteration 1");
        if (y_out !== 16'sd0 || error_out !== 16'sd16384)
            $fatal(1, "LMS iteration 1: output/error mismatch");
        check_weights(16'sd1024, 16'sd2048, -16'sd1024, 16'sd512,
                      "LMS iteration 1 weights");
        if (overflow_seen) $fatal(1, "Unexpected overflow in LMS iteration 1");

        // TEST 2: LMS weights persist and update over a second iteration.
        run_iteration(1'b0, 1'b0, "LMS iteration 2");
        if (y_out !== 16'sd1600 || error_out !== 16'sd14784)
            $fatal(1, "LMS iteration 2: output/error mismatch");
        check_weights(16'sd1948, 16'sd3896, -16'sd1948, 16'sd974,
                      "LMS iteration 2 weights");
        if (overflow_seen) $fatal(1, "Unexpected overflow in LMS iteration 2");

        // TEST 3: NLMS directed regression, including positive and negative samples.
        reset_dut();
        set_reference_vectors();
        run_iteration(1'b1, 1'b0, "NLMS mixed-sign inputs");
        if (y_out !== 16'sd0 || error_out !== 16'sd16384)
            $fatal(1, "NLMS mixed-sign inputs: output/error mismatch");
        check_weights(16'sd2556, 16'sd5112, -16'sd2556, 16'sd1278,
                      "NLMS mixed-sign weights");
        if (overflow_seen) $fatal(1, "Unexpected overflow in NLMS mixed-sign test");

        // TEST 4: Zero input energy with positive epsilon must not divide by zero.
        reset_dut();
        x0 = 0; x1 = 0; x2 = 0; x3 = 0;
        desired = 16'sd16384;
        mu = 16'sd8192;
        epsilon_q30 = EPSILON_001_Q30;
        run_iteration(1'b1, 1'b0, "NLMS zero input energy");
        if (y_out !== 16'sd0 || error_out !== 16'sd16384)
            $fatal(1, "Zero-energy NLMS: output/error mismatch");
        check_weights(16'sd0, 16'sd0, 16'sd0, 16'sd0,
                      "NLMS zero-energy weights");
        if (overflow_seen) $fatal(1, "Unexpected overflow in zero-energy NLMS");

        // TEST 5: Non-positive epsilon is rejected; no weights may be updated.
        reset_dut();
        set_reference_vectors();
        epsilon_q30 = 0;
        run_iteration(1'b1, 1'b1, "NLMS zero epsilon rejected");
        check_weights(16'sd0, 16'sd0, 16'sd0, 16'sd0,
                      "Invalid epsilon no update");

        // TEST 6: All-negative samples exercise signed energy/correction handling.
        reset_dut();
        x0 = -16'sd8192;
        x1 = -16'sd8192;
        x2 = -16'sd8192;
        x3 = -16'sd8192;
        desired = 16'sd16384;
        mu = 16'sd8192;
        epsilon_q30 = EPSILON_001_Q30;
        run_iteration(1'b1, 1'b0, "NLMS all-negative inputs");
        if (y_out !== 16'sd0 || error_out !== 16'sd16384)
            $fatal(1, "All-negative NLMS: output/error mismatch");
        check_weights(-16'sd3938, -16'sd3938, -16'sd3938, -16'sd3938,
                      "NLMS all-negative weights");

        // TEST 7: A start pulse while busy must not corrupt the active LMS iteration.
        reset_dut();
        set_reference_vectors();
        @(negedge clk);
        mode = 1'b0;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        wait (busy === 1'b1);
        @(negedge clk);
        // Attempt a second request with different inputs/mode while first is active.
        mode = 1'b1;
        x0 = 0; x1 = 0; x2 = 0; x3 = 0;
        desired = -16'sd1234;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        wait_for_completion(1'b0, 700, "Start while busy");
        if (y_out !== 16'sd0 || error_out !== 16'sd16384)
            $fatal(1, "Busy-start test: active iteration inputs/mode were not latched");
        check_weights(16'sd1024, 16'sd2048, -16'sd1024, 16'sd512,
                      "Busy-start active result");
        // Give the FSM one more cycle: the rejected request must not launch another job.
        @(negedge clk);
        if (busy !== 1'b0 || done !== 1'b0)
            $fatal(1, "Busy-start test: unexpected second iteration started");

        // TEST 8: Reset aborts an active operation and leaves the controller reusable.
        reset_dut();
        set_reference_vectors();
        @(negedge clk);
        mode = 1'b1;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        wait (busy === 1'b1);
        repeat (8) @(negedge clk);
        rst_n = 1'b0;
        repeat (2) @(negedge clk);
        if (busy !== 1'b0 || done !== 1'b0)
            $fatal(1, "Reset-while-busy: busy/done not cleared");
        check_weights(16'sd0, 16'sd0, 16'sd0, 16'sd0,
                      "Reset abort weights");
        rst_n = 1'b1;
        @(negedge clk);
        set_reference_vectors();
        run_iteration(1'b0, 1'b0, "LMS after reset abort");
        check_weights(16'sd1024, 16'sd2048, -16'sd1024, 16'sd512,
                      "Reusable after reset");

        $display("");
        $display("ALL ADAPTIVE FILTER TESTS PASSED: %0d checks", total_checks);
        $finish;
    end

    initial begin
        #200000;
        $fatal(1, "GLOBAL TESTBENCH WATCHDOG: simulation exceeded 200 us");
    end
endmodule
