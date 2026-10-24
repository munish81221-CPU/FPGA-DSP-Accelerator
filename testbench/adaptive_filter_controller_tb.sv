`timescale 1ns/1ps

module adaptive_filter_controller_tb;
    logic clk = 0;
    logic rst_n = 0;
    logic start = 0;
    logic mode = 0;
    logic signed [15:0] x0, x1, x2, x3, desired, mu;
    logic signed [33:0] epsilon_q30;
    logic done, busy;
    logic signed [15:0] y_out, error_out, w0, w1, w2, w3;
    logic overflow_seen, error_status;
    logic cmd_valid, cmd_ready;
    logic [2:0] opcode;
    logic signed [15:0] operand_a, operand_b, result;
    logic result_valid, engine_overflow;
    logic engine_busy;
    logic signed [39:0] accumulator_value;

    always #5 clk = ~clk;

    arithmetic_engine u_engine (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .opcode(opcode), .operand_a(operand_a), .operand_b(operand_b),
        .result(result), .result_valid(result_valid), .busy(engine_busy),
        .overflow(engine_overflow), .accumulator_value(accumulator_value)
    );

    adaptive_filter_controller dut (
        .clk(clk), .rst_n(rst_n), .start(start), .mode(mode),
        .x0(x0), .x1(x1), .x2(x2), .x3(x3),
        .desired(desired), .mu(mu), .epsilon_q30(epsilon_q30),
        .done(done), .busy(busy), .y_out(y_out), .error_out(error_out),
        .w0(w0), .w1(w1), .w2(w2), .w3(w3),
        .overflow_seen(overflow_seen), .error_status(error_status),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .opcode(opcode),
        .operand_a(operand_a), .operand_b(operand_b),
        .result_valid(result_valid), .result(result),
        .engine_overflow(engine_overflow), .accumulator_value(accumulator_value)
    );

    task automatic run_iteration(input logic selected_mode);
        integer cycles;
        begin
            @(negedge clk);
            mode = selected_mode;
            start = 1;
            @(negedge clk);
            start = 0;
            cycles = 0;
            while ((done !== 1'b1) && (cycles < 700)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end
            if (done !== 1'b1) $fatal(1, "TIMEOUT mode=%0d", selected_mode);
            if (busy !== 1'b0) $fatal(1, "busy remained high at done");
            if (error_status !== 1'b0) $fatal(1, "error_status set mode=%0d", selected_mode);
            $display("PASS mode=%s cycles=%0d y=%0d e=%0d weights={%0d,%0d,%0d,%0d} overflow=%0b",
                     selected_mode ? "NLMS" : "LMS", cycles, y_out, error_out,
                     w0, w1, w2, w3, overflow_seen);
        end
    endtask

    initial begin
        x0 = 16'sd8192;    // +0.25 Q1.15
        x1 = 16'sd16384;   // +0.50 Q1.15
        x2 = -16'sd8192;   // -0.25 Q1.15
        x3 = 16'sd4096;    // +0.125 Q1.15
        desired = 16'sd16384;
        mu = 16'sd8192;
        epsilon_q30 = 34'sd10737418; // approximately 0.01 in Q*.30

        repeat (3) @(negedge clk);
        rst_n = 1;

        // Verify existing LMS behavior first.
        run_iteration(1'b0);
        if (y_out !== 0 || error_out !== 16384) $fatal(1, "LMS iteration 1 output/error mismatch");
        if (w0 !== 1024 || w1 !== 2048 || w2 !== -1024 || w3 !== 512)
            $fatal(1, "LMS iteration 1 weights mismatch");
        if (overflow_seen) $fatal(1, "Unexpected LMS overflow");

        run_iteration(1'b0);
        if (y_out !== 1600 || error_out !== 14784) $fatal(1, "LMS iteration 2 output/error mismatch");
        if (w0 !== 1948 || w1 !== 3896 || w2 !== -1948 || w3 !== 974)
            $fatal(1, "LMS iteration 2 weights mismatch");
        if (overflow_seen) $fatal(1, "Unexpected LMS overflow iteration 2");

        // Reset weights, then check a directed NLMS iteration from zero weights.
        @(negedge clk); rst_n = 0;
        repeat (2) @(negedge clk);
        rst_n = 1;
        run_iteration(1'b1);
        if (y_out !== 0 || error_out !== 16384) $fatal(1, "NLMS output/error mismatch");
        // Energy = 0.390625; epsilon ~= 0.01; divider truncates toward zero.
        if (w0 !== 2556 || w1 !== 5112 || w2 !== -2556 || w3 !== 1278)
            $fatal(1, "NLMS weights mismatch: {%0d,%0d,%0d,%0d}", w0,w1,w2,w3);
        if (overflow_seen) $fatal(1, "Unexpected NLMS overflow");

        $display("ALL ADAPTIVE FILTER CHECKS PASSED");
        $finish;
    end
endmodule
