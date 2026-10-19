`timescale 1ns/1ps

module lms_controller_tb;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;

    logic signed [15:0] x0, x1, x2, x3, desired, mu;
    logic done, busy;
    logic signed [15:0] y_out, error_out;
    logic signed [15:0] w0, w1, w2, w3;
    logic overflow_seen;

    logic cmd_valid;
    logic cmd_ready;
    logic [2:0] opcode;
    logic signed [15:0] operand_a, operand_b;
    logic result_valid;
    logic signed [15:0] result;
    logic engine_overflow;

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
        .busy(),
        .overflow(engine_overflow)
    );

    lms_controller dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .x0(x0), .x1(x1), .x2(x2), .x3(x3),
        .desired(desired), .mu(mu),
        .done(done), .busy(busy),
        .y_out(y_out), .error_out(error_out),
        .w0(w0), .w1(w1), .w2(w2), .w3(w3),
        .overflow_seen(overflow_seen),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .opcode(opcode), .operand_a(operand_a), .operand_b(operand_b),
        .result_valid(result_valid), .result(result),
        .engine_overflow(engine_overflow)
    );

task automatic start_iteration;
    integer cycles;
    begin
        // Generate a one-cycle start pulse.
        @(negedge clk);
        start = 1'b1;

        @(negedge clk);
        start = 1'b0;

        // Wait for completion, with a bounded timeout.
        cycles = 0;

        while ((done !== 1'b1) && (cycles < 300)) begin
            @(posedge clk);
            cycles = cycles + 1;
        end

        if (done !== 1'b1)
            $fatal(1, "TIMEOUT: LMS iteration did not finish");

        // Allow outputs to settle before checking them.
        @(negedge clk);
    end
endtask

    initial begin
        x0 = 16'sd8192;   // +0.25
        x1 = 16'sd16384;  // +0.50
        x2 = -16'sd8192;  // -0.25
        x3 = 16'sd4096;   // +0.125
        desired = 16'sd16384; // +0.50
        mu = 16'sd8192;       // +0.25

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        // First iteration starts with all weights equal to zero:
        // y=0, e=0.5, mu*e=0.125.
        // Expected corrections: [1024, 2048, -1024, 512].
        start_iteration();

        if (y_out !== 16'sd0)       $fatal(1, "Iteration 1 y expected 0, got %0d", y_out);
        if (error_out !== 16'sd16384) $fatal(1, "Iteration 1 error expected 16384, got %0d", error_out);
        if (w0 !== 16'sd1024)       $fatal(1, "Iteration 1 w0 expected 1024, got %0d", w0);
        if (w1 !== 16'sd2048)       $fatal(1, "Iteration 1 w1 expected 2048, got %0d", w1);
        if (w2 !== -16'sd1024)      $fatal(1, "Iteration 1 w2 expected -1024, got %0d", w2);
        if (w3 !== 16'sd512)        $fatal(1, "Iteration 1 w3 expected 512, got %0d", w3);
        if (overflow_seen !== 1'b0) $fatal(1, "Unexpected overflow in iteration 1");

        $display("PASS iteration 1: y=%0d e=%0d weights={%0d,%0d,%0d,%0d}",
                 y_out, error_out, w0, w1, w2, w3);

        // The weights persist. With the same sample, y is exactly 1600 in
        // Q1.15 (0.048828125). Expected next weights: [1948,3896,-1948,974].
        start_iteration();

        if (y_out !== 16'sd1600)    $fatal(1, "Iteration 2 y expected 1600, got %0d", y_out);
        if (error_out !== 16'sd14784) $fatal(1, "Iteration 2 error expected 14784, got %0d", error_out);
        if (w0 !== 16'sd1948)       $fatal(1, "Iteration 2 w0 expected 1948, got %0d", w0);
        if (w1 !== 16'sd3896)       $fatal(1, "Iteration 2 w1 expected 3896, got %0d", w1);
        if (w2 !== -16'sd1948)      $fatal(1, "Iteration 2 w2 expected -1948, got %0d", w2);
        if (w3 !== 16'sd974)        $fatal(1, "Iteration 2 w3 expected 974, got %0d", w3);
        if (overflow_seen !== 1'b0) $fatal(1, "Unexpected overflow in iteration 2");

        $display("PASS iteration 2: y=%0d e=%0d weights={%0d,%0d,%0d,%0d}",
                 y_out, error_out, w0, w1, w2, w3);
        $display("ALL LMS CONTROLLER CHECKS PASSED");
        $finish;
    end
endmodule
