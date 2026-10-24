`timescale 1ns/1ps

module divider_unit_tb;


// ------------------------------------------------------------
// Parameters: Match the current 58-iteration divider
// ------------------------------------------------------------
localparam int NUM_W       = 16;
localparam int DEN_W       = 32;
localparam int QUOT_W      = 32;
localparam int SCALE_SHIFT = 41;
localparam int DIV_W       = NUM_W + SCALE_SHIFT;

// Fixed-point formats:
// Numerator:   signed Q1.15
// Denominator: signed Q3.29
// Quotient:    signed Q5.27

logic clk   = 1'b0;
logic rst_n = 1'b0;
logic start = 1'b0;

logic signed [NUM_W-1:0] numerator;
logic signed [DEN_W-1:0] denominator;

logic busy;
logic done;
logic div_by_zero;
logic overflow;

logic signed [QUOT_W-1:0] quotient;

// ------------------------------------------------------------
// DUT: Device Under Test
// ------------------------------------------------------------
divider_unit #(
    .NUM_W      (NUM_W),
    .DEN_W      (DEN_W),
    .QUOT_W     (QUOT_W),
    .SCALE_SHIFT(SCALE_SHIFT)
) dut (
    .clk         (clk),
    .rst_n       (rst_n),
    .start       (start),
    .numerator   (numerator),
    .denominator (denominator),
    .busy        (busy),
    .done        (done),
    .quotient    (quotient),
    .div_by_zero (div_by_zero),
    .overflow    (overflow)
);

// ------------------------------------------------------------
// Clock generation: 10 ns period
// ------------------------------------------------------------
always #5 clk = ~clk;

// ------------------------------------------------------------
// Reusable division test task
// ------------------------------------------------------------
task automatic run_division(
    input logic signed [NUM_W-1:0]  num,
    input logic signed [DEN_W-1:0]  den,
    input logic signed [QUOT_W-1:0] expected_quotient,
    input logic                     expected_div_by_zero,
    input logic                     expected_overflow
);

    int unsigned cycles;

    begin
        // Drive inputs away from the active clock edge.
        @(negedge clk);
        numerator   = num;
        denominator = den;
        start       = 1'b1;

        // Start is sampled at the next rising edge.
        @(negedge clk);
        start = 1'b0;

        // Wait for the completion pulse.
        cycles = 0;

        while ((done !== 1'b1) && (cycles < DIV_W + 10)) begin
            @(negedge clk);
            cycles++;
        end

        // Check completion.
        if (done !== 1'b1)
            $fatal(1,
                "TIMEOUT: numerator=%0d denominator=%0d",
                num, den);

        // Check divide-by-zero status.
        if (div_by_zero !== expected_div_by_zero)
            $fatal(1,
                "DIV0 MISMATCH: numerator=%0d denominator=%0d",
                num, den);

        // Check overflow status.
        if (overflow !== expected_overflow)
            $fatal(1,
                "OVERFLOW MISMATCH: numerator=%0d denominator=%0d, got=%b expected=%b",
                num, den, overflow, expected_overflow);

        // Check quotient when division by zero did not occur.
        if (!expected_div_by_zero &&
            quotient !== expected_quotient)
            $fatal(1,
                "QUOTIENT MISMATCH: numerator=%0d denominator=%0d, got=%0d expected=%0d",
                num, den, quotient, expected_quotient);

        $display(
            "PASS: num_raw=%0d den_raw=%0d quotient_raw=%0d div0=%b overflow=%b cycles=%0d",
            num, den, quotient, div_by_zero, overflow, cycles
        );
    end
endtask




task automatic test_reset_while_busy;
    begin
        $display("\n--- Reset while busy test ---");

        // Begin a division.
        @(negedge clk);
        numerator   = 16'sd16384;
        denominator = 32'sd268435456;
        start       = 1'b1;

        @(negedge clk);
        start = 1'b0;

        // Wait until the divider is actively processing.
        wait (busy === 1'b1);

        // Allow several iterations before asserting reset.
        repeat (5) @(negedge clk);

        rst_n = 1'b0;

        // Allow the synchronous reset to take effect.
        @(negedge clk);

        if (busy !== 1'b0)
            $fatal(1, "FAIL: busy did not clear after reset");

        if (done !== 1'b0)
            $fatal(1, "FAIL: done did not clear after reset");

        if (quotient !== 32'sd0)
            $fatal(1, "FAIL: quotient did not clear after reset");

        if (div_by_zero !== 1'b0 || overflow !== 1'b0)
            $fatal(1, "FAIL: status flags did not clear after reset");

        $display("PASS: reset aborts an active division");

        rst_n = 1'b1;
        repeat (2) @(negedge clk);
    end
endtask



task automatic test_start_while_busy;
    integer cycles;
    begin
        $display("\n--- Start while busy test ---");

        // Start the first division: 0.5 / 0.5 = 1.0.
        @(negedge clk);
        numerator   = 16'sd16384;
        denominator = 32'sd268435456;
        start       = 1'b1;

        @(negedge clk);
        start = 1'b0;

        wait (busy === 1'b1);

        // Present a second request while the first is running.
        @(negedge clk);
        numerator   = 16'sd8192;
        denominator = 32'sd536870912;
        start       = 1'b1;

        @(negedge clk);
        start = 1'b0;

        cycles = 0;
        while ((done !== 1'b1) && (cycles < DIV_W + 10)) begin
            @(negedge clk);
            cycles++;
        end

        if (done !== 1'b1)
            $fatal(1, "FAIL: first division timed out");

        if (quotient !== 32'sd134217728)
            $fatal(1, "FAIL: busy start corrupted first quotient: %0d",
                   quotient);

        if (overflow !== 1'b0 || div_by_zero !== 1'b0)
            $fatal(1, "FAIL: unexpected status on first division");

        $display("PASS: busy start does not corrupt active division");
    end
endtask



task automatic test_start_held_high;
    integer cycles;
    integer completed;
    begin
        $display("\n--- Start held high test ---");

        @(negedge clk);
        numerator   = 16'sd16384;
        denominator = 32'sd268435456;
        start       = 1'b1;

        completed = 0;
        cycles = 0;

        // Observe enough cycles for the first completion and possible
        // acceptance of a second request.
        while ((completed < 2) && (cycles < 2*(DIV_W + 5))) begin
            @(negedge clk);

            if (done === 1'b1)
                completed++;

            cycles++;
        end

        start = 1'b0;

        if (completed < 2)
            $fatal(1, "FAIL: held-high start did not initiate repeated work");

        $display("PASS: held-high start behavior observed");
    end
endtask



task automatic test_back_to_back_divisions;
    begin
        $display("\n--- Back-to-back division test ---");

        run_division(
            16'sd16384,
            32'sd268435456,
            32'sd134217728,
            1'b0,
            1'b0
        );

        run_division(
            -16'sd8192,
            32'sd268435456,
            -32'sd67108864,
            1'b0,
            1'b0
        );

        $display("PASS: consecutive divisions completed independently");
    end
endtask






// ------------------------------------------------------------
// Test sequence
// ------------------------------------------------------------
initial begin

    numerator   = '0;
    denominator = '0;

    // Apply reset.
    repeat (3) @(negedge clk);
    rst_n = 1'b1;

    // --------------------------------------------------------
    // Test 1: 0.5 / 0.5 = 1.0
    //
    // Numerator Q1.15:
    //   0.5 * 2^15 = 16384
    //
    // Denominator Q2.30:
    //   0.5 * 2^30 = 536870912
    //
    // Quotient Q5.27:
    //   1.0 * 2^27 = 134217728
    // --------------------------------------------------------
    run_division(
    16'sd16384,
    32'sd268435456,
    32'sd134217728,
    1'b0,
    1'b0
    );
    // --------------------------------------------------------
    // Test 2: 0.25 / 0.5 = 0.5
    // --------------------------------------------------------
   run_division(
    16'sd8192,
    32'sd536870912,
    32'sd33554432,
    1'b0,
    1'b0
    );
   
    // --------------------------------------------------------
    // Test 3: -0.25 / 0.5 = -0.5
    // Q1.15 / Q3.29 -> Q5.27
    // --------------------------------------------------------
    run_division(
        -16'sd8192,        // -0.25
         32'sd268435456,   // +0.5
        -32'sd67108864,     // -0.5
         1'b0,
         1'b0
    );

    // --------------------------------------------------------
    // Test 4: 0.25 / 1.0 = 0.25
    // --------------------------------------------------------
    run_division(
         16'sd8192,        // +0.25
         32'sd536870912,   // +1.0
         32'sd33554432,    // +0.25
         1'b0,
         1'b0
    );

    // --------------------------------------------------------
    // Test 5: 0.25 / -0.5 = -0.5
    // --------------------------------------------------------
    run_division(
         16'sd8192,        // +0.25
        -32'sd268435456,   // -0.5
        -32'sd67108864,    // -0.5
         1'b0,
         1'b0
    );

    // --------------------------------------------------------
    // Test 6: Divide by zero
    // --------------------------------------------------------
    run_division(
         16'sd8192,
         32'sd0,
         32'sd0,
         1'b1,
         1'b0
    );

    // --------------------------------------------------------
    // Test 7: Positive overflow
    // --------------------------------------------------------
    run_division(
         16'sd16384,       // +0.5
         32'sd1,           // Extremely small positive denominator
         32'sh7FFFFFFF,    // Maximum signed 32-bit value
         1'b0,
         1'b1
    );

    // --------------------------------------------------------
    // Test 8: Negative overflow
    // --------------------------------------------------------
    run_division(
        -16'sd16384,       // -0.5
         32'sd1,
         32'sh80000000,    // Minimum signed 32-bit value
         1'b0,
         1'b1
    );

// --------------------------------------------------------
// Test 9: Zero numerator / nonzero denominator
// Expected quotient = 0
// --------------------------------------------------------
run_division(
    16'sd0,
    32'sd536870912,
    32'sd0,
    1'b0,
    1'b0
);

// --------------------------------------------------------
// Test 10: Negative / negative = positive
// (-0.25) / (-0.5) = +0.5
// --------------------------------------------------------
run_division(
    -16'sd8192,
    -32'sd268435456,
    32'sd67108864,
    1'b0,
    1'b0
);

// --------------------------------------------------------
// Test 11: Minimum signed numerator
// (-1.0) / (+0.5) = -2.0
// --------------------------------------------------------
run_division(
    -16'sd32768,
    32'sd268435456,
    -32'sd268435456,
    1'b0,
    1'b0
);


// --------------------------------------------------------
// Test 12: Minimum signed denominator
// Numerator = +0.5
// Denominator = -4.0 in Q3.29
// Quotient = -0.125
// --------------------------------------------------------
run_division(
    16'sd16384,
    -32'sd2147483648,
    -32'sd16777216,
    1'b0,
    1'b0
);


// --------------------------------------------------------
// Test 13: Zero numerator / zero denominator
// --------------------------------------------------------
run_division(
    16'sd0,
    32'sd0,
    32'sd0,
    1'b1,
    1'b0
);


test_reset_while_busy();
test_start_while_busy();
test_start_held_high();
test_back_to_back_divisions();


    $display("");
    $display("ALL DIVIDER TESTS PASSED");
    $display("Configuration: DIV_W=%0d, SCALE_SHIFT=%0d",
             DIV_W, SCALE_SHIFT);

    $finish;
end


endmodule
