`timescale 1ns/1ps

module divider_unit_tb;
    localparam int NUM_W = 16;
    localparam int DEN_W = 16;
    localparam int QUOT_W = 24;
    localparam int FRACTION_BITS = 8;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic start = 1'b0;
    logic signed [NUM_W-1:0] numerator;
    logic signed [DEN_W-1:0] denominator;
    logic busy, done, div_by_zero, overflow;
    logic signed [QUOT_W-1:0] quotient;

    divider_unit #(
        .NUM_W(NUM_W),
        .DEN_W(DEN_W),
        .QUOT_W(QUOT_W),
        .FRACTION_BITS(FRACTION_BITS)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .numerator(numerator), .denominator(denominator),
        .busy(busy), .done(done), .quotient(quotient),
        .div_by_zero(div_by_zero), .overflow(overflow)
    );

    always #5 clk = ~clk;

    task automatic run_division(
        input logic signed [NUM_W-1:0] num,
        input logic signed [DEN_W-1:0] den,
        input logic signed [QUOT_W-1:0] expected,
        input logic expected_zero_error
    );
        integer cycles;
        begin
            @(negedge clk);
            numerator = num;
            denominator = den;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            cycles = 0;
            while ((done !== 1'b1) && (cycles < 100)) begin
                @(negedge clk);
                cycles = cycles + 1;
            end

            if (done !== 1'b1)
                $fatal(1, "TIMEOUT for %0d / %0d", num, den);
            if (div_by_zero !== expected_zero_error)
                $fatal(1, "divide-by-zero flag mismatch for %0d / %0d", num, den);
            if (!expected_zero_error && quotient !== expected)
                $fatal(1, "quotient mismatch: %0d / %0d -> %0d, expected %0d",
                       num, den, quotient, expected);
            if (!expected_zero_error && overflow !== 1'b0)
                $fatal(1, "unexpected overflow for %0d / %0d", num, den);

            $display("PASS: %0d / %0d -> quotient raw=%0d (fraction bits=%0d), div0=%0b",
                     num, den, quotient, FRACTION_BITS, div_by_zero);
        end
    endtask

    initial begin
        numerator = '0;
        denominator = '0;
        repeat (2) @(negedge clk);
        rst_n = 1'b1;

        // quotient_raw = (numerator / denominator) * 2**8, truncated toward zero.
        run_division(16'sd13, 16'sd3, 24'sd1109, 1'b0);
        run_division(-16'sd13, 16'sd3, -24'sd1109, 1'b0);
        run_division(16'sd12, 16'sd4, 24'sd768, 1'b0);
        run_division(16'sd1, 16'sd2, 24'sd128, 1'b0);
        run_division(16'sd5, 16'sd0, 24'sd0, 1'b1);

        $display("ALL DIVIDER TESTS PASSED");
        $finish;
    end
endmodule
