`timescale 1ns/1ps

module arithmetic_engine_tb;
    localparam logic [2:0] OP_MUL       = 3'b000;
    localparam logic [2:0] OP_MAC       = 3'b001;
    localparam logic [2:0] OP_ADD       = 3'b010;
    localparam logic [2:0] OP_SUB       = 3'b011;
    localparam logic [2:0] OP_ACC_CLEAR = 3'b100;
    localparam logic [2:0] OP_ACC_READ  = 3'b101;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic cmd_valid;
    logic cmd_ready;
    logic [2:0] opcode;
    logic signed [15:0] operand_a, operand_b;
    logic signed [15:0] result;
    logic result_valid, busy, overflow;

    int checks = 0;
    int failures = 0;

    always #5 clk = ~clk;

    arithmetic_engine dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .opcode(opcode),
        .operand_a(operand_a), .operand_b(operand_b),
        .result(result), .result_valid(result_valid), .busy(busy), .overflow(overflow)
    );

    // Drives commands away from the active edge and samples the registered completion.
    task automatic issue_and_check(
        input logic [2:0] op,
        input logic signed [15:0] a,
        input logic signed [15:0] b,
        input logic signed [15:0] expected_result,
        input logic expected_overflow,
        input string label_text
    );
        begin
            @(negedge clk);
            while (!cmd_ready) @(negedge clk);
            opcode = op;
            operand_a = a;
            operand_b = b;
            cmd_valid = 1'b1;
            @(posedge clk); // command handshake
            @(negedge clk);
            cmd_valid = 1'b0;

            // Completion occurs on the next active edge; sample at the following falling edge.
            while (!result_valid) @(negedge clk);
            checks++;
            if ((result !== expected_result) || (overflow !== expected_overflow)) begin
                failures++;
                $error("FAIL %-24s result=%0d (0x%04h), overflow=%b; expected=%0d (0x%04h), overflow=%b",
                       label_text, result, result, overflow,
                       expected_result, expected_result, expected_overflow);
            end else begin
                $display("PASS %-24s result=%0d (0x%04h), overflow=%b",
                         label_text, result, result, overflow);
            end
        end
    endtask




// Verify the internal MAC accumulator after a command completes.
task automatic check_accumulator(
    input logic signed [39:0] expected_value,
    input string label_text
);
    begin
        if ($signed(dut.accumulator_q) !== expected_value) begin
            $error(
                "FAIL %-20s accumulator=%0d expected=%0d",
                label_text,
                $signed(dut.accumulator_q),
                expected_value
            );
            failures++;
        end
        else begin
            $display(
                "PASS %-20s accumulator=%0d",
                label_text,
                $signed(dut.accumulator_q)
            );
        end
    end
endtask


task automatic check_accumulator_output(
    input logic signed [39:0] expected_value,
    input string label_text
);
    begin
        if ($signed(dut.accumulator_value) !== expected_value) begin
            $error(
                "FAIL %-30s output=%0d expected=%0d",
                label_text,
                $signed(dut.accumulator_value),
                expected_value
            );
            failures++;
        end
        else begin
            $display(
                "PASS %-30s output=%0d",
                label_text,
                $signed(dut.accumulator_value)
            );
        end
    end
endtask



    initial begin
        cmd_valid = 1'b0;
        opcode = '0;
        operand_a = '0;
        operand_b = '0;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        // Q1.15: 0.5 * 0.5 = 0.25; 0.25 is 8192 in Q1.15.
        issue_and_check(OP_MUL, 16'sd16384, 16'sd16384, 16'sd8192, 1'b0, "positive multiply");
        issue_and_check(OP_MUL, -16'sd16384, 16'sd16384, -16'sd8192, 1'b0, "negative multiply");
        issue_and_check(OP_MUL, 16'sd1, 16'sd16384, 16'sd1, 1'b0, "positive half-LSB round");
        issue_and_check(OP_MUL, -16'sd1, 16'sd16384, -16'sd1, 1'b0, "negative half-LSB round");
        issue_and_check(OP_MUL, -16'sd32768, -16'sd32768, 16'sd32767, 1'b1, "multiply saturation");

        issue_and_check(OP_ADD, 16'sd1000, 16'sd2000, 16'sd3000, 1'b0, "addition");
        issue_and_check(OP_ADD, 16'sd20000, 16'sd20000, 16'sd32767, 1'b1, "addition saturation");
        issue_and_check(OP_SUB, -16'sd1000, 16'sd2000, -16'sd3000, 1'b0, "subtraction");
        issue_and_check(OP_SUB, -16'sd20000, 16'sd20000, -16'sd32768, 1'b1, "subtraction saturation");

        issue_and_check(OP_ACC_CLEAR, 16'sd0, 16'sd0, 16'sd0, 1'b0, "clear accumulator");
        issue_and_check(OP_MAC, 16'sd16384, 16'sd16384, 16'sd8192, 1'b0, "first MAC");
        issue_and_check(OP_MAC, 16'sd16384, 16'sd16384, 16'sd16384, 1'b0, "second MAC");
        issue_and_check(OP_ACC_READ, 16'sd0, 16'sd0, 16'sd16384, 1'b0, "read accumulator");

        


// --------------------------------------------------------
// ACCUMULATOR VERIFICATION
// Q1.15 * Q1.15 = Q2.30
// --------------------------------------------------------

// Clear any previous accumulator value.
issue_and_check(
    OP_ACC_CLEAR,
    16'sd0,
    16'sd0,
    16'sd0,
    1'b0,
    "energy test clear"
);

// First MAC: 0.5 * 0.5 = 0.25
issue_and_check(
    OP_MAC,
    16'sd16384,
    16'sd16384,
    16'sd8192,
    1'b0,
    "energy MAC 1"
);
check_accumulator(40'sd268435456, "after MAC 1");

// Second MAC: total = 0.50
issue_and_check(
    OP_MAC,
    16'sd16384,
    16'sd16384,
    16'sd16384,
    1'b0,
    "energy MAC 2"
);
check_accumulator(40'sd536870912, "after MAC 2");

// Third MAC: total = 0.75
issue_and_check(
    OP_MAC,
    16'sd16384,
    16'sd16384,
    16'sd24576,
    1'b0,
    "energy MAC 3"
);
check_accumulator(40'sd805306368, "after MAC 3");

// Fourth MAC: total = 1.00
issue_and_check(
    OP_MAC,
    16'sd16384,
    16'sd16384,
    16'sd32767,
    1'b1,
    "energy MAC 4"
);
check_accumulator(40'sd1073741824, "after MAC 4");
check_accumulator_output(
    40'sd1073741824,
    "external energy after MAC 4"
);



issue_and_check(
    OP_MAC, 16'sd16384, 16'sd16384,
    16'sd32767, 1'b1, "energy MAC 5"
);

issue_and_check(
    OP_MAC, 16'sd16384, 16'sd16384,
    16'sd32767, 1'b1, "energy MAC 6"
);

issue_and_check(
    OP_MAC, 16'sd16384, 16'sd16384,
    16'sd32767, 1'b1, "energy MAC 7"
);

issue_and_check(
    OP_MAC, 16'sd16384, 16'sd16384,
    16'sd32767, 1'b1, "energy MAC 8"
);

check_accumulator_output(
    40'sd2147483648,
    "external energy equals 2.0"
);



// ------------------------------------------------------------
// Q30 TO Q29 ENERGY REPRESENTATION CHECK
// ------------------------------------------------------------

if (($signed(dut.accumulator_value) >>> 1) !== 40'sd1073741824) begin
    $error(
        "FAIL Q30-to-Q29 conversion: actual=%0d expected=%0d",
        $signed(dut.accumulator_value) >>> 1,
        1073741824
    );
    failures++;
end
else begin
    $display(
        "PASS Q30-to-Q29 conversion: energy=2.0, raw_Q29=%0d",
        $signed(dut.accumulator_value) >>> 1
    );
end



// ------------------------------------------------------------
// POSITIVE INTERNAL ACCUMULATOR OVERFLOW TEST
// ------------------------------------------------------------

$display("\n--- Positive accumulator overflow test ---");

// Place accumulator one step below its positive saturation limit.
dut.accumulator_q = 40'sd549755813886;

// Adding (-1.0) * (-1.0) adds 2^30,
// which exceeds the positive 40-bit accumulator limit.
issue_and_check(
    OP_MAC,
    -16'sd32768,
    -16'sd32768,
    16'sd32767,
    1'b1,
    "positive acc overflow"
);

// Verify that the accumulator saturated at its maximum.
if ($signed(dut.accumulator_q) !== 40'sd549755813887) begin
    $error(
        "FAIL positive accumulator saturation: actual=%0d",
        $signed(dut.accumulator_q)
    );
    failures++;
end
else begin
    $display(
        "PASS positive accumulator saturation: accumulator=%0d",
        $signed(dut.accumulator_q)
    );
end


// ------------------------------------------------------------
// NEGATIVE INTERNAL ACCUMULATOR OVERFLOW TEST
// ------------------------------------------------------------

$display("\n--- Negative accumulator overflow test ---");

// Place accumulator one raw unit above its negative limit.
dut.accumulator_q = -40'sd549755813887;

// (+1.0) * (-1.0) produces a negative Q30 product.
issue_and_check(
    OP_MAC,
    16'sd32767,
    -16'sd32768,
    -16'sd32768,
    1'b1,
    "negative acc overflow"
);

// Verify saturation at the minimum signed 40-bit value.
if ($signed(dut.accumulator_q) !== -40'sd549755813888) begin
    $error(
        "FAIL negative accumulator saturation: actual=%0d",
        $signed(dut.accumulator_q)
    );
    failures++;
end
else begin
    $display(
        "PASS negative accumulator saturation: accumulator=%0d",
        $signed(dut.accumulator_q)
    );
end






        if (failures == 0)
            $display("\nALL TESTS PASSED: %0d checks", checks);
        else
            $fatal(1, "TESTBENCH FAILED: %0d failures out of %0d checks", failures, checks);
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "TIMEOUT: testbench did not finish");
    end
endmodule
