`timescale 1ns/1ps

// Boundary-focused self-checking testbench for arithmetic_engine.
// Covers Q1.15 ADD/SUB saturation, MUL rounding/saturation, accumulator
// readback, MAC accumulation, reset, and command handshake behavior.
//
// Intended configuration: DATA_W=16, ACC_W=40.
module arithmetic_engine_boundary_tb;

    localparam logic [2:0] OP_MUL       = 3'b000;
    localparam logic [2:0] OP_MAC       = 3'b001;
    localparam logic [2:0] OP_ADD       = 3'b010;
    localparam logic [2:0] OP_SUB       = 3'b011;
    localparam logic [2:0] OP_ACC_CLEAR = 3'b100;
    localparam logic [2:0] OP_ACC_READ  = 3'b101;

    logic clk;
    logic rst_n;
    logic cmd_valid;
    logic cmd_ready;
    logic [2:0] opcode;
    logic signed [15:0] operand_a;
    logic signed [15:0] operand_b;

    logic signed [15:0] result;
    logic signed [39:0] accumulator_value;
    logic result_valid;
    logic busy;
    logic overflow;

    integer checks;
    integer failures;

    arithmetic_engine #(
        .DATA_W(16),
        .ACC_W(40)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),
        .opcode(opcode),
        .operand_a(operand_a),
        .operand_b(operand_b),
        .result(result),
        .accumulator_value(accumulator_value),
        .result_valid(result_valid),
        .busy(busy),
        .overflow(overflow)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // A watchdog prevents a broken handshake from hanging simulation.
    initial begin
        #200000;
        $display("FAIL: global watchdog expired");
        $fatal(1);
    end

    task automatic check_result(
        input string label,
        input logic signed [15:0] expected_result,
        input logic expected_overflow
    );
        begin
            checks = checks + 1;
            if ((result !== expected_result) || (overflow !== expected_overflow)) begin
                failures = failures + 1;
                $error("%s: expected result=%0d (0x%04h), overflow=%b; got result=%0d (0x%04h), overflow=%b",
                       label, expected_result, expected_result,
                       expected_overflow, result, result, overflow);
                $fatal(1, "Boundary check failed");
            end else begin
                $display("PASS %-34s result=%0d (0x%04h) overflow=%b",
                         label, result, result, overflow);
            end
        end
    endtask

    task automatic check_accumulator(
        input string label,
        input logic signed [39:0] expected_accumulator
    );
        begin
            checks = checks + 1;
            if (accumulator_value !== expected_accumulator) begin
                failures = failures + 1;
                $error("%s: expected accumulator=%0d (0x%010h), got %0d (0x%010h)",
                       label, expected_accumulator, expected_accumulator,
                       accumulator_value, accumulator_value);
                $fatal(1, "Accumulator check failed");
            end else begin
                $display("PASS %-34s accumulator=%0d (0x%010h)",
                         label, accumulator_value, accumulator_value);
            end
        end
    endtask

    task automatic issue_command(
        input logic [2:0] op_value,
        input logic signed [15:0] a_value,
        input logic signed [15:0] b_value
    );
        integer wait_cycles;
        begin
            // Drive inputs away from the active clock edge.
            @(negedge clk);
            wait_cycles = 0;
            while (cmd_ready !== 1'b1) begin
                @(negedge clk);
                wait_cycles = wait_cycles + 1;
                if (wait_cycles > 10)
                    $fatal(1, "Timeout waiting for cmd_ready");
            end

            opcode    = op_value;
            operand_a = a_value;
            operand_b = b_value;
            cmd_valid = 1'b1;

            // The engine samples the command on the next rising edge.
            @(negedge clk);
            cmd_valid = 1'b0;

            wait_cycles = 0;
            while (result_valid !== 1'b1) begin
                @(negedge clk);
                wait_cycles = wait_cycles + 1;
                if (wait_cycles > 10)
                    $fatal(1, "Timeout waiting for result_valid, opcode=%b", op_value);
            end
            // Completion outputs are registered on the preceding posedge and
            // remain stable through this negedge.
            #1;
        end
    endtask

    task automatic reset_engine;
        begin
            @(negedge clk);
            rst_n = 1'b0;
            cmd_valid = 1'b0;
            opcode = OP_ACC_CLEAR;
            operand_a = '0;
            operand_b = '0;
            repeat (2) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        cmd_valid = 1'b0;
        opcode = OP_ACC_CLEAR;
        operand_a = '0;
        operand_b = '0;
        checks = 0;
        failures = 0;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        $display("\n=== ADD saturation boundaries ===");
        issue_command(OP_ADD, 16'sd32767, 16'sd1);
        check_result("ADD positive overflow", 16'sd32767, 1'b1);

        issue_command(OP_ADD, -16'sd32768, -16'sd1);
        check_result("ADD negative overflow", -16'sd32768, 1'b1);

        issue_command(OP_ADD, 16'sd32766, 16'sd1);
        check_result("ADD positive exact limit", 16'sd32767, 1'b0);

        issue_command(OP_ADD, -16'sd32767, -16'sd1);
        check_result("ADD negative exact limit", -16'sd32768, 1'b0);

        issue_command(OP_ADD, -16'sd1, 16'sd1);
        check_result("ADD cancellation to zero", 16'sd0, 1'b0);

        $display("\n=== SUB saturation boundaries ===");
        issue_command(OP_SUB, -16'sd32768, 16'sd1);
        check_result("SUB negative overflow", -16'sd32768, 1'b1);

        issue_command(OP_SUB, 16'sd32767, -16'sd1);
        check_result("SUB positive overflow", 16'sd32767, 1'b1);

        issue_command(OP_SUB, 16'sd32766, -16'sd1);
        check_result("SUB positive exact limit", 16'sd32767, 1'b0);

        issue_command(OP_SUB, -16'sd32767, 16'sd1);
        check_result("SUB negative exact limit", -16'sd32768, 1'b0);

        issue_command(OP_SUB, -16'sd1234, -16'sd1234);
        check_result("SUB equal operands", 16'sd0, 1'b0);

        $display("\n=== MUL rounding and saturation ===");
        // Product is signed Q30. The product is converted to Q1.15 by the DUT.
        // These operands target the half-LSB and near-limit neighborhoods.
        issue_command(OP_MUL, 16'sd1, 16'sd16383);
        check_result("MUL below positive half-LSB", 16'sd0, 1'b0);

        issue_command(OP_MUL, 16'sd1, 16'sd16384);
        check_result("MUL positive half-LSB", 16'sd1, 1'b0);

        issue_command(OP_MUL, 16'sd1, 16'sd16385);
        check_result("MUL above positive half-LSB", 16'sd1, 1'b0);

        issue_command(OP_MUL, -16'sd1, 16'sd16383);
        check_result("MUL above negative half-LSB", 16'sd0, 1'b0);

        issue_command(OP_MUL, -16'sd1, 16'sd16384);
        check_result("MUL negative half-LSB", -16'sd1, 1'b0);

        issue_command(OP_MUL, -16'sd1, 16'sd16385);
        check_result("MUL below negative half-LSB", -16'sd1, 1'b0);

        issue_command(OP_MUL, 16'sh8000, 16'sh8000);
        check_result("MUL min times min saturation", 16'sd32767, 1'b1);

        issue_command(OP_MUL, 16'sh7FFF, 16'sh7FFF);
        check_result("MUL max times max", 16'sd32766, 1'b0);

        issue_command(OP_MUL, 16'sh8000, 16'sh7FFF);
        check_result("MUL negative extreme product", -16'sd32767, 1'b0);

        $display("\n=== MAC accumulation and readback ===");
        issue_command(OP_ACC_CLEAR, 16'sd0, 16'sd0);
        check_result("ACC_CLEAR result", 16'sd0, 1'b0);
        check_accumulator("ACC_CLEAR accumulator", 40'sd0);

        issue_command(OP_MAC, 16'sh7FFF, 16'sh7FFF);
        check_result("MAC one positive product", 16'sd32766, 1'b0);
        check_accumulator("MAC raw Q30 accumulator", 40'sd1073676289);

        issue_command(OP_ACC_READ, 16'sd0, 16'sd0);
        check_result("ACC_READ converted result", 16'sd32766, 1'b0);
        check_accumulator("ACC_READ preserves accumulator", 40'sd1073676289);

        // Accumulator may exceed Q1.15 output range without overflowing 40 bits.
        issue_command(OP_MAC, 16'sh8000, 16'sh8000);
        check_result("MAC output saturates only", 16'sd32767, 1'b1);
        check_accumulator("MAC retains wide accumulated sum", 40'sd2147418113);

        $display("\n=== Reset recovery ===");
        reset_engine();
        check_accumulator("Reset clears accumulator", 40'sd0);
        issue_command(OP_ADD, 16'sd123, -16'sd23);
        check_result("ADD after reset", 16'sd100, 1'b0);

        $display("\nALL ARITHMETIC ENGINE BOUNDARY CHECKS PASSED: %0d checks", checks);
        $finish;
    end

endmodule
