`timescale 1ns/1ps

// Board-independent shared arithmetic engine for the LMS / NLMS / Kalman DSP accelerator.
// Data operands and result use signed Q1.15 format.
// MAC accumulator stores products in signed Q*.30 format (40-bit internal accumulator).
// One command is accepted, then executed one clock later. Commands are not pipelined.
module arithmetic_engine #(
    parameter int DATA_W = 16,
    parameter int ACC_W  = 40
) (
    input  logic                         clk,
    input  logic                         rst_n,

    input  logic                         cmd_valid,
    output logic                         cmd_ready,
    input  logic [2:0]                   opcode,
    input  logic signed [DATA_W-1:0]     operand_a,
    input  logic signed [DATA_W-1:0]     operand_b,

  
    output logic signed [DATA_W-1:0]     result,
    output logic signed [ACC_W-1:0]      accumulator_value,
    output logic                         result_valid,
    output logic                         busy,
    output logic                         overflow


);

    localparam logic [2:0] OP_MUL       = 3'b000;
    localparam logic [2:0] OP_MAC       = 3'b001;
    localparam logic [2:0] OP_ADD       = 3'b010;
    localparam logic [2:0] OP_SUB       = 3'b011;
    localparam logic [2:0] OP_ACC_CLEAR = 3'b100;
    localparam logic [2:0] OP_ACC_READ  = 3'b101;

    logic [2:0]               opcode_q;
    logic signed [DATA_W-1:0] operand_a_q, operand_b_q;
    logic signed [ACC_W-1:0]  accumulator_q;
    assign accumulator_value = accumulator_q;

    // Q1.15 x Q1.15 -> Q2.30. This implementation is parameterized for DATA_W,
    // but the product/accumulator sizing below is intentionally designed for DATA_W=16.
    logic signed [(2*DATA_W)-1:0] product;
    logic signed [ACC_W:0]         mac_sum_ext;
    logic signed [DATA_W:0]        add_ext, sub_ext;

    assign cmd_ready = ~busy;
    assign product = operand_a_q * operand_b_q;

    // Extend product and accumulator to ACC_W+1 bits before addition so signed
    // accumulator overflow can be detected rather than silently wrapping.
    assign mac_sum_ext = $signed({accumulator_q[ACC_W-1], accumulator_q}) +
                         $signed({{(ACC_W+1-(2*DATA_W)){product[(2*DATA_W)-1]}}, product});

    assign add_ext = $signed({operand_a_q[DATA_W-1], operand_a_q}) +
                     $signed({operand_b_q[DATA_W-1], operand_b_q});
    assign sub_ext = $signed({operand_a_q[DATA_W-1], operand_a_q}) -
                     $signed({operand_b_q[DATA_W-1], operand_b_q});

    // Convert a signed Q30 value to Q1.15 using round-to-nearest, ties away from zero,
    // then saturate to the signed 16-bit range. ACC_W=40 and DATA_W=16 are the intended
    // project configuration; the slice/scale convention is Q30 -> Q15 (shift by 15).


function automatic logic signed [DATA_W-1:0] q30_to_q15_sat(
    input logic signed [ACC_W-1:0] value
);
    logic signed [ACC_W:0] rounded;
    logic signed [ACC_W:0] scaled;

    begin
        // Round to nearest, with exact half-way cases rounded away from zero.
        if (value[ACC_W-1])
            rounded = $signed({value[ACC_W-1], value})
                      + ((1 <<< 14) - 1);
        else
            rounded = $signed({value[ACC_W-1], value})
                      + (1 <<< 14);

        scaled = rounded >>> 15;

        if (scaled > 32767)
            q30_to_q15_sat = {1'b0, {(DATA_W-1){1'b1}}};
        else if (scaled < -32768)
            q30_to_q15_sat = {1'b1, {(DATA_W-1){1'b0}}};
        else
            q30_to_q15_sat = scaled[DATA_W-1:0];
    end
endfunction




function automatic logic q30_to_q15_overflow(
    input logic signed [ACC_W-1:0] value
);
    logic signed [ACC_W:0] rounded;
    logic signed [ACC_W:0] scaled;

    begin
        // Round to nearest, with exact half-way cases rounded away from zero.
        if (value[ACC_W-1])
            rounded = $signed({value[ACC_W-1], value})
                      + ((1 <<< 14) - 1);
        else
            rounded = $signed({value[ACC_W-1], value})
                      + (1 <<< 14);

        scaled = rounded >>> 15;

        q30_to_q15_overflow =
            (scaled > 32767) || (scaled < -32768);
    end
endfunction




    function automatic logic signed [DATA_W-1:0] q15_saturate(
        input logic signed [DATA_W:0] value
    );
        begin
            if (value > 32767)
                q15_saturate = {1'b0, {(DATA_W-1){1'b1}}};
            else if (value < -32768)
                q15_saturate = {1'b1, {(DATA_W-1){1'b0}}};
            else
                q15_saturate = value[DATA_W-1:0];
        end
    endfunction

    function automatic logic q15_overflow(input logic signed [DATA_W:0] value);
        begin
            q15_overflow = (value > 32767) || (value < -32768);
        end
    endfunction


    

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            opcode_q     <= OP_ACC_CLEAR;
            operand_a_q  <= '0;
            operand_b_q  <= '0;
            accumulator_q <= '0;
            result       <= '0;
            result_valid <= 1'b0;
            busy         <= 1'b0;
            overflow     <= 1'b0;
        end else begin
            // result_valid and overflow describe the completion pulse for the current command.
            result_valid <= 1'b0;
            overflow     <= 1'b0;

            if (!busy) begin
                if (cmd_valid && cmd_ready) begin
                    opcode_q    <= opcode;
                    operand_a_q <= operand_a;
                    operand_b_q <= operand_b;
                    busy        <= 1'b1;
                end
            end else begin
                busy         <= 1'b0;
                result_valid <= 1'b1;

                unique case (opcode_q)
                    OP_MUL: begin
                        result   <= q30_to_q15_sat({{(ACC_W-(2*DATA_W)){product[(2*DATA_W)-1]}}, product});
                        overflow <= q30_to_q15_overflow({{(ACC_W-(2*DATA_W)){product[(2*DATA_W)-1]}}, product});
                    end

                    OP_MAC: begin
                        if (mac_sum_ext[ACC_W] != mac_sum_ext[ACC_W-1]) begin
                            // Saturate the internal accumulator if the 40-bit sum overflows.
                            if (mac_sum_ext[ACC_W]) begin
                                accumulator_q <= {1'b1, {(ACC_W-1){1'b0}}};
                                result        <= {1'b1, {(DATA_W-1){1'b0}}};
                            end else begin
                                accumulator_q <= {1'b0, {(ACC_W-1){1'b1}}};
                                result        <= {1'b0, {(DATA_W-1){1'b1}}};
                            end
                            overflow <= 1'b1;
                        end else begin
                            accumulator_q <= mac_sum_ext[ACC_W-1:0];
                            result        <= q30_to_q15_sat(mac_sum_ext[ACC_W-1:0]);
                            overflow      <= q30_to_q15_overflow(mac_sum_ext[ACC_W-1:0]);
                        end
                    end

                    OP_ADD: begin
                        result   <= q15_saturate(add_ext);
                        overflow <= q15_overflow(add_ext);
                    end

                    OP_SUB: begin
                        result   <= q15_saturate(sub_ext);
                        overflow <= q15_overflow(sub_ext);
                    end

                    OP_ACC_CLEAR: begin
                        accumulator_q <= '0;
                        result        <= '0;
                        overflow      <= 1'b0;
                    end

                    OP_ACC_READ: begin
                        result   <= q30_to_q15_sat(accumulator_q);
                        overflow <= q30_to_q15_overflow(accumulator_q);
                    end

                    default: begin
                        result   <= '0;
                        overflow <= 1'b1; // Invalid opcode is reported as an error/overflow pulse.
                    end
                endcase
            end
        end
    end

endmodule
