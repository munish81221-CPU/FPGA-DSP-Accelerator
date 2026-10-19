`timescale 1ns/1ps

module lms_controller (
    input  logic clk,
    input  logic rst_n,              
    input  logic start,              

    input  logic signed [15:0] x0,
    input  logic signed [15:0] x1,
    input  logic signed [15:0] x2,
    input  logic signed [15:0] x3,
    input  logic signed [15:0] desired,
    input  logic signed [15:0] mu,

    output logic done,               
    output logic busy,
    output logic signed [15:0] y_out,
    output logic signed [15:0] error_out,
    output logic signed [15:0] w0,
    output logic signed [15:0] w1,
    output logic signed [15:0] w2,
    output logic signed [15:0] w3,
    output logic overflow_seen,      

    // Command channel to the shared arithmetic engine
    output logic cmd_valid,
    input  logic cmd_ready,
    output logic [2:0] opcode,
    output logic signed [15:0] operand_a,
    output logic signed [15:0] operand_b,

    // Response channel from the shared arithmetic engine
    input  logic result_valid,
    input  logic signed [15:0] result,
    input  logic engine_overflow
);

    localparam logic [2:0] OP_MUL       = 3'b000;
    localparam logic [2:0] OP_MAC       = 3'b001;
    localparam logic [2:0] OP_ADD       = 3'b010;
    localparam logic [2:0] OP_SUB       = 3'b011;
    localparam logic [2:0] OP_ACC_CLEAR = 3'b100;
    localparam logic [2:0] OP_ACC_READ  = 3'b101;

    typedef enum logic [4:0] {
        ST_IDLE,
        ST_ISSUE_CLEAR, ST_WAIT_CLEAR,
        ST_ISSUE_MAC,   ST_WAIT_MAC,
        ST_ISSUE_READ,  ST_WAIT_READ,
        ST_ISSUE_ERROR, ST_WAIT_ERROR,
        ST_ISSUE_MUE,   ST_WAIT_MUE,
        ST_ISSUE_CORR,  ST_WAIT_CORR,
        ST_ISSUE_WADD,  ST_WAIT_WADD
    } state_t;

    state_t state;

    logic signed [15:0] x_q [0:3];
    logic signed [15:0] desired_q;
    logic signed [15:0] mu_q;
    logic signed [15:0] error_q;
    logic signed [15:0] mu_error_q;
    logic signed [15:0] correction_q;
    logic signed [15:0] weights [0:3];
    logic [1:0] tap_idx;

    integer i;

    assign w0 = weights[0];
    assign w1 = weights[1];
    assign w2 = weights[2];
    assign w3 = weights[3];


    always_comb begin
        cmd_valid = 1'b0;
        opcode    = OP_ACC_CLEAR;
        operand_a = '0;
        operand_b = '0;

        case (state)
            ST_ISSUE_CLEAR: begin
                cmd_valid = 1'b1;
                opcode    = OP_ACC_CLEAR;
            end

            ST_ISSUE_MAC: begin
                cmd_valid = 1'b1;
                opcode    = OP_MAC;
                operand_a = weights[tap_idx];
                operand_b = x_q[tap_idx];
            end

            ST_ISSUE_READ: begin
                cmd_valid = 1'b1;
                opcode    = OP_ACC_READ;
            end

            ST_ISSUE_ERROR: begin
                cmd_valid = 1'b1;
                opcode    = OP_SUB;
                operand_a = desired_q;
                operand_b = y_out;
            end

            ST_ISSUE_MUE: begin
                cmd_valid = 1'b1;
                opcode    = OP_MUL;
                operand_a = mu_q;
                operand_b = error_q;
            end

            ST_ISSUE_CORR: begin
                cmd_valid = 1'b1;
                opcode    = OP_MUL;
                operand_a = mu_error_q;
                operand_b = x_q[tap_idx];
            end

            ST_ISSUE_WADD: begin
                cmd_valid = 1'b1;
                opcode    = OP_ADD;
                operand_a = weights[tap_idx];
                operand_b = correction_q;
            end

            default: begin

            end
        endcase
    end

    // Sequential control and data capture.
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            busy            <= 1'b0;
            done            <= 1'b0;
            y_out           <= '0;
            error_out       <= '0;
            desired_q       <= '0;
            mu_q            <= '0;
            error_q         <= '0;
            mu_error_q      <= '0;
            correction_q    <= '0;
            tap_idx         <= '0;
            overflow_seen   <= 1'b0;

            for (i = 0; i < 4; i = i + 1) begin
                x_q[i]   <= '0;
                weights[i] <= '0;
            end
        end 
        else begin
           
            done <= 1'b0;

            
            if (result_valid && engine_overflow)
                overflow_seen <= 1'b1;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        x_q[0] <= x0;
                        x_q[1] <= x1;
                        x_q[2] <= x2;
                        x_q[3] <= x3;
                        desired_q <= desired;
                        mu_q <= mu;

                        y_out         <= '0;
                        error_out     <= '0;
                        error_q       <= '0;
                        mu_error_q    <= '0;
                        correction_q  <= '0;
                        tap_idx       <= 2'd0;
                        overflow_seen <= 1'b0;
                        busy          <= 1'b1;
                        state         <= ST_ISSUE_CLEAR;
                    end
                end

                ST_ISSUE_CLEAR: if (cmd_ready) state <= ST_WAIT_CLEAR;
                ST_WAIT_CLEAR: if (result_valid) begin
                    tap_idx <= 2'd0;
                    state <= ST_ISSUE_MAC;
                end

                ST_ISSUE_MAC: if (cmd_ready) state <= ST_WAIT_MAC;
                ST_WAIT_MAC: if (result_valid) begin
                    if (tap_idx == 2'd3) begin
                        state <= ST_ISSUE_READ;
                    end else begin
                        tap_idx <= tap_idx + 2'd1;
                        state <= ST_ISSUE_MAC;
                    end
                end

                ST_ISSUE_READ: if (cmd_ready) state <= ST_WAIT_READ;
                ST_WAIT_READ: if (result_valid) begin
                    y_out <= result;
                    state <= ST_ISSUE_ERROR;
                end

                ST_ISSUE_ERROR: if (cmd_ready) state <= ST_WAIT_ERROR;
                ST_WAIT_ERROR: if (result_valid) begin
                    error_q   <= result;
                    error_out <= result;
                    state     <= ST_ISSUE_MUE;
                end

                ST_ISSUE_MUE: if (cmd_ready) state <= ST_WAIT_MUE;
                ST_WAIT_MUE: if (result_valid) begin
                    mu_error_q <= result;
                    tap_idx    <= 2'd0;
                    state      <= ST_ISSUE_CORR;
                end

                ST_ISSUE_CORR: if (cmd_ready) state <= ST_WAIT_CORR;
                ST_WAIT_CORR: if (result_valid) begin
                    correction_q <= result;
                    state <= ST_ISSUE_WADD;
                end

                ST_ISSUE_WADD: if (cmd_ready) state <= ST_WAIT_WADD;
                ST_WAIT_WADD: if (result_valid) begin
                    

                    weights[tap_idx] <= result;

                    if (tap_idx == 2'd3) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        tap_idx <= tap_idx + 2'd1;
                        state   <= ST_ISSUE_CORR;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    busy  <= 1'b0;
                end
            endcase
        end
    end

endmodule
