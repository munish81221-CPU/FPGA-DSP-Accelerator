// Sequential signed fixed-point restoring divider.
// Computes quotient_raw = (numerator_raw * 2**SCALE_SHIFT) / denominator_raw.




module divider_unit #(
    parameter int NUM_W = 16,
    parameter int DEN_W = 32,
    parameter int QUOT_W = 32,
    parameter int SCALE_SHIFT = 41
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     start,
    input  logic signed [NUM_W-1:0]  numerator,
    input  logic signed [DEN_W-1:0]  denominator,
    output logic                     busy,
    output logic                     done,
    output logic signed [QUOT_W-1:0] quotient,
    output logic                     div_by_zero,
    output logic                     overflow
);

    localparam int DIV_W = NUM_W + SCALE_SHIFT;   
    localparam int COUNT_W = $clog2(DIV_W + 1);

    typedef enum logic [1:0] {DIV_IDLE, DIV_RUN, DIV_FINISH} state_t;
    state_t state;

    logic [DIV_W-1:0] dividend_shift;
    logic [DIV_W-1:0] quotient_work;
    logic [DIV_W:0] remainder_work;
    logic [DIV_W:0] divisor_abs;
    logic [DIV_W:0] trial_remainder;
    logic [COUNT_W-1:0] bit_count;
    logic quotient_negative;

    logic [DIV_W-1:0] quotient_magnitude;
    logic [DIV_W-1:0] max_positive;
    logic [DIV_W-1:0] max_negative_magnitude;


//below two functions are used to get absolute values of numerator and denominator it works as---
//When the original number is nonnegative, the function takes its lower NUM_W bits directly. 
//When the original number is negative, it first negates the extended value to calculate the positive magnitude.


    function automatic logic [NUM_W-1:0] abs_num(  //abs->absolute value 
        input logic signed [NUM_W-1:0] value
    );
        logic signed [NUM_W:0] extended;
        begin
            extended = {value[NUM_W-1], value};
            if (value[NUM_W-1])
                abs_num = -extended;
            else
                abs_num = extended[NUM_W-1:0];
        end
    endfunction

    function automatic logic [DEN_W-1:0] abs_den(
        input logic signed [DEN_W-1:0] value
    );
        logic signed [DEN_W:0] extended;
        begin
            extended = {value[DEN_W-1], value};
            if (value[DEN_W-1])
                abs_den = -extended;
            else
                abs_den = extended[DEN_W-1:0];
        end
    endfunction

    always_comb begin
        // Shift partial remainder and bring in the next dividend bit.
        trial_remainder = {remainder_work[DIV_W-1:0], dividend_shift[DIV_W-1]};
        quotient_magnitude = quotient_work;
        max_positive = ({{(DIV_W-1){1'b0}}, 1'b1} << (QUOT_W-1)) - 1'b1;
        max_negative_magnitude = ({{(DIV_W-1){1'b0}}, 1'b1} << (QUOT_W-1));
    end

    assign busy = (state != DIV_IDLE);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state             <= DIV_IDLE;
            done              <= 1'b0;
            quotient          <= '0;
            div_by_zero       <= 1'b0;
            overflow          <= 1'b0;
            dividend_shift    <= '0;
            quotient_work     <= '0;
            remainder_work    <= '0;
            divisor_abs       <= '0;
            bit_count         <= '0;
            quotient_negative <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                DIV_IDLE: begin
                    if (start) begin
                        quotient    <= '0;
                        div_by_zero <= 1'b0;
                        overflow    <= 1'b0;

                        if (denominator == '0) begin
                            div_by_zero <= 1'b1;
                            state       <= DIV_FINISH;
                        end 
                        else begin
                            dividend_shift <=
                                {{(DIV_W-NUM_W){1'b0}}, abs_num(numerator)}
                                << SCALE_SHIFT;
                            quotient_work  <= '0;
                            remainder_work <= '0;
                            divisor_abs    <=
                                {{(DIV_W+1-DEN_W){1'b0}}, abs_den(denominator)};
                            bit_count         <= DIV_W;
                            quotient_negative <= numerator[NUM_W-1] ^ denominator[DEN_W-1];
                            state <= DIV_RUN;
                        end
                    end
                end

                DIV_RUN: begin
                    dividend_shift <= {dividend_shift[DIV_W-2:0], 1'b0};

                    if (trial_remainder >= divisor_abs) begin
                        remainder_work <= trial_remainder - divisor_abs;
                        quotient_work  <= {quotient_work[DIV_W-2:0], 1'b1};
                    end
                    else begin
                        remainder_work <= trial_remainder;
                        quotient_work  <= {quotient_work[DIV_W-2:0], 1'b0};
                    end

                    if (bit_count == 1)
                        state <= DIV_FINISH;
                    bit_count <= bit_count - 1'b1;
                end

            DIV_FINISH: begin
                if (!div_by_zero) begin
                    if (quotient_negative) begin
                        // Negative output range:
                        // -2^(QUOT_W-1) to -1

                        if (quotient_magnitude > max_negative_magnitude) begin
                            quotient <= {1'b1, {(QUOT_W-1){1'b0}}};
                            overflow <= 1'b1;
                        end
                    else begin
                        if (quotient_magnitude == max_negative_magnitude) begin
                            // Exact minimum signed value: -2^(QUOT_W-1)
                            quotient <= {1'b1, {(QUOT_W-1){1'b0}}};
                        end
                        else begin
                            // Magnitude fits in a positive signed value.
                            quotient <= -$signed(quotient_magnitude[QUOT_W-1:0]);
                        end

                        overflow <= 1'b0;
                    end   
                    end
                    else begin
                        // Positive output range:
                        // 0 to 2^(QUOT_W-1)-1

                        if (quotient_magnitude > max_positive) begin
                            quotient <= {1'b0, {(QUOT_W-1){1'b1}}};
                            overflow <= 1'b1;
                        end
                        else begin
                            quotient <= $signed(quotient_magnitude[QUOT_W-1:0]);
                            overflow <= 1'b0;
                        end
                    end
                end

                done  <= 1'b1;
                state <= DIV_IDLE;
            end   
                default: state <= DIV_IDLE;
            endcase
        end
    end

endmodule
