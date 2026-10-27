`timescale 1ns/1ps
module kalman_2state_ip_tb;
    logic clk = 0;
    always #5 clk = ~clk;

    logic rst_n = 0;
    logic init_valid = 0;
    logic signed [15:0] init_x0, init_x1;
    logic signed [39:0] init_p00, init_p01, init_p11;
    logic start = 0;
    logic busy, done, error_status, overflow_seen;
    logic signed [15:0] cfg_f00, cfg_f01, cfg_f10, cfg_f11;
    logic signed [39:0] cfg_q00, cfg_q01, cfg_q11, cfg_r;
    logic signed [15:0] cfg_h0, cfg_h1, measurement;
    logic signed [15:0] state0, state1;
    logic signed [39:0] p00, p01, p11;

    kalman_2state_ip dut (
        .clk, .rst_n, .init_valid, .init_x0, .init_x1,
        .init_p00, .init_p01, .init_p11, .start,
        .busy, .done, .error_status, .overflow_seen,
        .cfg_f00, .cfg_f01, .cfg_f10, .cfg_f11,
        .cfg_q00, .cfg_q01, .cfg_q11, .cfg_h0, .cfg_h1, .cfg_r,
        .measurement, .state0, .state1, .p00, .p01, .p11
    );

    task automatic wait_done;
        integer timeout;
        begin
            timeout = 0;
            while (done !== 1'b1 && timeout < 40) begin
                @(posedge clk);
                timeout++;
            end
            if (timeout >= 40) $fatal(1, "TIMEOUT waiting for done");
            @(negedge clk);
        end
    endtask

    initial begin
        init_x0 = 16'sd13107; // 0.4 in Q1.15
        init_x1 = 16'sd6554;  // 0.2 in Q1.15
        init_p00 = 40'sd42949673; // 0.04 in Q10.30
        init_p01 = 40'sd0;
        init_p11 = 40'sd96636764; // 0.09 in Q10.30

        cfg_f00 = 16'sd16384; cfg_f01 = 0;
        cfg_f10 = 0; cfg_f11 = 16'sd16384; // identity F
        cfg_q00 = 40'sd10737418; // 0.01
        cfg_q01 = 0;
        cfg_q11 = 40'sd10737418;
        cfg_h0 = 16'sd16384; cfg_h1 = 0; // measure state 0
        cfg_r = 40'sd42949673; // 0.04
        measurement = 16'sd16384; // 0.5

        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);
        init_valid = 1;
        @(negedge clk);
        init_valid = 0;

        start = 1;
        @(negedge clk);
        start = 0;
        wait_done();

        $display("state0_raw=%0d state1_raw=%0d", state0, state1);
        $display("p00_raw=%0d p01_raw=%0d p11_raw=%0d", p00, p01, p11);
        $display("error_status=%0b overflow_seen=%0b", error_status, overflow_seen);

        if (error_status) $fatal(1, "Kalman transaction reported error");
        // State 0 should move from approximately 0.4 toward measurement 0.5.
        if (!(state0 > 16'sd13107 && state0 < 16'sd16384))
            $fatal(1, "state0 did not move toward measurement");
        // State 1 is unobserved and uncorrelated in this setup.
        if (state1 !== 16'sd6554)
            $fatal(1, "state1 changed unexpectedly: %0d", state1);
        // Updated variance should decrease below predicted P00 ~= 0.05.
        if (!(p00 > 0 && p00 < 40'sd53687091))
            $fatal(1, "p00 did not decrease as expected: %0d", p00);

        $display("PASS: basic one-measurement Kalman smoke test");
        $finish;
    end
endmodule
