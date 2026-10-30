`timescale 1ns/1ps

// KAL-004A: independent real-valued reference checks for the one-measurement
// kalman_2state_ip MVP. The reference uses real-number matrix equations rather
// than copying the DUT's fixed-point shifts/products. Inputs are quantized to
// the DUT's declared formats before the real model is calculated.
//
// This is a mathematical cross-check, not yet a bit-accurate fixed-point model.
// The tolerances allow for the DUT's intermediate fixed-point rounding.

module kalman_2state_reference_tb;
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

    integer tests_passed = 0;
    integer tests_failed = 0;

    localparam real STATE_SCALE = 32768.0;
    localparam real COEF_SCALE  = 16384.0;
    localparam real COV_SCALE   = 1073741824.0;

    kalman_2state_ip dut (
        .clk, .rst_n, .init_valid, .init_x0, .init_x1,
        .init_p00, .init_p01, .init_p11, .start,
        .busy, .done, .error_status, .overflow_seen,
        .cfg_f00, .cfg_f01, .cfg_f10, .cfg_f11,
        .cfg_q00, .cfg_q01, .cfg_q11, .cfg_h0, .cfg_h1, .cfg_r,
        .measurement, .state0, .state1, .p00, .p01, .p11
    );

    function automatic integer enc_state(input real v);
        real scaled;
        begin
            scaled = v * STATE_SCALE;
            if (scaled >= 0.0) enc_state = $rtoi(scaled + 0.5);
            else               enc_state = $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic integer enc_coef(input real v);
        real scaled;
        begin
            scaled = v * COEF_SCALE;
            if (scaled >= 0.0) enc_coef = $rtoi(scaled + 0.5);
            else               enc_coef = $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic longint signed enc_cov(input real v);
        real scaled;
        begin
            scaled = v * COV_SCALE;
            if (scaled >= 0.0) enc_cov = $rtoi(scaled + 0.5);
            else               enc_cov = $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic real dec_state(input logic signed [15:0] v);
        dec_state = $itor($signed(v)) / STATE_SCALE;
    endfunction

    function automatic real dec_cov(input logic signed [39:0] v);
        dec_cov = $itor($signed(v)) / COV_SCALE;
    endfunction

    task automatic wait_for_done;
        integer cycles;
        begin
            cycles = 0;
            while ((done !== 1'b1) && (cycles < 50)) begin
                @(posedge clk);
                cycles = cycles + 1;
            end
            if (cycles >= 50) begin
                $display("FAIL: timeout waiting for done");
                tests_failed = tests_failed + 1;
            end
            @(negedge clk);
        end
    endtask

    // Inputs are mathematical real values. They are quantized to the DUT
    // interface formats first, then the reference equations use those decoded
    // quantized inputs and the actual quantized initial state/covariance.
    task automatic run_case(
        input string case_name,
        input real ix0, input real ix1,
        input real ip00, input real ip01, input real ip11,
        input real f00, input real f01, input real f10, input real f11,
        input real q00, input real q01, input real q11,
        input real h0, input real h1, input real rv, input real z
    );
        real x0, x1, p00r, p01r, p11r;
        real ff00, ff01, ff10, ff11, qq00, qq01, qq11, hh0, hh1, rr, zz;
        real xp0, xp1;
        real pp00r, pp01r, pp11r;
        real innov, s, k0, k1, xn0, xn1;
        real hp0r, hp1r;
        real pn00, pn01, pn11;
        real err_x0, err_x1, err_p00, err_p01, err_p11;
        real state_tol, cov_tol;
        begin
            // Quantize all inputs to the exact interface formats.
            init_x0 = enc_state(ix0);
            init_x1 = enc_state(ix1);
            init_p00 = enc_cov(ip00);
            init_p01 = enc_cov(ip01);
            init_p11 = enc_cov(ip11);

            cfg_f00 = enc_coef(f00); cfg_f01 = enc_coef(f01);
            cfg_f10 = enc_coef(f10); cfg_f11 = enc_coef(f11);
            cfg_q00 = enc_cov(q00); cfg_q01 = enc_cov(q01); cfg_q11 = enc_cov(q11);
            cfg_h0 = enc_coef(h0); cfg_h1 = enc_coef(h1);
            cfg_r = enc_cov(rv);
            measurement = enc_state(z);

            // Decode the quantized values for the independent real reference.
            x0 = dec_state(init_x0); x1 = dec_state(init_x1);
            p00r = dec_cov(init_p00); p01r = dec_cov(init_p01); p11r = dec_cov(init_p11);
            ff00 = $itor($signed(cfg_f00))/COEF_SCALE;
            ff01 = $itor($signed(cfg_f01))/COEF_SCALE;
            ff10 = $itor($signed(cfg_f10))/COEF_SCALE;
            ff11 = $itor($signed(cfg_f11))/COEF_SCALE;
            qq00 = dec_cov(cfg_q00); qq01 = dec_cov(cfg_q01); qq11 = dec_cov(cfg_q11);
            hh0 = $itor($signed(cfg_h0))/COEF_SCALE;
            hh1 = $itor($signed(cfg_h1))/COEF_SCALE;
            rr = dec_cov(cfg_r);
            zz = dec_state(measurement);

            // Independent real-valued Kalman equations.
            xp0 = ff00*x0 + ff01*x1;
            xp1 = ff10*x0 + ff11*x1;

            pp00r = ff00*(p00r*ff00 + p01r*ff01)
                  + ff01*(p01r*ff00 + p11r*ff01) + qq00;
            pp01r = ff00*(p00r*ff10 + p01r*ff11)
                  + ff01*(p01r*ff10 + p11r*ff11) + qq01;
            pp11r = ff10*(p00r*ff10 + p01r*ff11)
                  + ff11*(p01r*ff10 + p11r*ff11) + qq11;

            innov = zz - (hh0*xp0 + hh1*xp1);
            hp0r = hh0*pp00r + hh1*pp01r;
            hp1r = hh0*pp01r + hh1*pp11r;
            s = hh0*hp0r + hh1*hp1r + rr;

            if (s <= 0.0) begin
                $display("SKIP %s: reference S is non-positive (%f)", case_name, s);
                tests_failed = tests_failed + 1;
            end else begin
                k0 = hp0r/s;
                k1 = hp1r/s;
                xn0 = xp0 + k0*innov;
                xn1 = xp1 + k1*innov;

                // Match the current DUT's simplified covariance equation.
                pn00 = pp00r - k0*hp0r;
                pn01 = pp01r - k0*hp1r;
                pn11 = pp11r - k1*hp1r;

                // Initialize DUT for an isolated transaction.
                @(negedge clk);
                init_valid = 1;
                @(negedge clk);
                init_valid = 0;
                start = 1;
                @(negedge clk);
                start = 0;
                wait_for_done();

                err_x0 = dec_state(state0) - xn0;
                err_x1 = dec_state(state1) - xn1;
                err_p00 = dec_cov(p00) - pn00;
                err_p01 = dec_cov(p01) - pn01;
                err_p11 = dec_cov(p11) - pn11;

                // This is a real-valued mathematical cross-check, not a
                // bit-accurate model. The DUT rounds several matrix-product
                // terms at intermediate boundaries, so covariance error can
                // accumulate across those stages. Start with 12 state LSBs
                // and 4096 covariance LSBs; tighten these after a bit-accurate
                // reference model is available.
                state_tol = 12.0 / STATE_SCALE;
                cov_tol = 4096.0 / COV_SCALE;

                if (error_status) begin
                    $display("FAIL %s: DUT error_status asserted", case_name);
                    tests_failed = tests_failed + 1;
                end else if ((err_x0 > state_tol) || (err_x0 < -state_tol) ||
                             (err_x1 > state_tol) || (err_x1 < -state_tol) ||
                             (err_p00 > cov_tol) || (err_p00 < -cov_tol) ||
                             (err_p01 > cov_tol) || (err_p01 < -cov_tol) ||
                             (err_p11 > cov_tol) || (err_p11 < -cov_tol)) begin
                    $display("FAIL %s", case_name);
                    $display("  state DUT=(%f,%f) REF=(%f,%f)", dec_state(state0), dec_state(state1), xn0, xn1);
                    $display("  P DUT=(%f,%f,%f) REF=(%f,%f,%f)",
                             dec_cov(p00), dec_cov(p01), dec_cov(p11), pn00, pn01, pn11);
                    $display("  errors x=(%e,%e), P=(%e,%e,%e)",
                             err_x0, err_x1, err_p00, err_p01, err_p11);
                    tests_failed = tests_failed + 1;
                end else begin
                    $display("PASS %s: x=(%f,%f) P=(%f,%f,%f)",
                             case_name, dec_state(state0), dec_state(state1),
                             dec_cov(p00), dec_cov(p01), dec_cov(p11));
                    tests_passed = tests_passed + 1;
                end
            end
        end
    endtask

    initial begin
        // Defaults to avoid X values during reset.
        init_x0=0; init_x1=0; init_p00=0; init_p01=0; init_p11=0;
        cfg_f00=0; cfg_f01=0; cfg_f10=0; cfg_f11=0;
        cfg_q00=0; cfg_q01=0; cfg_q11=0; cfg_h0=0; cfg_h1=0; cfg_r=0; measurement=0;

        repeat (3) @(negedge clk);
        rst_n = 1;

        // R01: original hand-calculated example.
        run_case("R01_identity_original", 0.4, 0.2, 0.04, 0.0, 0.09,
                 1.0, 0.0, 0.0, 1.0, 0.01, 0.0, 0.01,
                 1.0, 0.0, 0.04, 0.5);

        // R02: non-identity F; both predicted states depend on both previous states.
        run_case("R02_nonidentity_F", 0.25, -0.10, 0.08, 0.01, 0.06,
                 0.90, 0.10, -0.05, 0.95, 0.005, 0.001, 0.006,
                 1.0, 0.25, 0.03, 0.18);

        // R03: correlated initial state; measurement can update both components.
        run_case("R03_correlated_states", -0.15, 0.30, 0.07, 0.025, 0.08,
                 1.0, 0.0, 0.0, 1.0, 0.002, 0.0, 0.003,
                 1.0, 0.5, 0.02, 0.05);

        // R04: negative innovation.
        run_case("R04_negative_innovation", 0.35, -0.20, 0.05, 0.0, 0.07,
                 1.0, 0.0, 0.0, 1.0, 0.002, 0.0, 0.002,
                 1.0, 0.0, 0.03, 0.10);

        // R05: nontrivial measurement combination.
        run_case("R05_mixed_measurement", 0.10, 0.20, 0.06, 0.015, 0.05,
                 0.98, 0.08, -0.02, 0.90, 0.003, 0.0, 0.004,
                 0.75, -0.50, 0.025, 0.12);

        $display("KAL-004A SUMMARY: passed=%0d failed=%0d", tests_passed, tests_failed);
        if (tests_failed != 0) $fatal(1, "KAL-004A reference verification failed");
        if (tests_passed != 5) $fatal(1, "KAL-004A did not run all 5 cases");
        $display("ALL KAL-004A REFERENCE CASES PASSED");
        $finish;
    end
endmodule
