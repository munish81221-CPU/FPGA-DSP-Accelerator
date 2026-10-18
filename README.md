
# Shared Arithmetic Engine — Milestone 1

## Purpose

Portable, board-independent SystemVerilog starting point for the shared arithmetic engine in the LMS/NLMS/Kalman FPGA DSP accelerator.

## Files

- `arithmetic_engine.sv` — synthesizable arithmetic engine RTL.
- `arithmetic_engine_tb.sv` — self-checking simulation testbench.

## Interface and timing

A command is accepted on a rising clock edge when `cmd_valid && cmd_ready` is true. The engine captures the opcode and operands, asserts `busy`, executes the command on the next rising edge, pulses `result_valid`, and deasserts `busy`. Commands are not pipelined; the controller must wait for `cmd_ready` before issuing another command.

`overflow` is meaningful with the `result_valid` completion pulse. Invalid opcodes return zero and assert `overflow`.

## Opcode map

| Opcode | Name | Behavior |
|---|---|---|
| `000` | `OP_MUL` | Signed Q1.15 multiply, rounded/saturated to Q1.15 output |
| `001` | `OP_MAC` | Add signed Q*.30 product into the 40-bit accumulator; output the rounded/saturated accumulator |
| `010` | `OP_ADD` | Signed Q1.15 addition with saturation |
| `011` | `OP_SUB` | Signed Q1.15 subtraction with saturation |
| `100` | `OP_ACC_CLEAR` | Clear the internal accumulator |
| `101` | `OP_ACC_READ` | Read accumulator converted from Q30 to Q1.15 |

## Numerical convention

- External operands and `result`: signed 16-bit Q1.15.
- Product: signed 32-bit Q2.30.
- Accumulator: signed 40-bit with 30 fractional bits.
- Q30-to-Q15 conversion: round to nearest, ties away from zero, then saturate.
- Addition/subtraction: saturating Q1.15 operations.

The initial implementation targets `DATA_W=16`, `ACC_W=40`. Although widths are parameters, the scaling constants and Q30-to-Q15 conversion currently assume this format; do not change these parameters without updating the numerical logic.

## Run the testbench

With Icarus Verilog installed:

```sh
iverilog -g2012 -s arithmetic_engine_tb -o arithmetic_engine_sim arithmetic_engine.sv arithmetic_engine_tb.sv
vvp arithmetic_engine_sim
```

A successful run ends with `ALL TESTS PASSED`.

## Known scope limits for this first milestone

- No divider/reciprocal unit yet. NLMS and Kalman scheduling will require a planned reciprocal/division implementation or a validated alternative.
- No memory interface or algorithm sequencer yet.
- No vendor-specific FPGA primitives.
- Synthesis/timing/resource claims must wait until the target FPGA/toolchain is known.

## Integration contract

The algorithm sequencer should present stable opcode/operands while `cmd_valid` is asserted and only consider the command accepted on `cmd_valid && cmd_ready`. It should wait for `result_valid` before consuming `result` and `overflow`.
=======
# FPGA-DSP-Accelerator
A fixed-point FPGA DSP accelerator implementing LMS, NLMS, and Kalman filtering using System-Verilog, a shared arithmetic engine, and modular control logic.

