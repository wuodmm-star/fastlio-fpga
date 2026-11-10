# HTH engine results (2026-11-09/10)

## Step 2 — golden model (model/test_model.py)
- Fixed-point formats in SPEC.md. 20 synthetic iterations (200-2500 points, ranges 1-60 m):
  worst relative error vs float64 FAST-LIO formulas: HTH 1.8e-7, HTh 1.7e-6;
  resulting IEKF update dx differs by ~1e-9 while |dx| ~ 1e-3.
- Microcode: 54 instructions/point (54 multiplies, 0 NOP stalls, pipeline latency 3), checked with a
  cycle model == golden on 4 full iterations.

## Step 3 — RTL
Simulation (iverilog):
- engine only (sim/tb_engine.v): 5 iterations (1, 41, 42 with a junk word, 997, 300 points at 8 km) -> all 5 replies
  bit-exact (bad-header counter = 1 where expected). Negative control: shift 30->31 => 18/27 sums wrong, detected.
- whole chip (sim/tb_system.v): real Ethernet frames on the RMII pins -> 2 reply frames, FCS + IP checksum OK,
  sent back to the sender's port, payload bit-exact.
- Bugs found by simulation: (1) reply announced in the same clock as its last word was written -> added S_FIN;
  (2) testbench reset without a real 1->0 edge left async-reset logic at X.

Quartus II 13.0 (EP4CE10F17C8): 9,966 LE (97 %), 4,465 registers, 69,632 memory bits, 8 multiplier 9-bit elements;
worst setup slack +2.88 ns at 50 MHz (eth_clk), +10.5 ns at 25 MHz; hold +0.17 ns. RMII pin registers placed in I/O cells.

Hardware (Windows laptop, Python client host/hth_hw_test.py, random iterations of 300-3000 points):

| run | iterations | bit-exact | wrong | no reply | points/s (incl. PC) |
|---|---|---|---|---|---|
| first | 50 | 46 | 4 | 0 | 135k |
| seed 3, no pause / 2 ms pause | 100 + 100 | 200 | 0 | 0 | 141k / 120k |
| seed 7 | 1000 | 992 | 8 | 0 | 164k |
| seed 7 (NIC counters: all 42,944 datagrams sent, 0 errors) | 1000 | 993 | 7 | 0 | — |
| seed 7, one header-only warm-up packet before BEGIN | 1000 | **1000** | 0 | 0 | **282k** |
| seed 7, NIC "Power Saving Mode" off (restored afterwards) | 1000 | 998 | 2 | 0 | 131k |

Every wrong reply = the first packet(s) of an iteration never reached the engine (id of the previous iteration and
the point counts of two iterations added; once 5 point packets right after BEGIN). Windows says it sent them.
Root cause still open (candidates: NIC energy saving / first frames after idle, board RMII receive). Mitigations:
warm-up packet + the protocol check (id, point count) with re-send.

Open items: LE usage 97 % (sums/register file in LEs -> move to M9K); board-side frame counters to locate the loss.

## Step 4 — real FAST-LIO data (NTU-VIRAL eee_03, Ouster OS1-16 + VN100, 1810 scans, 127.8 m path)
FAST-LIO (ROS2 Humble, official cc3c9e8) + hook include/fpga_hth.hpp (modes cpu / fixed / fpga, dump, TUM trajectory).
Fixes needed to run the bag: rosbags-convert v8 metadata rewritten for Humble; lidar subscription SensorDataQoS (best
effort, depth 5) dropped 2/3 of the scans (658/1814 processed) -> reliable QoS depth 200 -> 1810/1814.

| run | ATE RMSE | max |
|---|---|---|
| CPU (original), run 1 | 11.62 cm | 32.7 cm |
| CPU (original), run 2 | 11.61 cm | 35.2 cm |
| software model of the FPGA fixed-point math, closed loop | 11.58 cm | 29.4 cm |
Two identical CPU runs differ by up to 4.8 cm (FAST-LIO multithreading is not deterministic); fixed-point is inside that.

Recorded iterations (7069, median 1915 effective points/iteration in the first 800, |coord| <= 102 m, |pd2| <= 0.62 m):
- golden fixed-point vs FAST-LIO float64: HTH max rel. error 1.9e-7; HTh median 1.2e-5, worst 5e-3 (HTh is a sum of
  residuals that cancel near convergence, so its relative error is inflated).
- replayed on the board (Windows client, 1 warm-up packet): 7064/7069 bit-exact, 4 wrong + 1 no reply (= the known
  loss of the first packets of an iteration; the protocol detects it and FAST-LIO's client re-sends);
  6.10 ms per iteration incl. Python on the PC, 204,505 points/s.

## Step 3b — smaller design + root cause of the lost packets (2026-11-10)
- 27 sums moved from registers (each with its own 80-bit adder) into one M9K RAM with a 2-clock read-modify-write
  (generator asserts no sum is reused within 2 clocks): 9,966 LE (97 %) -> 5,103 LE (49 %), worst setup slack +3.6 ns.
  Engine + system simulations still bit-exact.
- Board receive counters added (frames started / preamble+SFD ok / accepted, reported in reply word 3).
  1000 iterations before the fix: PC sent 42,856 datagrams, board saw 42,990 frame starts (extra = Windows broadcasts),
  only 42,945 passed preamble+SFD, 42,835 accepted. Lossy iterations always contained foreign frames, e.g. sent 24,
  started 26, SFD ok 16.
- Root cause (in the Embedfire ip_receive used by all their Ethernet examples): the receive state machine and its byte
  counters were never reset at the end of a frame. A foreign frame that ended mid-header (e.g. trailing 0x55 bytes
  that looked like a preamble) left it mid-count, and the next frame's preamble was mis-counted and dropped.
  Fix: force IDLE and clear cnt_byte / cnt_data / cnt_rec_data whenever rx_dv is low.
  Reproduced in simulation (foreign broadcast frames ending in 0x55 between our frames): original drops the BEGINs
  (reply id 0, 0 points; second reply missing), fixed version bit-exact.
- After the fix, no warm-up packet: 1000/1000 bit-exact; counters: started 42,967 = SFD ok 42,967, accepted 42,872 =
  sent 42,872. Real FAST-LIO iterations: 7069/7069 bit-exact, 4.43 ms per iteration, 281,442 points/s.

## Step 5 — closed loop with the real board (FAST-LIO in WSL2 Ubuntu 22.04 on the laptop, NAT, board on Ethernet)
FAST-LIO used the board's HTH / HTh for every IEKF iteration of eee_03 (1810 scans, ~8,100 iterations, ~8.1 M points),
0 retries, 4.4 ms per iteration (UDP round trip incl.).

| run | ATE RMSE | max |
|---|---|---|
| CPU, Ubuntu desktop, run 1 / run 2 | 11.62 / 11.61 cm | 32.7 / 35.2 cm |
| fixed-point software model, Ubuntu desktop | 11.58 cm | 29.4 cm |
| CPU, WSL laptop | 11.57 cm | 30.0 cm |
| **FPGA (board in the loop), WSL laptop** | **11.63 cm** | 32.5 cm |
All within the run-to-run spread of FAST-LIO itself (two identical CPU runs differ by up to 4.8 cm in position).
