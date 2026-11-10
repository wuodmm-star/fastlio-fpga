# FAST-LIO nearest-neighbour search on 征途Pro — design draft (2026-11-10)

Goal (portfolio): run FAST-LIO's heaviest step — finding the 5 nearest map points of every scan point — on a
no-ARM Cyclone IV E (EP4CE10) with the map in the on-board 32 MB SDRAM, bit-exact against a golden model, and
FAST-LIO's trajectory error unchanged. Expected NOT to beat the PC; the point is "it fits and it is verified".

Measured on NTU-VIRAL eee_03 (Ubuntu desktop, FAST-LIO runtime log): per scan 23.3 ms total, of which "match"
(NN search + plane fit, OpenMP) 17.8 ms, H construction 0.2 ms.

## Phase 0 — FPGA-friendly map in software (include/voxmap.hpp, FPGA_NN_MODE=voxel)
0.5 m voxels (= FAST-LIO filter_size_map), hash -> bucket of <= 4 points; FAST-LIO's own insert rules;
search the (2R+1)^3 voxels around the query, keep the 5 nearest. Pass = ATE close to ikd-tree (11.6 cm).

## Map in SDRAM (draft)
- SDRAM: 16-bit words, 16 M words (32 MB). Embedfire sdram_* modules are FIFO/stream oriented -> write a
  random-access burst reader for buckets; reuse their init / refresh timing.
- Point: x, y, z as int32 Q16.16 metres (world frame) = 6 words.
- Bucket (one hash slot) = 32 words = 64 bytes: word 0 = count (0..4), words 1..3 = voxel key tag (3 x 16 bit
  is not enough for 21-bit indices -> pack 63 bits into 4 words: words 1..4), then 4 points x 6 words = 24 words.
- Table: 2^18 buckets x 64 B = 16 MB (half the SDRAM). Index = hash(voxel x,y,z) mod 2^18; the tag detects
  collisions (a colliding voxel is treated as "different voxel" -> linear probe 1 slot; golden model copies this).
- Search cost (R=1): 27 buckets; read count + tag first (5 words), then only the occupied points.
  Rough: ~10 cycles row activation + 5 + 6*n words -> ~400-600 cycles per query at 100 MHz = 4-6 us,
  2000 queries ~ 10 ms per search pass (PC: 17.8 ms per scan incl. plane fit with 16 threads).
- Updates: FAST-LIO adds a few hundred points per scan (downsample insert = read bucket, keep closest to centre,
  write back; plain insert = append).
- Link: per scan ~2000 query points x 12 B in, 5 neighbours x 12 B out (120 KB) -> ~1.5-3 MB/s at 10 Hz,
  inside the 100 Mb/s budget (measured ~11 MB/s possible). Later: plane fit on the FPGA -> 16 B out per point.

## Phase 0 result (2026-11-10, eee_03, 1810 scans)
| map | ATE RMSE | "match" per scan |
|---|---|---|
| ikd-tree (original) | 11.62 / 11.61 cm | 17.8 ms |
| voxel hash, R=1 (27 voxels) | 11.35 cm | 13.3 ms (ikd-tree still maintained in the background) |
| voxel hash, R=2 (125 voxels) | 11.57 cm | 31.1 ms |
=> R=1 is enough. PASS.

## Phase 1 — FPGA rules fixed (2026-11-10)
First fixed-point version (2^18 slots, hash = low-bit XOR, 2 probes) dropped 1.45 M adds on eee_03: the map is
~220 x 200 m and the occupied voxels are thin surfaces, so a bit-fold hash collides badly (up to 12 voxels/slot).
Measured on all 152,917 voxels of the run (same 524,288-slot capacity):
| table | dropped voxels |
|---|---|
| 4-way x 2^17 groups | 527 (0.35 %) |
| **8-way x 2^16 groups** | **33 (0.022 %)** |
| 16-way x 2^15 groups | 0 |
Final rules (voxmap.hpp MapFx == model/nn_model.py):
- point = Q16.16; voxel = coord >> 15; stored as 3 x 16-bit offsets inside the voxel (lossless)
- slot = 4 header words (tag 3 x 17 bit + count) + 4 points x 3 words = 16 words (32 B)
- group = 8 slots = 128 words = one SDRAM row segment; 2^16 groups = 16 MB
- group index = bits 8..23 of (uint32)(x + 1021 y + 262147 z) * 2654435761
FAST-LIO with these rules (software): 146,740 voxels, 35 dropped adds, **ATE 11.46 cm** (ikd-tree 11.62 / 11.61).

## Phase 2 — SDRAM controller (rtl/sdram_rw.v), simulation only so far
Own random-access controller (open row -> full-page burst cut after LEN words -> close row; auto-refresh between
requests every 7.5 us; init 200 us + PRE + 8 x REF + MRS(CL3, full page)). Simulated against Micron's
mt48lc16m16a2 model (same geometry as the board's W9825G6: 13 row / 9 col bits, 4 banks), SDRAM clock leading by
0.833 ns like the Embedfire PLL. Random test (4000 requests, lengths 1-64, 50 % writes, 16 rows, ~245 refreshes):
40,652 words compared, 0 wrong, no model timing warnings with RD_LAT = 3 (4 and 5 shift by 1/2 words).
RD_LAT must be re-checked on the board.
Testbench pitfall: `assign #9.167 sdram_clk = clk` swallows 5 ns pulses (inertial delay) -> model never clocked.

## Phase 3 — nn_core + Ethernet, simulation and first hardware results (2026-11-10)
- nn_core (rtl/nn_core.v): 8-way group header read -> tag match / free slot -> points -> top-5 (search) or
  closest-to-centre / append / replace-farthest (add). Clears all 65,536 group headers after reset (SDRAM powers up
  random; first sim had every add "dropped" because uncleared headers looked occupied). ~29 ms at 100 MHz.
- sdram_rw got a req/ack handshake (a request arriving while a refresh started was lost before); refresh every 6 us.
- Simulation (Micron model): real FAST-LIO ops 60,000 adds (57k downsample) + 395 searches bit-exact; synthetic
  stress 6000 ops (629 replace-farthest, 867 downsample, 2110 searches) bit-exact; whole chip (Ethernet frames in,
  51 replies out) bit-exact. Python golden == FAST-LIO C++ MapFx on the whole run (968,641 adds, 11,965,621 searches).
- Bug found in whole-chip sim: reply length registered on the same 25 MHz edge that started ip_send -> previous length.
- Timing closure at 100 MHz (EP4CE10 C8): -9.5 ns -> +0.086 ns by: 3-stage centre distance, 18-bit search deltas,
  3-stage hash with shift-add constants, registered tag search, 2-stage squared distance, counters instead of /3 %3,
  31-bit centre distances. 7,279 LE (71 %), 24/46 9-bit multipliers, 98 kbit RAM.
- Hardware (first programming, RD_LAT = 3 as in simulation): synthetic 2110/2110 searches bit-exact;
  real FAST-LIO ops 395/395 bit-exact after reprogramming (a first run failed 2/395 because the map still held the
  previous test's points -> need a CLEAR command). Stop-and-wait from Windows Python: ~80-100 us per search incl.
  the PC round trip (18 searches per packet).

## Phase 4 — FAST-LIO closed loop (FPGA_NN_MODE=fpga, voxmap.hpp FpgaNN; WSL2 -> board)
- Client: searches of one IEKF iteration batched before the parallel match loop (18 per packet), adds buffered and
  sent once per scan (72 per packet), up to 4 packets in flight, CLEAR (0xA513) at start-up. Same quantisation as
  MapFx -> answers identical to the voxfx software mode.
- First run: after 8,698 adds / 14,420 searches (21.2 us per search incl. network) one reply was missing
  ("packet 133 of 134"). Cause: a reply announced while ip_send was still sending the previous (long) one -> start
  pulse ignored. Fix: reply queue in the 25 MHz domain (q_cnt / tx_busy / send_go). Reproduced in the whole-chip
  simulation with 4 packets in flight: original 72/73 replies, fixed 73/73 bit-exact. 7,311 LE, slack +0.088 ns.
- Closed loop after the fix (eee_03, 1810 scans, bag at 0.25x): 0 reply timeouts; the board handled 969,140 adds
  (2.1 ms per scan on average) and 11,919,724 searches (18.9 us each incl. the WSL->board round trip, ~6,600 per
  scan -> ~125 ms per scan; the PC's whole "match" step is 17.8 ms per scan with 16 threads).

| run | ATE RMSE | max |
|---|---|---|
| ikd-tree, CPU (desktop / WSL) | 11.62 / 11.57 cm | 32.7 / 30.0 cm |
| voxel map float (software) | 11.35 cm | 30.0 cm |
| voxel map, FPGA rules (software) | 11.46 cm | 35.3 cm |
| **map + NN search on the FPGA** | **11.31 cm** | 27.8 cm |
All within FAST-LIO's own run-to-run spread (two identical CPU runs differ by up to 4.8 cm in position).

## Speed-up v2: summary layout (2026-11-10)
Measured in simulation (v1, real ops): ~1400 clocks per search = 27 neighbour voxels x ~52 clocks, of which 32 clocks
read all 8 slot headers of a group. v2 layout: group words 0-1 = slot counts, 2-5 = 8-bit fingerprints
(h[31:24]), slot k at 8 + 15k = 3 tag words + 4 points; a lookup reads 6 summary words and reads tag + points
(one burst) only for slots whose fingerprint matches. Tags shrink to 16 bits (|coordinate| < 16 km). Results are
unchanged (same slot allocation and drop rule). Old core kept as rtl/nn_core_v1.v.
- simulation: synthetic 2110/2110 and real 395/395 bit-exact; real ops ~740 clocks per search (1.9x faster)
- Quartus: 7,048 LE (68 %), 24 multipliers, worst setup slack +0.215 ns at 100 MHz
- hardware: synthetic 2110/2110, real 395/395 bit-exact
- FAST-LIO closed loop, v2 (eee_03, 1810 scans, bag at 0.25x, 16 search packets in flight): 0 reply timeouts,
  975,196 adds (2.0 ms per scan), 11,919,684 searches at 11.35 us each incl. the WSL->board round trip
  (v1: 18.9 us) -> ~75 ms per scan for the searches, ~77 ms with the adds (10 Hz LiDAR budget: 100 ms).
  **ATE 11.40 cm** (max 34.7 cm). With 4 packets in flight v2 gave no gain (20.2 us): the bottleneck had moved to
  the per-packet cost of the WSL -> Windows -> board path.
