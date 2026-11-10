"""Step 2 checks (no hardware):
 a) golden fixed-point vs float64 FAST-LIO formulas  -> how much precision the integer format costs
 b) microcode table run through a cycle model        -> must equal the golden sums exactly
 c) writes the Verilog microcode include file
"""
import os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hth_model as M

rng = np.random.default_rng(2026)
LAT = 3
prog = M.make_microcode(LAT)
n_nop = sum(op["mode"] == M.NOP for op in prog)
print(f"microcode: {len(prog)} instructions per point ({len(prog) - n_nop} multiplies, {n_nop} NOP), latency {LAT}")

worst_rel_hth = worst_rel_hth_e = 0.0
t0 = time.time()
for it in range(20):
    npts = int(rng.integers(200, 2500))
    R, RL, tL, pb, n, pd2 = M.synthetic_iteration(rng, npts)
    qd = M.quantize(R, RL, tL, pb, n, pd2)
    S = M.golden_fixed(qd)
    HTH_q, HTh_q = M.sums_to_float(S)
    HTH_f, HTh_f = M.reference_float(R, RL, tL, pb, n, pd2)
    rel1 = np.abs(HTH_q - HTH_f).max() / np.abs(HTH_f).max()
    rel2 = np.abs(HTh_q - HTh_f).max() / np.abs(HTh_f).max()
    worst_rel_hth = max(worst_rel_hth, rel1); worst_rel_hth_e = max(worst_rel_hth_e, rel2)
    if it < 4:   # microcode cycle model is slow in Python; check a few iterations fully
        S_u = M.simulate_microcode(prog, qd, LAT)
        assert S_u == S, f"microcode mismatch in iteration {it}"
    # the solution FAST-LIO actually uses: dx = (HTH/R + P^-1)^-1 HTh/R ; compare with a typical P
    Rm = 0.001
    Pinv = np.eye(6) * 1e4
    dx_q = np.linalg.solve(HTH_q / Rm + Pinv, HTh_q / Rm)
    dx_f = np.linalg.solve(HTH_f / Rm + Pinv, HTh_f / Rm)
    if it < 4:
        print(f"iter {it}: {npts:4d} pts | max rel err HTH {rel1:.2e}  HTh {rel2:.2e} | "
              f"update dx diff {np.abs(dx_q - dx_f).max():.2e} (|dx| {np.abs(dx_f).max():.2e}) | microcode == golden: yes")
print(f"20 iterations: worst relative error HTH {worst_rel_hth:.2e}, HTh {worst_rel_hth_e:.2e}  ({time.time() - t0:.1f}s)")

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "hth_ucode.vh")
os.makedirs(os.path.dirname(out), exist_ok=True)
M.write_verilog_ucode(prog, out)
print("wrote", os.path.normpath(out))
