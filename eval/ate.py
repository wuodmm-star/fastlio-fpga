"""ATE of a FAST-LIO TUM trajectory against NTU-VIRAL Leica ground truth (prism position).
usage: python ate.py traj.tum [traj2.tum ...]
Body pose -> prism position: p_prism = p + R * t_body_prism, then rigid (rotation+translation) Umeyama alignment.
"""
import sys
import numpy as np

GT = "gt_eee_03.csv"   # NTU-VIRAL ground_truth/eee_03/ground_truth.csv
T_BODY_PRISM = np.array([-0.293656, -0.012288, -0.273095])


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_gt():
    a = np.loadtxt(GT, delimiter=",", skiprows=1)
    return a[:, 2] * 1e-9, a[:, 3:6]


def ate(path):
    tr = np.loadtxt(path)
    t = tr[:, 0]
    P = np.array([r[1:4] + quat_to_R(*r[4:8]) @ T_BODY_PRISM for r in tr])
    gt_t, gt_p = load_gt()
    # associate each GT sample with the trajectory interpolated at that time
    m = (gt_t >= t[0]) & (gt_t <= t[-1])
    gt_t, gt_p = gt_t[m], gt_p[m]
    est = np.column_stack([np.interp(gt_t, t, P[:, i]) for i in range(3)])
    mu_e, mu_g = est.mean(0), gt_p.mean(0)
    U, _, Vt = np.linalg.svd((gt_p - mu_g).T @ (est - mu_e))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    err = np.linalg.norm((est - mu_e) @ R.T + mu_g - gt_p, axis=1)
    return np.sqrt((err ** 2).mean()), err.max(), len(gt_t), np.linalg.norm(np.diff(gt_p, axis=0), axis=1).sum()


if __name__ == "__main__":
    for p in sys.argv[1:]:
        rmse, mx, n, L = ate(p)
        print(f"{p}: ATE RMSE {rmse * 100:.2f} cm, max {mx * 100:.2f} cm  ({n} GT samples, path {L:.1f} m)")
