# HTH engine — what the FPGA computes and how the PC talks to it

FAST-LIO (`laserMapping.cpp: h_share_model` + `esekfom::update_iterated_dyn_share_modified`) spends each
IEKF iteration building, for every matched point i,

    p_i  = R_LI * pb_i + t_LI            (point in IMU frame)
    C_i  = R^T * n_i                      (plane normal rotated into body frame)
    A_i  = p_i x C_i                      (= skew(p_i) * C_i)
    H_i  = [ n_i , A_i ]                  (6 numbers; the other 6 columns of the 12-wide row are 0)
    h_i  = -pd2_i                         (point-to-plane residual)

and then only uses   HTH = sum_i H_i H_i^T  (6x6, symmetric -> 21 numbers)   and   HTh = sum_i H_i h_i  (6).
The FPGA does exactly this per point and keeps the 27 sums. (extrinsic estimation off, as in the default configs)

## Number formats (all inputs are signed 32-bit integers, big-endian on the wire)

| quantity | format | meaning of 1 LSB | range |
|---|---|---|---|
| R, R_LI (rotation entries) | Q2.30 | 2^-30 | +-2 |
| t_LI, pb (metres) | Q16.16 | 2^-16 m (15 um) | +-32768 m (we require < 16384 m) |
| n (unit normal) | Q2.30 | 2^-30 | +-2 |
| pd2 (metres) | Q8.24 | 2^-24 m | +-128 m |

PC rounds floats to nearest. Inside the FPGA (and in the golden model) every intermediate is exact integer math
with ONE rule: `>> 30` is an arithmetic shift (floor), result kept as 32-bit.

    p_i = (RL[i][0]*pb0 + RL[i][1]*pb1 + RL[i][2]*pb2 + tL_i * 2^30) >> 30     -> Q16.16
    C_i = (R[0][i]*n0 + R[1][i]*n1 + R[2][i]*n2) >> 30                         -> Q2.30
    A0  = (p1*C2 - p2*C1) >> 30 ; A1 = (p2*C0 - p0*C2) >> 30 ; A2 = (p0*C1 - p1*C0) >> 30   -> Q16.16
    H   = [n0 n1 n2 A0 A1 A2]     (scales 2^30,2^30,2^30,2^16,2^16,2^16)
    S_k += H_i*H_j   for (i<=j) in row-major order  k = 0..20   (exact, 80-bit accumulators)
    S_21+i -= H_i*pd2             i = 0..5                       (= sum H_i * h, h = -pd2)

Float value of S_k = S_k / 2^(scale_i + scale_j)  (pd2 scale = 24).

## UDP protocol (board 192.168.137.234 port 1234, replies go to the sender)

Every payload is a list of 32-bit big-endian words; the first word is a header `0xA5 | type | count16`.
Payloads must stay <= 1172 bytes (WSL2 MTU 1200 => bigger datagrams get IP-fragmented and the board drops fragments).

| type | count | followed by | reply |
|---|---|---|---|
| 0x01 BEGIN | iteration id | 21 words: R (9, row-major), R_LI (9), t_LI (3). Clears the sums. | none |
| 0x02 POINTS | N points (<= 41) | 7 words per point: pb (3), n (3), pd2 (1) | none |
| 0x03 END | 0 | nothing | 85 words: see below |

Reply to END (340 bytes):

    w0 = 0xA5 03 <iteration id>
    w1 = number of points processed since BEGIN
    w2 = bad-header count (16 b) << 16 | FIFO-overflow flag
    w3 = 0x48544831 ("HTH1", format version)
    w4.. = S_0..S_26, each as 96-bit sign-extended two's complement, 3 words, most significant first

The PC checks w0 (id), w1 (= points it sent) and w2 (= 0); if any is wrong, it re-sends the whole iteration.
