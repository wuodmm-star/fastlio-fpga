"""Offline video frames: replays a relay_dump file into RViz (running on a virtual display) one scan at a time and
grabs the screen after each scan, straight into an mp4.  render.py <dump> <node log> <out.mp4> [first last step]"""
import io, os, re, struct, subprocess, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from PIL import Image, ImageDraw, ImageFont
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster

dump, logf, out = sys.argv[1:4]
first, last, step = (int(a) for a in (sys.argv[4:7] + ["0", "1000000", "1"][len(sys.argv) - 4:]))
SETTLE = float(os.environ.get("SETTLE", "0.5")); FONT = os.environ.get("CJK_FONT")
F1, F2, F3 = (ImageFont.truetype(FONT, z, index=2) for z in (28, 24, 20))
TITLE = os.environ.get("TITLE", "FAST-LIO：地图和近邻搜索由 FPGA 板子完成（Cyclone IV EP4CE10，无处理器核）")
CROP = (26, 62, 1182, 938)            # the 3-D view: below the menu and tool bars, between the dock handles
def grab():                           # one frame of the virtual display
    raw = subprocess.run(["import", "-window", "root", "bmp:-"], capture_output=True).stdout
    return Image.open(io.BytesIO(raw)).convert("RGB").crop(CROP)
W, H = CROP[2] - CROP[0], CROP[3] - CROP[1]; H -= H % 2
CROP = (CROP[0], CROP[1], CROP[2], CROP[1] + H)
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", os.environ.get("FPS", "30"),
                       "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "20", "-pix_fmt", "yuv420p", "-threads", "2", out], stdin=subprocess.PIPE)
frames = []; odo = None; data = open(dump, "rb").read(); off = 0
while off + 5 <= len(data):
    kind, ln = struct.unpack_from("<BI", data, off); off += 5
    if off + ln > len(data): break
    body = data[off + 8: off + ln]; off += ln
    if kind == 2: odo = struct.unpack("<7d", body)
    elif kind == 1 and odo is not None:
        # this dataset's world frame has z pointing down (VN100 IMU): turn it upright (180 deg about x) for viewing
        p = np.frombuffer(body, "<f4").reshape(-1, 4).copy(); p[:, 1:3] *= -1
        frames.append((p, (odo[0], -odo[1], -odo[2]) + tuple(odo[3:])))
print(len(frames), "scans in the dump", flush=True)
# statistics lines of the FPGA client, keyed by scan number
stats = {}
for line in open(logf, errors="replace"):
    g = re.search(r"scan (\d+): FPGA (\d+) adds .*?, (\d+) searches \(([\d.]+) us each", line)
    if g: stats[int(g.group(1))] = (int(g.group(2)), int(g.group(3)), float(g.group(4)))
keys = sorted(stats)
def stat_at(k):                       # searches so far, interpolated between the log lines (every 300 scans)
    if not keys: return 0, 0.0
    hi = next((s for s in keys if s >= k), keys[-1]); lo = max([s for s in keys if s < hi] or [0])
    a = stats.get(lo, (0, 0, stats[hi][2])); b = stats[hi]
    w = min(1.0, (k - lo) / max(1, hi - lo))
    return int(a[1] + (b[1] - a[1]) * w), b[2]

rclpy.init(); n = Node("fpga_video")
pc = n.create_publisher(PointCloud2, "/cloud_registered", 5); pmap = n.create_publisher(PointCloud2, "/Laser_map", 2)
po = n.create_publisher(Odometry, "/Odometry", 5); pp = n.create_publisher(Path, "/path", 2); ptraj = n.create_publisher(PointCloud2, "/traj_cloud", 2); ppose = n.create_publisher(PointCloud2, "/pose_cloud", 2)
FIELDS = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, k in enumerate(("x", "y", "z", "intensity"))]
def cloud(a):
    m = PointCloud2(); m.header.frame_id = "camera_init"; m.header.stamp = n.get_clock().now().to_msg(); m.height = 1; m.width = len(a)
    m.fields = FIELDS; m.is_bigendian = False; m.point_step = 16; m.row_step = 16 * len(a); m.is_dense = True
    m.data = np.ascontiguousarray(a, "<f4").tobytes(); return m
tfb = TransformBroadcaster(n)
path = Path(); path.header.frame_id = "camera_init"
vox = {}                              # accumulated map: one point per 0.25 m voxel
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.01)
spin(2.0)
for k, (pts, o) in enumerate(frames):
    key = np.floor(pts[:, :3] / 0.25).astype(np.int32)
    _, idx = np.unique(key, axis=0, return_index=True)
    for i in idx: vox.setdefault(key[i].tobytes(), pts[i])
    ps = PoseStamped(); ps.header.frame_id = "camera_init"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = o[:3]
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = o[3:]
    path.poses.append(ps)
    if k < first or k > last or (k - first) % step: continue
    now = n.get_clock().now().to_msg()
    pmap.publish(cloud(np.array(list(vox.values()), "<f4")))
    pc.publish(cloud(pts))
    od = Odometry(); od.header.frame_id = "camera_init"; od.header.stamp = now; od.child_frame_id = "body"; od.pose.pose = ps.pose
    od.pose.covariance[0] = od.pose.covariance[7] = od.pose.covariance[14] = 1e-4; po.publish(od)
    tf = TransformStamped(); tf.header.frame_id = "camera_init"; tf.header.stamp = now; tf.child_frame_id = "body"
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = o[:3]
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = o[3:]
    tfb.sendTransform(tf)
    path.header.stamp = now; pp.publish(path)
    tr = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z, 0.0] for q in path.poses], "<f4")
    if len(tr) > 1:                       # dense points along the trajectory (every ~4 cm) so it reads as a line
        seg = np.linalg.norm(np.diff(tr[:, :3], axis=0), axis=1); d = np.concatenate([[0.0], np.cumsum(seg)])
        u = np.arange(0.0, d[-1], 0.04)
        tr = np.column_stack([np.interp(u, d, tr[:, i]) for i in range(3)] + [np.zeros(len(u))]).astype("<f4") if len(u) else tr
    ptraj.publish(cloud(tr)); ppose.publish(cloud(np.array([[o[0], o[1], o[2], 0.0]], "<f4")))
    ns, us = stat_at(k + 1)
    spin(SETTLE)
    img = grab()
    d = ImageDraw.Draw(img, "RGBA")
    d.rectangle([0, 0, img.width, 118], fill=(10, 14, 18, 190))
    d.text((24, 12), TITLE, font=F1, fill=(255, 214, 80))
    d.text((24, 50), f"第 {k + 1} / {len(frames)} 帧    飞行第 {(k + 1) / 10:.1f} 秒", font=F2, fill=(235, 240, 240))
    if keys: d.text((24, 82), f"板子已回答的查找：约 {ns:,} 次（每次 {us:.1f} 微秒，含网线往返）", font=F2, fill=(235, 240, 240))
    d.rectangle([0, img.height - 40, img.width, img.height], fill=(10, 14, 18, 190))
    d.text((24, img.height - 34), "白点：当前这一帧雷达    彩色点：累积的地图（按高度上色）    红线、青点：轨迹和当前位置    数据：NTU-VIRAL eee_03", font=F3, fill=(200, 210, 210))
    ff.stdin.write(img.tobytes())
    if k % 100 == 0: print("frame", k, "map points", len(vox), flush=True)
ff.stdin.close(); ff.wait()
print("RENDER_DONE", flush=True)
