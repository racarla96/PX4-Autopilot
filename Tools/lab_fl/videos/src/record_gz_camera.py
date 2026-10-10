import sys, time, math, json, subprocess
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean
out = sys.argv[1]; maxn = int(sys.argv[2])
CAM = (5.0, -14.0, 3.0)
drone = [5.0, 0.0, 3.0]; yaw = math.pi/2; pitch = 0.0
ff = None; n = 0
node = Node()
def on_pose(m):
    for p in m.pose:
        if p.name == 'x500_0':
            drone[:] = [p.position.x, p.position.y, p.position.z]
def cb(m):
    global ff, n, yaw, pitch
    if ff is None:
        ff = subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','%dx%d'%(m.width,m.height),
                               '-r','20','-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','20',out], stdin=subprocess.PIPE)
    ff.stdin.write(m.data); n += 1

print(node.subscribe(Pose_V, '/world/default/dynamic_pose/info', on_pose), node.subscribe(Image, '/viewcam2', cb), flush=True)
t0 = time.time()
while n < maxn: time.sleep(0.1)
node.unsubscribe('/viewcam2'); time.sleep(0.3)
if ff: ff.stdin.close(); ff.wait()
print('frames', n)
