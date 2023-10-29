import arena
import cv2
from dt_apriltags import Detector
import numpy as np
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import sys
sys.path.append('.')
import RTIMU
import os.path

scene = arena.Scene(host="arenaxr.org", scene="453lab4")

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
print("IMU Name: " + imu.IMUName())
if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(False)

detector = Detector(families='tag36h11',
                    nthreads=4,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
picam = Picamera2()
config = picam.create_video_configuration({"size": (1920, 1080)})
picam.configure(config)

apriltag = arena.Entity(
    object_id='my-tag',
    position=(0, 0, 0),
    rotation=(0, 0, 1, 0),
    parent='my-camera'
)
handcam = arena.Entity(
    object_id='my-handcam',
    position=(0, 0, 0),
    rotation=(0, 0, 0, 1),
    parent='my-tag'
)
hand = arena.Entity(
    object_id='my-hand',
    position=(0, 0, 0),
    rotation=(0, np.sqrt(2) / 2, 0, np.sqrt(2) / 2),
    parent='my-handcam'
)
imu_orig = arena.Entity(
    object_id='my-imu-orig',
    position=(0, 0, 0),
    rotation=(0, 0, 0, 1),
    parent='my-hand'
)
smoothhand = arena.Box(
    object_id='my-smoothhand',
    width=.1, height=.1, depth=2,
    position=(0, 0, 0),
    rotation=(0, 0, 0, 1),
    parent='my-imu-orig'
)


def imuquat2arena(imuquat):
    return ((-imuquat[2], -imuquat[3], imuquat[1], imuquat[0]))

def arenaquat_conj(arenaquat):
    return(-arenaquat[0], -arenaquat[1], -arenaquat[2], arenaquat[3])


mtx_string = open("mtx.txt", "r").read()

cleaned = mtx_string[1:-1]
rows = cleaned.split('\n')

intrinsic_mtx = []
for row in rows:
    row = row.strip()[1:-1]
    cols = row.split()
    intrinsic_mtx.append(cols)

cam_mtx = None
fx = float(intrinsic_mtx[0][0])
fy = float(intrinsic_mtx[1][1])
cx = float(intrinsic_mtx[0][2])
cy = float(intrinsic_mtx[1][2])
cam_params = [fx, fy, cx, cy]
print(fx, fy, cx, cy)

tag_size = 0.096 #meters

@scene.run_once
def scene_init():
    scene.add_object(hand)
    scene.add_object(apriltag)
    scene.add_object(handcam)
    scene.add_object(imu_orig)
    scene.add_object(smoothhand)
    print("Objects Added")

@scene.run_forever(interval_ms=1000)
def cam_updates():
    picam.capture_file("capture.jpg")
    img = cv2.imread(cv2.samples.findFile("capture.jpg"))
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(img_gray, 
            estimate_tag_pose=True, 
            camera_params=cam_params, 
            tag_size=tag_size)
    for tag in tags:
        if (tag.tag_id == 1):
            t = tag.pose_t
            t_arena = arena.Position(-t[0][0]+0.5, -t[1][0]-0.5, -t[2][0]-0.5)

            R = tag.pose_R
            R_euler = Rotation.from_matrix(R.T).as_euler('zyx', degrees=True)
            R_arena = arena.Rotation(-R_euler[0], -R_euler[1], -R_euler[2])
            
            hand.update_attributes(rotation=R_arena, position=t_arena)
            scene.update_object(hand)

@scene.run_forever(interval_ms=25)
def imu_updates():
    imu.IMURead()
    R_quat = imuquat2arena(imu.getIMUData()['fusionQPose'])
    smoothhand.update_attributes(rotation=R_quat)
    scene.update_object(smoothhand)
    # R_origin_quat = arenaquat_conj(R_quat)
    # imu_orig.update_attributes(rotation=R_origin_quat)
    # scene.update_object(imu_orig)

picam.start()

scene.run_tasks()

