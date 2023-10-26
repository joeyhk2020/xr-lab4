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

tag = arena.Entity(
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


picam.start()
