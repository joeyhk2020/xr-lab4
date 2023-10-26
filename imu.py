import arena
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

hand = arena.Box(
    object_id='my-hand',
    width=.1, height=.1, depth=2,
    position=(.5,-.5,-.5),
    parent='my-camera'
)


def imuquat2arena(imuquat):
    return ((-imuquat[2], -imuquat[3], imuquat[1], imuquat[0]))

