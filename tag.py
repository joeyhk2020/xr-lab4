import arena
import cv2
from dt_apriltags import Detector
import numpy as np
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import time

scene = arena.Scene(host="arenaxr.org", scene="453lab4")

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
hand = arena.Box(
    object_id='my-hand',
    width=.1, height=.1, depth=.1,
    position=(.5,-.5,-.5),
    parent='my-camera'
)

detector = Detector(
    families='tag36h11',
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)
picam = Picamera2()
config = picam.create_video_configuration({"size": (1920, 1080)})
picam.configure(config)



# picam.start()
# time.sleep(0.1)
# picam.capture_file("capture.jpg")

# img = cv2.imread(cv2.samples.findFile("capture.jpg"))
# if img is None:
#     print("No capture found :(")

mtx_string = open("mtx.txt", "r").read()

cleaned = mtx_string[1:-1]
rows = cleaned.split('\n')

intrinsic_mtx = []
for row in rows:
    row = row.strip()[1:-1]
    cols = row.split()
    intrinsic_mtx.append(cols)



# print(rows)
cam_mtx = None
fx = float(intrinsic_mtx[0][0])
fy = float(intrinsic_mtx[1][1])
cx = float(intrinsic_mtx[0][2])
cy = float(intrinsic_mtx[1][2])
cam_params = [fx, fy, cx, cy]
print(fx, fy, cx, cy)

tag_size = 0.096 #meters

# img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# tags = detector.detect(img_gray, 
#             estimate_tag_pose=True, 
#             camera_params=cam_params, 
#             tag_size=tag_size)

# print(tags)
###############################################

@scene.run_once
def scene_init():

    scene.add_object(hand)
    scene.add_object(apriltag)
    scene.add_object(handcam)
    print("OBJECTS ADDED")

picam.start()
@scene.run_forever(interval_ms=50)
def main():
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
            t_arena = arena.Position(-t[0][0], -t[1][0], -t[2][0])

            R = tag.pose_R
            R_euler = Rotation.from_matrix(R.T).as_euler('zyx', degrees=True)
            R_arena = arena.Rotation(-R_euler[0], -R_euler[1], -R_euler[2])
            
            hand.update_attributes(rotation=R_arena, position=t_arena)
            scene.update_object(hand)

            print("=======")
            print(apriltag.data.position)
            print(hand.data.position)
            
            

            # print(Rotation.from_matrix(R).as_quat())
            # print(t[0], t[1], t[2])

scene.run_tasks()


