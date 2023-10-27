import arena
import cv2
from dt_apriltags import Detector
import numpy as np
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import time


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
hand = arena.Box(
    object_id='my-hand',
    width=.1, height=.1, depth=2,
    position=(0, 0, 0),
    rotation=(0, np.sqrt(2) / 2, 0, np.sqrt(2) / 2),
    parent='my-handcam'
)

picam.start()
time.sleep(2)
picam.capture_file("Test.jpg")
