import cv2
from pathlib import Path
import os
import numpy as np
import struct

def create_video_file():
    print(os.getcwd())

    new_directory_path = 'Data/' + str(np.datetime64('today', 'D'))
    Path(new_directory_path).mkdir(parents=True, exist_ok=True)
    folders = 1
    for _, dirnames, _ in os.walk(new_directory_path):
        folders += len(dirnames)
    save_data_directory = new_directory_path +'/acquisition_' +str(folders)+'_unlocked'
    Path(save_data_directory).mkdir(parents=True, exist_ok=False)
    save_dir = os.getcwd()+'/'+save_data_directory+"/"

    fourcc_in = cv2.VideoWriter_fourcc(*'FFV1')

    cam1_video_file = cv2.VideoWriter(save_dir+'cam1.avi', fourcc_in, 25, (640,480),
                                                       False)
    """cam1_video_file = cv2.VideoWriter('cam1.avi', fourcc_in, 25, (640,480),
                                                       False)"""
    print(cam1_video_file.isOpened())
    print(save_dir+'cam1.avi')

    """for _, dirnames, _ in os.walk(new_directory_path):
        print(dirnames)"""

    frame = np.zeros((480,640,3), dtype=np.uint8)
    cam1_video_file.write(frame)
    cam1_video_file.release()

    inputVideo = cv2.VideoCapture(save_dir+'cam1.avi')
    print(inputVideo.isOpened())
    fourcc = int(inputVideo.get(cv2.CAP_PROP_FOURCC))

    print("FOURCC is '%s'" % struct.pack("<I", fourcc), struct.pack("<I", fourcc_in))
    return

save_dir = os.getcwd()[0:-7]
save_dir += 'Data/...'
print(save_dir)

cap = cv2.VideoCapture(save_dir+'cam1.avi')
while (cap.isOpened()):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
print("FOURCC is '%s'" % struct.pack("<I", fourcc))