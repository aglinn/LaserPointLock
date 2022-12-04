import numpy as np
import time
import timeit
import cv2
from scipy import ndimage
np.random.seed(22)
img_size = [2048, 1536]
num_images_to_process = 100
images = np.random.rand(num_images_to_process, img_size[0], img_size[1])

def process_image(img):
    Nx, Ny = img.shape
    x = np.arange(Nx)
    y = np.arange(Ny)
    img1_X_mesh, img1_Y_mesh = np.meshgrid(x, y, indexing='ij')
    w = np.divide(img, np.sum(img))
    com_x = np.sum(img1_X_mesh * w)
    com_y = np.sum(img1_Y_mesh * w)
    return com_x, com_y

def loop_process_image():
    global images
    for i in range(images.shape[0]):
        _, _ = process_image(images[i])

def process_image_cv(img):
        ret, thresh = cv2.threshold(img, 0.1, 255, 0)
        M = cv2.moments(thresh)
        if M['m00'] != 0:
            com_x = M["m10"]/M["m00"]
            com_y = M["m01"]/M["m00"]
            return com_x, com_y

def loop_process_image_cv():
    global images
    for i in range(images.shape[0]):
        _, _ = process_image_cv(images[i])

time_cv = timeit.timeit(loop_process_image_cv, number=1)
time = timeit.timeit(loop_process_image, number = 1)
print("Time to process 1 frame with open_cv is ", time_cv/num_images_to_process)
print("Time to process 1 frame with other is ", time/num_images_to_process)
print("(CV_time minus other time)/time is ", (time_cv - time)/time)

print(process_image_cv(np.zeros([10,10])))