import numpy as np
import timeit
import cv2

np.random.seed(22)
img_size = [2048, 1536]
num_images_to_process = 100
images = np.random.randint(0,255, size=[num_images_to_process, img_size[0], img_size[1]], dtype='uint8')


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
    cv2.subtract(img, 0, img)
    #ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_TOZERO)
    M = cv2.moments(img)
    if M['m00'] != 0:
        com_x = M["m01"]/M["m00"]
        com_y = M["m10"]/M["m00"]
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

print("old_method ", process_image(images[0]), "new method ", process_image_cv(images[0]))

test_img = np.array(images[0]*255).astype('uint8')
print(np.min(cv2.subtract(test_img, 1000)))
print(np.max(cv2.subtract(test_img, 10)))