import tkinter

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import numpy as np

from test_comm import Camera
import time

c = Camera(decimation=2)
img = c.get_frame()

root = tkinter.Tk()
root.wm_title("Mightex Camera View")

fig = Figure(figsize=(5, 4), dpi=100)
t = np.arange(0, 3, 0.01)
ax = fig.add_subplot(111)
img_display = ax.imshow(img, clim=(0, 256))
vert_line = ax.axvline(157)
horz_line = ax.axhline(153)
vert_line_com = ax.axvline(img.shape[1]//2, color='r')
horz_line_com = ax.axhline(img.shape[0]//2, color='r')

# TK Frames
top_frame = tkinter.Frame(root)
bottom_frame = tkinter.Frame(root)
top_frame.pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=True)
bottom_frame.pack(side=tkinter.BOTTOM)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack(in_=top_frame, side=tkinter.TOP, fill=tkinter.BOTH, expand=1)
bg_cache = fig.canvas.copy_from_bbox(ax.bbox)

#toolbar = NavigationToolbar2Tk(canvas, root)
#toolbar.update()
#canvas.get_tk_widget().pack(in_=top_frame, side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

def on_key_press(event):
    print("you pressed {}".format(event.key))
    key_press_handler(event, canvas, toolbar)

canvas.mpl_connect("key_press_event", on_key_press)

X, Y = np.meshgrid(np.arange(img.shape[0]), np.arange(img.shape[1]))
X = X.T
Y = Y.T
def find_com(img, X, Y):
    temp = img
    temp[temp < 5] = 0
    sum_x = np.sum(np.sum(img*X))
    sum_y = np.sum(np.sum(img*Y))
    return sum_y/np.sum(np.sum(img)), sum_x/np.sum(np.sum(img))

def update():
    start_time = time.time()
    img = c.get_frame()
    print("Max pixel value = %d" % (np.max(np.max(img))))
    print("Mean pixel value = %.1f" % (np.mean(np.mean(img[img > 5]))))
    img_display.set_data(img)
    x, y = find_com(img, X, Y)
    print("Horiz Cursor: %.1f\tVert Cursor: %.1f" % (y, x))
    vert_line_com.set_xdata([x, x])
    horz_line_com.set_ydata([y, y])
    canvas.restore_region(bg_cache)
    ax.draw_artist(img_display)
    ax.draw_artist(horz_line)
    ax.draw_artist(vert_line)
    ax.draw_artist(horz_line_com)
    ax.draw_artist(vert_line_com)
    canvas.blit(ax.bbox)
    print("Time for frame update: %.3f (s)" % (time.time() - start_time))
    root.after(300, update)


def _quit():
    root.quit()
    root.destroy()

quit_button = tkinter.Button(master=root, text="Quit", command=_quit)
quit_button.pack(in_=bottom_frame, side=tkinter.BOTTOM)
#max_pixel = tkinter.Label(master=root, text="0")
#max_pixel.pack(in_=bottom_frame, side=tkinter.TOP)
exposure_label = tkinter.Label(master=root, text="Exposure Time (ms)")
exposure_label.pack(in_=bottom_frame, side=tkinter.LEFT)
exposure_time = tkinter.Entry(master=root)
exposure_time.pack(in_=bottom_frame, side=tkinter.LEFT)
exposure_time.delete(0, tkinter.END)
exposure_time.insert(0, "200")
def set_exp_time():
    try:
        exp_time = int(exposure_time.get())
    except ValueError:
        return
    c.set_exposure_time(exp_time)
set_exp_time_button = tkinter.Button(master=root, text="Set", command=set_exp_time)
set_exp_time_button.pack(in_=bottom_frame, side=tkinter.LEFT)

np.savetxt('test.csv', c.get_frame())

update()
tkinter.mainloop()
