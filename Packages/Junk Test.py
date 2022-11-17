import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0,np.pi,100)
y = np.sin(x)

fig, ax = plt.subplots(4,4,dpi=200, gridspec_kw={"hspace":0.5, "wspace":0.3})

ax[0,0].plot(x, y)
ax[1,0].plot(x, y)
ax[2,0].plot(x, y)
ax[3,0].plot(x, y)
ax[0,1].plot(x, y)
ax[1,1].plot(x, y)
ax[2,1].plot(x, y)
ax[3,1].plot(x, y)
ax[0,2].plot(x, y)
ax[1,2].plot(x, y)
ax[2,2].plot(x, y)
ax[3,2].plot(x, y)
ax[0,3].plot(x, y)
ax[1,3].plot(x, y)
ax[2,3].plot(x, y)
ax[3,3].plot(x, y)

ax[0,0].set_title('mot 1 x cam 1 x',fontsize=6)
ax[1,0].set_title('mot 1 x cam 1 x',fontsize=6)
ax[2,0].set_title('mot 1 x cam 1 x',fontsize=6)
ax[3,0].set_title('mot 1 x cam 1 x',fontsize=6)
ax[0,1].set_title('mot 1 x cam 1 x',fontsize=6)
ax[1,1].set_title('mot 1 x cam 1 x',fontsize=6)
ax[2,1].set_title('mot 1 x cam 1 x',fontsize=6)
ax[3,1].set_title('mot 1 x cam 1 x',fontsize=6)
ax[0,2].set_title('mot 1 x cam 1 x',fontsize=6)
ax[1,2].set_title('mot 1 x cam 1 x',fontsize=6)
ax[2,2].set_title('mot 1 x cam 1 x',fontsize=6)
ax[3,2].set_title('mot 1 x cam 1 x',fontsize=6)
ax[0,3].set_title('mot 1 x cam 1 x',fontsize=6)
ax[1,3].set_title('mot 1 x cam 1 x',fontsize=6)
ax[2,3].set_title('mot 1 x cam 1 x',fontsize=6)
ax[3,3].set_title('mot 1 x cam 1 x',fontsize=6)

ax[3,0].set_xlabel("Voltage",fontsize=6)
ax[3,0].set_ylabel("position (pixels)",fontsize=6)

ax[0,0].tick_params(axis='both', which='major', labelsize=6)
ax[1,0].tick_params(axis='both', which='major', labelsize=6)
ax[2,0].tick_params(axis='both', which='major', labelsize=6)
ax[3,0].tick_params(axis='both', which='major', labelsize=6)
ax[0,1].tick_params(axis='both', which='major', labelsize=6)
ax[1,1].tick_params(axis='both', which='major', labelsize=6)
ax[2,1].tick_params(axis='both', which='major', labelsize=6)
ax[3,1].tick_params(axis='both', which='major', labelsize=6)
ax[0,2].tick_params(axis='both', which='major', labelsize=6)
ax[1,2].tick_params(axis='both', which='major', labelsize=6)
ax[2,2].tick_params(axis='both', which='major', labelsize=6)
ax[3,2].tick_params(axis='both', which='major', labelsize=6)
ax[0,3].tick_params(axis='both', which='major', labelsize=6)
ax[1,3].tick_params(axis='both', which='major', labelsize=6)
ax[2,3].tick_params(axis='both', which='major', labelsize=6)
ax[3,3].tick_params(axis='both', which='major', labelsize=6)





plt.show()