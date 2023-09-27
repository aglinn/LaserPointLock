# LaserPointLock
This is a custom software package to implement a laser pointing stabilization system that stabilizes both the angular 
pointing and the offset. Please see "Point Lock Section of Thesis" for a discussion of the optical system setup, 
theory of operation, organization of the code, and more. 

The hardware that works with this code is a Thorlabs MDT693B (which is somewhat backwards compatible with older
MDT693A models), with the MDT693B controllers controlling Polaris piezo mirrors—but presumably any compatible piezo 
mirror would work, and FLIR Blackfly S cameras (only tested on BFS-U3-31S4M-C). The above is tested and works—see 
characterization data in "Point Lock Section of Thesis." Additionally, the code is written to work with FLIR Boson's, 
which allows the stabilization of IR laser beams out to ~17 um. I have not tested this functionality as extensively and 
cannot speak to the effectiveness very well. Lastly, I have made the code to work with Mightex SCE-B013U cameras. 
Originally, this worked okay using the grab frame command from the SDK, but I recently updated the code to use the 
supposedly better/faster Frame Hooker method, and that seems to be quite buggy. For some reason, the cameras stop 
sending frames every now-and-then for random lengths of time. I am not sure why, it may be that the camera cannot be 
run this way on a thread that is not the main thread, but I am not too sure... So, I would stick with the Blackfly S 
cameras which are nicer anyway, but if you need to use Mightex SCE-B013U, you can do so with some amount of debugging 
required. 

As far as I know, this code only works on Windows platform!

The code is multithreaded using PyQt and is able to read the frames from the BlackflyS cameras at the maximum frame 
rates, process those frames at the same rate, but only applies an update every ~1/3 of the camera frame rate.

Please understand that this code was written by a solo graduate student--me--as my first coding assignment among many 
other assignments. That is, it is not as crisp as something developed by a team of engineers but if you need a custom
solution, it is a working system that can be improved upon. I welcome improvements, please submit pull requests, but 
as such I make no warranties or promises and cannot continue to maintain this code as I move on to other projects.

## Branches:
### Master: 
Sorry should rename. Anyway, this is the last working version of the code. Mightex Cameras do not work on this branch
at all. The code is written such that you MUST stablize both the angular pointing and the offset of the pointing——i.e.
use two cameras.
### update_mightex:
This branch was where I was updating the Mightex SCE-B013U cameras to work again with the multithreaded version of the 
code including using the frame hooker method. See caveat above. I also merged the add_single_camera_stabilization branch
into this branch, so on this branch, the user can stabilize a single camera (i.e. just the angular pointing or the 
offset if they wish). Honestly, this is the newest and best version of the code, but I am afraid to merge it into Master
because of a lack of abundant use to determine that it actually works well. I do believe the single camera stabilization
works, so if I were going to use this code, I would use this branch, but I would know I might have to do a bit of 
debugging. The GUI is also improved to show the frame rates and the code is better organized. 
### add_single_camera_stabilization:
Old should be deleted, but I won't do that just in case someone wants this branch.
## Start up tips:
See "Setup Notes." These are my notes from when I setup this software on a totally new computer. This was, 
unforuntately, many commits ago so some packages may be missing, but this should be pretty close to accurate. Note the 
spec-file.txt file contains the up to date conda environment for running the application, so at least that is up to 
date. 

Note, the drivers must be installed from manufacturers. The windows 7 drivers for the cameras do work on windows 10. 
Then update drivers from device manager by explicitly pointing to installed drivers.