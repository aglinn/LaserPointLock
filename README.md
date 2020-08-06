# LaserPointLock

Initial code written by Kyle Gordon. 

Edited and improved by Garrison Linn

## Update notes: 
Orriginal updates:
- I squashed many bugs, mostly trivial errors (although not always trivial to find), which had prevented locking. 
- Communication with motors had not been tested to my knowledge, and these motors are quite finiky it turns out. So, the motor code was made more robust by catching errors and allowing multiple attempts at commands. 
- Also, in several places try, except statements were added to handle issues.
- Calibration was largely rewritten to allow couplings between x-y, which also makes the code more robust, because the camera x and piezo x may truly  be along different dimensions, i.e. camera x may be piezo y. Also, calibration was updated to use the actual voltage of the motors, rather than assuming the voltages of the motors are the calibration voltages, because the motors do not always go to the correct set voltage, errors can be as large as ~0.5 V; so rather than try over and over to get within an accuracy of 0.2 V, the code just accepts what we get, and since we use the actual voltage of the motors, it is fine to not have great accuracy during calibration. 
- I also added wait times between setting voltages and calculating COM, because the piezo motors are not great. 
- I updated the Take_image functions to average a couple of frames and to average a couple of COM calculations, because we want to take out some of the noise. 
- Also, the locking algorithm was improved. First, I made sure to not allow updates of the voltages outside of the ranges of the piezo motors, i.e. 0-150 V only allowed. Second, I tracked how often the update lock tries to apply voltages that are out of these bounds, and I allow the algorithm to reset the piezo to the start voltage--75.0 V, skip the update, and try again. If the attempted update voltages go out of bounds several times in a row within 60 seconds, then the code breaks the lock algorithm and returns to the measure algorithm. This is nice, because sometimes the COM jumps, perhaps from a bad frame grab or maybe a short term fluctuation, but resetting and skipping the update does often work, allowing the pointing to remain locked for longer. 

8/6/2020 update notes:
- The Gui was updated and improved: 1) Added "Define New Home" button which defines a new home position, which is used for locking and a new state (alignment mode). Home positions are stored by date in the appropriately named folder. 2) Added "Align" button that enters alignment mode. 3) Added option under file menu to load old home positions. 4) added a text box that specifies how many frames to average before calculating a COM on the averaged frame, although it is recommended to keep this to 1 to maximize control loop bandwidth; however, if the user does not want to control the higher frequencies, then they could increase the averaging of frames to reduce aliasing the laser pointing. 5) enabled the text box on pointing view, which now sets the number of points for the COM and piezzos to control. 6) Added a color coded marker for the current home position: red unlocked/not aligned, green locked/aligned.7) Added feedback arrows to help the user during alignment.
- New State, Alignment Mode, added: This state is used by the user to align the beam manually. This should be done prior to locking to roughly restore the pointing of the two control mirrors. This state displays a green circular home icon when the COM coordinates are within 10V of the home position. At this point, alignment mode may be exited by either retoggling align button (returns to measure state) or clicking the lock button to immediately lock. Additionally, red arrows will appear to help the user understand which direction to steer the beam on each camera. 
- New Camera Engine: Kyle Gordon wrote a camera engine, which uses the dll for the Mightex camera, providing quicker interfacing with the Mightex Cameras. I then updated the MightexCamera object to use this Mightex Engine. I also added a few functions to the mightex engine, e.g. set exposure time and updated the get frame function to also grab the exposure time from the frame meta data. 
- Several other minor improvements were made. 

There is still much to do; so look for future updates to the code... but this code now works with Mightex SCE-B013-U cameras and Thorlabs MDT_693A piezo controllers to stabalize pointing of a laser beam. And the code has been confirmed to maintain the locking of the pointing for >1 day. 


## Start up tips:
- Drivers must be installed from manufacturers. The windows 7 drivers for the cameras do work on windows 10. Then update drivers from device manager by explicitly pointing to installed drivers. 
- Make sure to download libusb, and make sure to install the drivers for the devices via zadig. If you already have a driver downloaded, then you will have to select the setting in zadig to show all devices not just ones without drivers, then overwrite the driver installation with zadig. This is important so that python can communicate with devices. 