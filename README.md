# LaserPointLock

Initial code written by Kyle Gordon. 

Edited and improved by Garrison Linn

## Update notes: 
- I squashed many bugs, mostly trivial errors (although not always trivial to find), which had prevented locking. 
- Communication with motors had not been tested to my knowledge, and these motors are quite finiky it turns out. So, the motor code was made more robust by catching errors and allowing multiple attempts at commands. 
- Also, in several places try, except statements were added to handle issues.
- Calibration was largely rewritten to allow couplings between x-y, which also makes the code more robust, because the camera x and piezo x may truly  be along different dimensions, i.e. camera x may be piezo y. Also, calibration was updated to use the actual voltage of the motors, rather than assuming the voltages of the motors are the calibration voltages, because the motors do not always go to the correct set voltage, errors can be as large as ~0.5 V; so rather than try over and over to get within an accuracy of 0.2 V, the code just accepts what we get, and since we use the actual voltage of the motors, it is fine to not have great accuracy during calibration. 
- I also added wait times between setting voltages and calculating COM, because the piezo motors are not great. 
- I updated the Take_image functions to average a couple of frames and to average a couple of COM calculations, because we want to take out some of the noise. 
- Also, the locking algorithm was improved. First, I made sure to not allow updates of the voltages outside of the ranges of the piezo motors, i.e. 0-150 V only allowed. Second, I tracked how often the update lock tries to apply voltages that are out of these bounds, and I allow the algorithm to reset the piezo to the start voltage--75.0 V, skip the update, and try again. If the attempted update voltages go out of bounds several times in a row within 60 seconds, then the code breaks the lock algorithm and returns to the measure algorithm. This is nice, because sometimes the COM jumps, perhaps from a bad frame grab or maybe a short term fluctuation, but resetting and skipping the update does often work, allowing the pointing to remain locked for longer. 

There is still much to do; so look for future updates to the code... but this code now works with Mightex SCE-B013-U cameras and Thorlabs MDT_693A piezo controllers to stabalize pointing of a laser beam. And the code has been confirmed to maintain the locking of the pointing for >1 day. 


## Start up tips:
- Drivers must be installed from manufacturers. The windows 7 drivers for the cameras do work on windows 10. Then update drivers from device manager by explicitly pointing to installed drivers. 
- Make sure to download libusb, and make sure to install the drivers for the devices via zadig. If you already have a driver downloaded, then you will have to select the setting in zadig to show all devices not just ones without drivers, then overwrite the driver installation with zadig. This is important so that python can communicate with devices. 