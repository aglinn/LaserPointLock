"""
The problem I see so far in the code is that the states are buried deep inside a statemachine, which is slave to the
high level "app" class; however, every state needs access to the devices. Also, it is difficult to disentangle the state
machine from the GUI. So, I am thinking we need something in the middle of the devices, the statemachine, and the GUI.
I am thinking that a databroker is the right middle person. So, the devices
"""