# Enable using Python scripts in RoboDK on Linux

In order to be able to use Python scripts in RoboDK, first make sure that you have installed Python 3.4 and Python IDLE 3.4 or newer. 
This can be done using:

		sudo apt-get install python3.4 idle-python3.4

After that, download the contents of res folder (robodk.py and robolink.py) and place them into the Python3.4 directory. 
The default folder for this is "/usr/lib/python3.4". Then open Tools->Options->Python and set:

	Python folder path: /usr/bin
	Python run path:    /usr/bin/idle-python3.4

This procedure was tested on Ubuntu 14.04 LTS and RoboDK 1.0 downloaded from the official website.

** Do this at your own risk! **

The author reserves the right not to be responsible for the topicality, correctness, completeness or quality of the information provided. Liability claims regarding damage caused by the use of any information provided, including any kind of information which is incomplete or incorrect, will therefore be rejected. 
