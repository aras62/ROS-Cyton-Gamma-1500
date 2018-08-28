Files available in this folder are designed to facilitate the use of Cyton Gamma1500 robotic arm. The server files are the original ones published by the manufacturer ROBAI (www.robai.com). The client files are based on the examples published by the manufacturer and are slightly modified to be used in any appliucation 

To Use these codes you need the following services:

ROS (Hydro or more recent version)
ros_cyton_pkg (available in your arm's installation CD. an older version can be find here: https://code.google.com/p/cyton-ros-pkg/)

How To Use the Code:

This code contains two modules, Server and client. Server is responsible for running ros node as well as the cyton server. ros_cyton_server file has to be run along side other ROS nodes. The basic format of this code is available in ros_cyton_pkg that comes with the arm. This code allows basic manipulation of the arm. In the version in this package few funtionalities are added such as receiving feedback from joints. To add/remove functionalities to the code, the user is encouraged to read on cyton arm's API (see: http://outgoing.energid.info/docs/actin/html/index.html).

How the Code Works:

Server Side:

In order to use ROS as an interface to the arm you need to run two services:

1- Ros node: once you made the changes or copied the ros_cyton_server file use catkin_make to compile the code. Next, use the executable file generated to run the ros service (the executables can be found in /build/bin). For instance you can use the following line:

ros_cyton_server -i 192.168.1.80 -c 1500 (file name -i <ip address of roscore> -c <arms model gamma1500>)

2- Actin server: there are two ways to run Actin server. Easy but also computationally expensive method is using cytonViewer GUI. To do so, first run cytonViewer. Once passed the initial setup, on the main page, click on plugin option on the top and select Load plugin. Here load the plugin remoteCommandServerPlugin.ecp. Close the menu and activate the arm. At this stage any manipulation through the client side should alter the state of the arm.

Alternatively you can use command line and run Actin server. This method is prefered because computationally is lighter. To do so from the same folder type the following command:

./actinRT remoteCommandServerPlugin.ecp  cytonPlugin.ecp  cyton.ecz manipulationActionExecPlugin.ecp  -m 0 

Note that in the above code 3 plugins are loaded and cyton.ecz which contains the setup information for your arm. -m option defines how much memory should be preallocated. Default value is set to 2 GB but if your memory is scarce you can set this to 0 which does not have any major impact on the overall performance. There are also additional options which you can learn by only typing ./actinRT.

***NOTE*** there is a known bug using actinRT. Everytime this service is used for first time after turining on the arm, the service does not operate properly. In such a case you need to first run the cytonViewer GUI app and manually change the state of the arm. Once the arm start moving, you can close cytonViewer and run actinRT service again.

Another potential problem is permissions on the usb port. if the software cannot find the arm make sure to set permission for the usb port correctly. In addition, it is common that the USB port switches from one to another e.g. USB0 to USB1. This can be resolved by simply disconnecting and reconnecting the usb port.

Client side:

Client side code contains a class which includes functions to send and receive commands from ROS server. This class also contains a function designed for testing purposes. This function alows altering the state of the arm using keyboard in command prompt. Both methods of manipulations namely joint and end effector modes are available. 


I hope the above instructions be useful to the potential users.

Amir Rasouli

 



 
