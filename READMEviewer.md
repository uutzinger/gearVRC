# gearVRC 

**gearVRCViewer.py** renders the controller with OpenGL and illustrates it's current pose and user activity.

The viewer can be run on an other computer as it connects to the client via **ZMQ**. To simulate the gearVRC you can run the **dummyZMQsender.py** and connect to localhost.

<img src="./assets/gearVRCviewer.jpg" alt="gearVRC Viewer" width="400" height="321">

## Usage
gearVRCViewer.py has the options listed below. You will need to run either the dummyZMQsender or the gearVRC program on a computer that can connect to the controller in order to see live updates. Please provide zmqport accordingly.

```
  -h, --help            show help message and exit
  -z <zmqport>, --zmq <zmqport>
                        port used by ZMQ, e.g. 'tcp://localhost:5556'
  -p, --position        Move sensor when position data is available, not recommended as you will experience run off (default off)
  -d, --debug           Debugging on or off, displays texture map (default off)
  -b, --buttons         Render button presses on or off (default on)
```