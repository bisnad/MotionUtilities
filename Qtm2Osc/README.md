## Qtm2Osc

### Summary

Qtm2Osc is a small Python-based command line tool for receiving motion capture data from the [Qualisys Track Manager](https://www.qualisys.com/software/qualisys-track-manager/) software and forwarding this data as [OSC](https://en.wikipedia.org/wiki/Open_Sound_Control) messages. 

### Features

Qtm2Osc receives and sends 3D mocap marker positions, 3D skeleton joint positions and joint rotations as quaternions, all in a global coordinate system. 
Qtm2Osc sends mocap data to two different OSC addresses.
Qtm2Osc scales the marker and joint positions automatically to the same units (cm).

### Limitations

Qtm2Osc only works with motion capture data from a single performer.
Qtm2Osc doesn't provide a GUI and has to be configured by changing its source code. 

The OSC messages whose values the recorder visualises and the network port the recorder listens to must be set in the source code and can not be modified through a GUI.
The visualisation of the sensor values as timeseries is very slow. 

### OSC Communication

Qtm2Osc sends the following OSC messages:

osc_send_address_marker_positions = "/mocap/marker/pos"
osc_send_address_joint_positions = "/mocap/joint/pos"
osc_send_address_joint_rotations = "/mocap/joint/rot"

