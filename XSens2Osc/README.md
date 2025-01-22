## XSens2Osc

### Summary

XSens2Osc is a small C++-based software that receives motion capture data from the [XSens MVN](https://www.movella.com/products/motion-capture/mvn-analyze) Software and and forwards this data as [OSC](https://en.wikipedia.org/wiki/Open_Sound_Control) messages. 

### Features

XSens2Osc receives and sends any of the following mocap data: 3D joint positions, joint rotations as Euler angles and quaternions, linear and angular joint velocities and accelerations, raw inertial measurement unit sensor data.  
XSens2Osc sends each of this data as OSC message to a specified network address and port. 
XSens2Osc supports an arbitrary number of performers.

### Limitations

XSens2Osc doesn't provide a GUI for changing the network address and port address for sending OSC messages to. These settings need to be changed by editing the "config.json" file in the bin/data/ Folder.

### OSC Communication

XSens2Osc adds the identifier for the tracked performer as "skelID" to the address part of the OSC message.
Xens2Osc groups joint motion capture data together as follows:  j1_p1 j1_p2 ... j1_pD, j2_p1, j2_p2, ... j2_pD, ... , jN_p1, jN_p2, ... jN_pD- Here, j stands for joint, p for parameter, N for number of joints, and D for dimension of parameters.
XSens2Osc groups raw sensor motion capture data together as follows:  s1_p1 s1_p2 ... s1_pD, s2_p1, s2_p2, ... s2_pD, ... , sN_p1, sN_p2, ... sN_pD- Here, s stands for sensor, p for parameter, N for number of sensors , and D for dimension of parameters.

joint positions as list of 3D vectors in world coordinates: `/skel/skelID/joint/pos_world <float j1x> <float j1y> <float j1z> .... <float jNx> <float jNy> <float jNz>` 

joint rotations as list of Euler angles in world coordinates: `/skel/skelID/joint/rot_world_euler <float j1x> <float j1y> <float j1z> .... <float jNx> <float jNy> <float jNz>` 

joint rotations as list of quaternions in world coordinates: `/skel/skelID/joint/rot_world_euler <float j1w> <float j1x> <float j1y> <float j1z> .... <float jNw> <float jNx> <float jNy> <float jNz>`

joint linear velocities as list of 3D vectors in world coordinates: `/skel/skelID/joint/lin_vel <float j1x> <float j1y> <float j1z> .... <float jNx> <float jNy> <float jNz>` 

joint linear accelerations as list of 3D vectors in world coordinates: `/skel/skelID/joint/lin_acc <float j1x> <float j1y> <float j1z> .... <float jNx> <float jNy> <float jNz>` 

joint angular velocities as list of Euler angles in world coordinates: `/skel/skelID/joint/rot_vel <float j1x> <float j1y> <float j1z> .... <float jNx> <float jNy> <float jNz>` 

tracking sensor identifiers as list of integers: `/skel/skelID/tracker/id <int s1> <int s2> .... <int sN>`  

tracking sensor rotations as list of quaternions in world coordinates: `/skel/skelID/tracker/rot_world <float s1w> <float s1x> <float s1y> <float s1z> .... <float sNw> <float sNx> <float sNy> <float sNz>` 

tracking sensor accelerations as list of Euler angles in world coordinates: `/skel/skelID/tracker/accel_world <float s1x> <float s1y> <float s1z> .... <float sNx> <float sNy> <float sNz>` 

tracking sensor accelerations as list of Euler angles in relative coordinates: `/skel/skelID/tracker/accel <float s1x> <float s1y> <float s1z> .... <float sNx> <float sNy> <float sNz>` 

tracking sensor angular velocities as list of Euler angles in relative coordinates: `/skel/skelID/tracker/angular_vel <float s1x> <float s1y> <float s1z> .... <float sNx> <float sNy> <float sNz>` 



/skel/skelID/tracker/magnet

