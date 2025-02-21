## AI-Toolbox - Motion Utilities - Qtm2Osc

### Summary

Qtm2Osc is a small Python-based command line tool for receiving motion capture data from the [Qualisys Track Manager](https://www.qualisys.com/software/qualisys-track-manager/) software and forwarding this data as [OSC](https://en.wikipedia.org/wiki/Open_Sound_Control) messages. 

### Installation

The software runs within the *qtm* anaconda environment. This environment has to be setup beforehand.  Instructions how to setup the *qtm* environment are available as part of the [installation documentation ](https://github.com/bisnad/AIToolbox/tree/main/Installers) in the [AI Toolbox github repository](https://github.com/bisnad/AIToolbox). 

The software can be downloaded by cloning the [MotionUtilities Github repository](https://github.com/bisnad/MotionUtilities). After cloning, the software is located in the MotionUtilities / Qtm2Osc directory.

### Directory Structure

- Qtm2Osc (contains software specific python scripts)

### Usage

#### Start

The Qualisys Track Manager software should be started before the Qtm2Osc. Qtm2Osccan be started either by double clicking the qtm_to_osc.bat (Windows) or qtm_to_osc.sh (MacOS) shell scripts or by typing the following commands into the Anaconda terminal:

```
conda activate qtm
cd Qtm2Osc
python qtm_to_osc.py
```

#### Functionality

The software receives marker positions, skeleton joint positions and joint rotations of a single performer from the Qualisys Track Manager and forwards this data as OSC messages to two different OSC addresses. This information originates either from a performer that is captured in real-time or by playing back a motion capture recording. The marker and joint positions and rotations are defined in a global coordinate system. The position information is automatically scaled to the same units (cm). 

### OSC Communication

The software sends OSC messages representing the marker positions, joint positions and joint rotations of a single performer. Each message contains all the marker positions, joint positions, and joint rotations grouped together. In the OSC messages described below, M represents the number of markers and N represents the number of joints.

The following OSC messages are sent by the software:

- marker positions as list of 3D vectors  in world coordinates `/mocap/marker/pos <float m1x> <float m1y> <float m1z> .... <float mMx> <float mMy> <float mMz>` 
- joint positions as list of 3D vectors in world coordinates: `/mocap/joint/pos <float j1x> <float j1y> <float j1z> .... <float jNx> <float jNy> <float jNz>` 
- joint rotations as list of Quaternions in world coordinates: `/mocap/joint/rot <float j1w> <float j1x> <float j1y> <float j1z> .... <float jNw> <float jNx> <float jNy> <float jNz>` 

By default, the software sends the OSC messages to the following two IP addresses and ports: IP 2.0.0.31 port 23456, IP 127.0.0.1 port 23456. To change these addresses and ports, the following source code in the file qtm_to_osc.py has to be modified:

```
osc_tracking_send_address = "2.0.0.31"
osc_tracking_send_port = 23456
osc_tracking_send_address2 = "127.0.0.1"
osc_tracking_send_port2 = 23456
```

In this code, the strings "2.0.0.31" and "127.0.0.1" needs to be replaced to specify different IP addresses and the numbers 23456 have to be replaced to specify different ports.

### Limitations

Qtm2Osc only works with motion capture data from a single performer.
