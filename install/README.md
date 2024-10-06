## Installation

This document describes the procedure for installing and running the tools in the Motion Utilities repository. 

This procedure is identical with the installation of all the other tools that form part of Premiere's AI/VR Toolbox, with the exception of pose estimation in the Motion Analysis repository.

The installation consists of the following two main steps:

- Download repository
- Setup a conda environment



### Download Repository

This repository employs the Git version control system. This system is mainly used to backup and track changes in source code files. There exist two methods for downloading this repository.

The simplest method is to 

To clone this repository, you need to install the Git client software. [Git](https://en.wikipedia.org/wiki/Git) is a version control system that is mostly used to backup and track changes in source code files. 





This repository contains a collection of small mostly python-based utilities for recording and playing back motion capture data.

The following utilities are available:

- **MocapPlayer** : a python-based tool for playing recorded motion capture data and sending joint data via OSC
- **MocapPlayer_old** : a previous version of the MocapPlayer, that is needed in combination with some other tools
- **MocapRecorder** : a python-based tool to record time-based sensor data
- **Qtm2Osc** : a python-based tool for converting the Qualisys native communication protocol to OSC
- **XSens2Osc**: a C++-based tool for converting the XSens native communication protocol to OSC