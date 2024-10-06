## Installation

This document describes the procedure for installing and running the tools in the Motion Utilities repository. 

This procedure is identical with the installation of all the other tools that form part of Premiere's AI/VR Toolbox, with the exception of pose estimation in the Motion Analysis repository.

The installation consists of the following three main steps:

- Install Cuda and Cudnn
- Download this repository
- Setup Conda environment

### Install Cuda and Cudnn

This step only applies to Windows and Linux computers that are equipped with an NVidia GPU. At the time of this writing, the versions guaranteed to work with the AI/VR Toolbox tools are Cuda 11.8 and Cudnn 8.7.0.84. Download the corresponding installers from Nvidia's website [here](https://developer.nvidia.com/cuda-toolkit) and [here](https://developer.nvidia.com/cudnn).

### Clone Repository

Copy this repository to your local computer either by downloading a zip archive from [here](https://github.com/bisnad/MotionUtilities/archive/refs/heads/main.zip) or by cloning it with the terminal command `git clone git@github.com:bisnad/MotionUtilities.git`.  

Now change into the install directory within the downloaded repository with the terminal command: `cd MotionUtilities/install`. 

### Setup Conda Environment

If the Conda package and environment management is not yet installed on your computer, download it either through [Anaconda](https://www.anaconda.com/download/success) or [Miniconda](https://www.anaconda.com/download/success#miniconda). 
Once Conda is available, add the conda-forge channel with the terminal command: `conda config --add channels conda-forge`

The easiest way to install the Conda environment required for running the Motion Utilities is to use one of several premade environments that have been exported as yml files:

- premiere-macos-cpu.yml (conda environment for MacOS)
- premiere-windows-cpu.yml (conda environment for Windows without Cuda Support)
- premiere-windows-gpu.yml (conda environment for Windows with Cuda Support)

Apart from the exported conda environments, this directory also contains versions of the Autodesk FBX python package for three different operating systems:

- fbx-2020.3.7-cp310-cp310-macosx_10_15_universal2.whl (MacOS FBX package)
- fbx-2020.3.7-cp310-cp310-manylinux1_x86_64.whl (Linux FBX package)
- fbx-2020.3.7-cp310-none-win_amd64.whl (Windows FBX package)

One of these packages will need to be installed as part of the manual installation process.

To install one of the exported environments, execute the following terminal command: `conda env create -f FILE_NAME.yml` where "FILE_NAME.yml" corresponds to one of the files listed above. Doing so will create an environment named "premiere".

Alternatively, the environment can be setup manually by first creating the "premiere" environment and then installing the required packages. To create  the "premiere" environment, execute the following terminal command:  `conda create --name premiere python=3.11`. Subsequently, activate this environment with the terminal command: `conda activate premiere` before installing any python packages. Finally, execute the sequence of commands described in the document "python-packages.txt" that is located in the install directory.



