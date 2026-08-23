
import os, sys, time, subprocess

import motion_player 
import motion_sender
import motion_control
import motion_gui

"""
Setup Motion Player
"""

"""
motion_player.config = { 
    "file_name": "data/mocap/Take_3_50fps_crop.fbx",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "E:/Data/mocap/stocos/Solos/MovementQualities/fbx_50hz/staccato_rightarm_take1.fbx",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "../../../Data/Mocap/Captury/MotionBank/Solos/bvh_50hz/amber_movement_qualities.bvh",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "../../../Data/Mocap/Zed/Daniel/Solos/fbx_30hz/daniel_zed_solo2.fbx",
    "fps": 30
    }
"""

"""
motion_player.config = { 
    "file_name": "../../../Data/Mocap/Qualisys/Stocos/Solos/fbx_50hz/polytopia_fullbody_take1.fbx",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "../../../Data/Mocap/Xsens/Stocos/Solos/fbx_50hz/Muriel_Embodied_Machine_variation.fbx",
    "fps": 50
    }

"""

"""
motion_player.config = { 
    "file_name": "../../../Data/Mocap/Xsens/Stocos/Duets/fbx_50hz/Jason_Take3.fbx",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "../../../Data/Mocap/Xsens/Stocos/Duets/fbx_50hz/Jason_Sherise_Take5.fbx",
    "fps": 50
    }
"""


motion_player.config = { 
    "file_name": "../../../Data/Mocap/Xsens/Stocos/Solos/npz_60hz/Muriel_Embodied_Machine_variation.npz",
    "skel_parents": [ -1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 13, 14, 15, 12, 17, 18, 19, 12, 21],
    "fps": 60
    }


"""
motion_player.config = { 
    "file_name": "C:/Users/dbisig/Projects/IntuitionMachine/Data/Mocap/Classes/004_Smelling_ImprovisationAroundRangeOfArticulations_RicardoVasquezAllen.npz",
    "skel_parents": [-1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 13, 14, 15, 12, 17, 18, 19, 12, 21],
    "fps": 60
    }
"""

"""
motion_player.config = { 
    "file_name": "E:/Data/mocap/Yurika/Mediapipe/Classes/Yurika_Test_Mediapipe_realtime.npz",
    "skel_parents": [-1,0,1,2,0,4,5,3,6,0,0,0,0,11,12,13,14,15,20,15,16,15,20,0,0,23,24,25,26,27,28,27,28],
    "fps": 30
    }
"""

"""
motion_player.config = { 
    "file_name": "../../../Data/Mocap/Pose3D/Stocos/Solos/Stocos_DoubleBind_MediaPipe.fbx",
    "fps": 30
    }
"""

player = motion_player.MotionPlayer(motion_player.config)

"""
OSC Sender
"""

motion_sender.config["ip"] = "127.0.0.1"
motion_sender.config["port"] = 9007

osc_sender = motion_sender.OscSender(motion_sender.config)

"""
Setup Motion GUI
"""

from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pathlib import Path

motion_gui.config["player"] = player
motion_gui.config["sender"] = osc_sender
motion_gui.config["view_scale"] = 1.0
#motion_gui.config["view_scale"] = 100.0

app = QtWidgets.QApplication(sys.argv)
gui = motion_gui.MotionGui(motion_gui.config)

# set close event
def closeEvent():
    QtWidgets.QApplication.quit()
app.lastWindowClosed.connect(closeEvent) # myExitHandler is a callable

"""
OSC Control
"""

motion_control.config["gui"] = gui
motion_control.config["ip"] = "0.0.0.0"
motion_control.config["port"] = 9002

osc_control = motion_control.MotionControl(motion_control.config)


"""
Start Application
"""

osc_control.start()
gui.show()
app.exec_()

osc_control.stop()