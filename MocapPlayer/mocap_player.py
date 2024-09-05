
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
    "file_name": "data/mocap/zachary_music_improvisation.fbx",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "D:/Data/mocap/Daniel/Zed/fbx/daniel_zed_solo1.fbx",
    "fps": 30
    }
"""

"""
motion_player.config = { 
    "file_name": "D:/data/mocap/stocos/Solos/Canal_14-08-2023/fbx_50hz/Muriel_Embodied_Machine_variation.fbx",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "D:/data/mocap/stocos/Solos/Canal_14-08-2023/bvh_50hz/Muriel_Embodied_Machine_variation.bvh",
    "fps": 50
    }
"""

"""
motion_player.config = { 
    "file_name": "D:/Data/mocap/stocos/Duets/Amsterdam_2024/fbx_50hz/Sherise_Take3.fbx",
    "fps": 50
    }
"""


motion_player.config = { 
    "file_name": "D:/Data/mocap/stocos/Solos/MovementQualities/fbx_50hz_2/volume_fullbody_take1.fbx",
    "fps": 50
    }


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
motion_control.config["ip"] = "127.0.0.1"
motion_control.config["port"] = 9002

osc_control = motion_control.MotionControl(motion_control.config)


"""
Start Application
"""

osc_control.start()
gui.show()
app.exec_()

osc_control.stop()