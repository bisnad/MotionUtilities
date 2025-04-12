import numpy as np
import pathlib

from common import utils
from common import bvh_tools as bvh
from common import fbx_tools as fbx
from common import mocap_tools as mocap

config = { 
    "file_path": "../../../Data/Mocap/Xsens/Stocos/Duets/fbx_50hz/",
    "file_names": [ "Jason_Take5.fbx", "Sherise_Take5.fbx"]
    }

class MotionPlayer():
    def __init__(self, config):
        
        self.file_path = config["file_path"]
        self.file_names = config["file_names"]
        
        self.load(self.file_path, self.file_names)
        
    def load(self, file_path, file_names):
        
        mocap_tools = mocap.Mocap_Tools()
        
        self.all_mocap_data = []
        
        for file_name in file_names:
            
            mocap_data = None
            
            full_file_name = self.file_path + file_name
            
            print("full_file_name ", full_file_name)
        
            # load mocap file
            file_suffix = pathlib.Path(full_file_name).suffix
            
            print("file_suffix ", file_suffix)

            if(file_suffix == ".bvh" or file_suffix == ".BVH"):
                mocap_data = self.load_bvh(full_file_name)
            elif(file_suffix == ".fbx" or file_suffix == ".FBX"):
                mocap_data = self.load_fbx(full_file_name)
                
            # compute local joint rotations and world joint positions
            if(file_suffix == ".bvh" or file_suffix == ".BVH"):
                mocap_data["motion"]["rot_local"] = mocap_tools.euler_to_quat_bvh(mocap_data["motion"]["rot_local_euler"], mocap_data["rot_sequence"])
            elif(file_suffix == ".fbx" or file_suffix == ".FBX"):
                mocap_data["motion"]["rot_local"] = mocap_tools.euler_to_quat(mocap_data["motion"]["rot_local_euler"], mocap_data["rot_sequence"])
            
            mocap_data["motion"]["pos_world"], mocap_data["motion"]["rot_world"] = mocap_tools.local_to_world(mocap_data["motion"]["rot_local"], mocap_data["motion"]["pos_local"], mocap_data["skeleton"])
            
            
            if mocap_data is not None:
                self.all_mocap_data.append(mocap_data)
                
        # get mocap info from first file (assuming it is the same for the other files)

        self.fps = self.all_mocap_data[0]["frame_rate"]
        rot_sequence = self.all_mocap_data[0]["rot_sequence"]
        offsets = self.all_mocap_data[0]["skeleton"]["offsets"]
        pos_local = self.all_mocap_data[0]["motion"]["pos_local"]
        rot_local_euler = self.all_mocap_data[0]["motion"]["rot_local_euler"]
                
        # update start, end, and play position
        self.play_frame = 0
        self.start_play_frame = 0
        self.end_play_frame = min([ mocap_data["motion"]["pos_world"].shape[0] - 1 for mocap_data in self.all_mocap_data ])
        
    def load_bvh(self, file_name):
        # load mocap data
        bvh_tools = bvh.BVH_Tools()
        mocap_tools = mocap.Mocap_Tools()

        bvh_data = bvh_tools.load(file_name)
        return mocap_tools.bvh_to_mocap(bvh_data)

    def load_fbx(self, file_name):
        # load mocap data
        fbx_tools = fbx.FBX_Tools()
        mocap_tools = mocap.Mocap_Tools()
        
        fbx_data = fbx_tools.load(file_name)

        all_mocap_data = mocap_tools.fbx_to_mocap(fbx_data)
        return all_mocap_data[0] # use only mocap data of first skeleton
        
    def update(self):

        if self.play_frame >= self.end_play_frame:
            self.play_frame = self.start_play_frame
        
        self.play_frame += 1
    
    def get_skeleton_count(self):
        return len(self.all_mocap_data)
    
    def get_file_names(self):
        return self.file_names
        
    def get_fps(self):
        return self.fps
    
    def set_fps(self, fps):
        self.fps = fps
        
    def get_play_frame(self):
        return self.play_frame
        
    def set_play_frame(self, frame):

        if frame >= self.end_play_frame:
            frame = self.end_play_frame - 1
            
        if frame <= self.start_play_frame:
             frame = self.start_play_frame + 1
             
        self.play_frame = frame
        
    def get_start_play_frame(self):
        return self.start_play_frame
        
    def set_start_play_frame(self, frame):
        
        if frame >= self.end_play_frame:
            frame = self.end_play_frame - 1

        self.start_play_frame = frame
        
        if self.play_frame < self.start_play_frame:
            self.play_frame = self.start_play_frame
        
        #self.play_frame  = self.start_play_frame
    
    def get_end_play_frame(self):
        return self.end_play_frame
        
    def set_end_play_frame(self, frame):
        
        if frame <= self.start_play_frame:
             frame = self.start_play_frame + 1

        self.end_play_frame = frame
        
        if self.play_frame > self.end_play_frame:
            self.play_frame = self.end_play_frame
        
        #self.play_frame  = self.end_play_frame

    def get_skeleton(self):
        return self.all_mocap_data[0]["skeleton"]

    def get_poses(self, pose_feature):
        return [ mocap_data["motion"][pose_feature][self.play_frame, ...] for mocap_data in self.all_mocap_data ]
