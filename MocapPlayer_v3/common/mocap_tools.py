import pandas
import math
import numpy as np
import transforms3d as t3d
from scipy.spatial.transform import Rotation
from common import bvh_tools as bvh
from common import fbx_tools as fbx
import copy

class Mocap_Tools:

    def bvh_to_mocap(self, bvh_data):
        
        mocap_data = {}
        mocap_data["frame_rate"] = 1.0 / bvh_data.framerate
        
        root_channels = list(bvh_data.skeleton.values())[0]["channels"]
        
        # Dynamically extract rotation channels regardless of if there are 3 or 6 total
        if len(root_channels) == 6:
            rot_channels = root_channels[3:]
        elif len(root_channels) >= 3:
            # If 3 or more but not 6, look for the string "rotation" (ignoring case)
            rot_channels = [ch for ch in root_channels if "rotation" in ch.lower()]
            if len(rot_channels) == 0:
                # Fallback if names are strange
                rot_channels = root_channels[-3:]
        else:
            rot_channels = ["Xrotation", "Yrotation", "Zrotation"]
            
        rot_channel_names = ["Xrotation", "Yrotation", "Zrotation"]
        
        # Safely map them, defaulting to 0, 1, 2 if names are unconventional
        rot_sequence = []
        for rot_channel in rot_channels:
            if rot_channel in rot_channel_names:
                rot_sequence.append(rot_channel_names.index(rot_channel))
        
        # Failsafe sequence if none were parsed
        if len(rot_sequence) < 3:
            rot_sequence = [0, 1, 2]
            
        mocap_data["rot_sequence"] = rot_sequence
        
        self._create_skeleton_data(bvh_data, mocap_data)
        self._create_motion_data(bvh_data, mocap_data)
        
        if hasattr(bvh_data, 'times_per_joint'):
            mocap_data["motion"]["times"] = bvh_data.times_per_joint
        
        return mocap_data
    
    def fbx_to_mocap(self, fbx_data):
        
        all_motion_data = []

        for fbx_per_skel_data in fbx_data:
            
            skeleton = {}
            skeleton["root"] = fbx_per_skel_data.skeleton_root
            skeleton["joints"] = fbx_per_skel_data.skeleton_joints
            skeleton["parents"] = fbx_per_skel_data.skeleton_parents
            skeleton["children"] = fbx_per_skel_data.skeleton_children
            skeleton["offsets"] = fbx_per_skel_data.skeleton_joint_offsets
            
            motion = {}
            motion["pos_local"] = fbx_per_skel_data.motion_pos_local
            motion["rot_local_euler"] = fbx_per_skel_data.motion_rot_local_euler
            
            # Map index-based fbx timestamps to the joint names so they are easy to look up
            motion["times"] = {}
            for j_idx, j_name in enumerate(skeleton["joints"]):
                if hasattr(fbx_per_skel_data, 'motion_times') and j_idx in fbx_per_skel_data.motion_times:
                    motion["times"][j_name] = fbx_per_skel_data.motion_times[j_idx]
            
            motion_data = {}
            motion_data["frame_rate"] = fbx_per_skel_data.motion_frame_rate 
            motion_data["rot_sequence"] = fbx_per_skel_data.motion_rot_sequence
            motion_data["skeleton"] = skeleton
            motion_data["motion"] = motion
            
            all_motion_data.append(motion_data)
    
        return all_motion_data
    
    def pkl_to_mocap(self, pkl_data, skeleton_parents=None):
        import numpy as np
        import transforms3d as t3d
        from scipy.spatial.transform import Rotation
        
        def get_shortest_quat(v1, v2):
            """Calculates the shortest-arc quaternion to rotate vector v1 to v2 without twisting."""
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            if v1_norm < 1e-6 or v2_norm < 1e-6:
                return np.array([1.0, 0.0, 0.0, 0.0])
                
            v1 = v1 / v1_norm
            v2 = v2 / v2_norm
            dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
            
            if dot > 0.999999:
                return np.array([1.0, 0.0, 0.0, 0.0])
            elif dot < -0.999999:
                ortho = np.cross(np.array([1.0, 0.0, 0.0]), v1)
                if np.linalg.norm(ortho) < 1e-6:
                    ortho = np.cross(np.array([0.0, 1.0, 0.0]), v1)
                ortho = ortho / np.linalg.norm(ortho)
                return np.array([0.0, ortho[0], ortho[1], ortho[2]])
            
            cross = np.cross(v1, v2)
            q = np.array([1.0 + dot, cross[0], cross[1], cross[2]])
            return q / np.linalg.norm(q)

        sensor_ids = pkl_data["sensor_ids"]
        sensor_values = pkl_data["sensor_values"]
        time_stamps = pkl_data["time_stamps"]
        
        skel_frames = {}
        for s_id, s_val, t in zip(sensor_ids, sensor_values, time_stamps):
            parts = s_id.strip('/').split('/')
            if len(parts) >= 4 and parts[0] == "mocap" and parts[2] == "joint":
                skel_idx = int(parts[1])
                attr = parts[3]
                if skel_idx not in skel_frames: skel_frames[skel_idx] = {}
                if t not in skel_frames[skel_idx]: skel_frames[skel_idx][t] = {}
                skel_frames[skel_idx][t][attr] = s_val
                
        all_mocap_data = []
        
        for skel_idx, frames_dict in skel_frames.items():
            times = sorted(list(frames_dict.keys()))
            frame_count = len(times)
            frame_rate = 1.0 / np.mean(np.diff(times)) if frame_count > 1 else 30.0 
                
            if skeleton_parents is not None:
                num_joints = len(skeleton_parents)
            else:
                num_joints = 0
                for t in times:
                    if "pos_world" in frames_dict[t]:
                        num_joints = len(frames_dict[t]["pos_world"]) // 3
                        break
            if num_joints == 0: continue
                
            pos_world = np.zeros((frame_count, num_joints, 3))
            pos_local = np.zeros((frame_count, num_joints, 3))
            rot_world = np.zeros((frame_count, num_joints, 4))
            rot_local = np.zeros((frame_count, num_joints, 4))
            
            rot_world[:, :, 0] = 1.0 
            rot_local[:, :, 0] = 1.0
            has_rot = False
            
            for f_idx, t in enumerate(times):
                frame_data = frames_dict[t]
                if "pos_world" in frame_data and len(frame_data["pos_world"]) > 0:
                    pos_world[f_idx] = np.array(frame_data["pos_world"]).reshape(num_joints, 3)
                elif f_idx > 0: pos_world[f_idx] = pos_world[f_idx-1]
                    
                if "rot_world" in frame_data and len(frame_data["rot_world"]) > 0:
                    rot_world[f_idx] = np.array(frame_data["rot_world"]).reshape(num_joints, 4)
                    has_rot = True
                elif f_idx > 0: rot_world[f_idx] = rot_world[f_idx-1]
                    
            parents = skeleton_parents if skeleton_parents is not None else [-1] + [i-1 for i in range(1, num_joints)]
            children = [[] for _ in range(num_joints)]
            for i, p in enumerate(parents):
                if p != -1: children[p].append(i)
                
            # Build topological order so parent rotations are always solved before children
            topo_order = []
            queue = [i for i, p in enumerate(parents) if p == -1]
            while queue:
                curr = queue.pop(0)
                topo_order.append(curr)
                queue.extend(children[curr])
            
            offsets = np.zeros((num_joints, 3))
            
            if has_rot:
                # --- FULL MOCAP DATA (Pass-through) ---
                for i in range(num_joints):
                    p = parents[i]
                    if p == -1:
                        pos_local[:, i, :] = pos_world[:, i, :]
                        if frame_count > 0:
                            offsets[i] = pos_world[0, i].copy()
                            offsets[i, [0, 2]] = 0.0
                    else:
                        for f in range(frame_count):
                            diff_world = pos_world[f, i] - pos_world[f, p]
                            q_parent_inv = t3d.quaternions.qconjugate(rot_world[f, p])
                            pos_local[f, i] = t3d.quaternions.rotate_vector(diff_world, q_parent_inv)
                        if frame_count > 0: offsets[i] = np.mean(pos_local[:, i, :], axis=0)
                            
            else:
                # --- 3D POSITION ONLY (Topological Swing-Twist IK) ---
                if frame_count > 0:
                    # 1. Establish Rigid Rest Offsets
                    for i in range(num_joints):
                        p = parents[i]
                        if p == -1:
                            offsets[i] = pos_world[0, i].copy()
                            offsets[i, [0, 2]] = 0.0
                        else:
                            offsets[i] = pos_world[0, i] - pos_world[0, p]

                    # 2. Compute Topologically Sorted IK Rotations
                    for f in range(frame_count):
                        for i in topo_order:
                            p = parents[i]
                            child_list = children[i]
                            
                            if p == -1: # ROOT JOINT
                                if len(child_list) >= 2:
                                    # Stable Procrustes constraint to anchor the core body
                                    v_rest = np.array([offsets[c] for c in child_list])
                                    v_curr = np.array([pos_world[f, c] - pos_world[f, i] for c in child_list])
                                    valid_idx = np.linalg.norm(v_rest, axis=1) > 1e-6
                                    if sum(valid_idx) >= 2:
                                        rot, _ = Rotation.align_vectors(v_curr[valid_idx], v_rest[valid_idx])
                                        q_scipy = rot.as_quat()
                                        rot_world[f, i] = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])
                                    else:
                                        rot_world[f, i] = np.array([1.0, 0.0, 0.0, 0.0])
                                elif len(child_list) == 1:
                                    v_rest = offsets[child_list[0]]
                                    v_curr = pos_world[f, child_list[0]] - pos_world[f, i]
                                    rot_world[f, i] = get_shortest_quat(v_rest, v_curr)
                                else:
                                    rot_world[f, i] = np.array([1.0, 0.0, 0.0, 0.0])
                                    
                            else: # CHILD JOINTS (Limbs/Spine)
                                if len(child_list) == 0:
                                    rot_world[f, i] = rot_world[f, p]
                                else:
                                    # SWING LOGIC: Inherit parent twist, compute minimal swing to reach target
                                    c = child_list[0]
                                    v_rest = offsets[c]
                                    v_curr = pos_world[f, c] - pos_world[f, i]
                                    
                                    # Where parent rotation naturally places the bone
                                    v_expected = t3d.quaternions.rotate_vector(v_rest, rot_world[f, p])
                                    
                                    # Swing quaternion to snap exactly to tracked AI data without twisting
                                    q_swing = get_shortest_quat(v_expected, v_curr)
                                    rot_world[f, i] = t3d.quaternions.qmult(q_swing, rot_world[f, p])

                    # 3. Derive Standardized Local Data for FK Rendering
                    for f in range(frame_count):
                        for i in range(num_joints):
                            p = parents[i]
                            if p == -1:
                                rot_local[f, i] = rot_world[f, i]
                                pos_local[f, i] = pos_world[f, i]
                            else:
                                q_p_inv = t3d.quaternions.qconjugate(rot_world[f, p])
                                rot_local[f, i] = t3d.quaternions.qmult(q_p_inv, rot_world[f, i])
                                pos_local[f, i] = offsets[i]

            skeleton = {
                "root": f"joint_{0}" if num_joints > 0 else "root",
                "joints": [f"joint_{i}" for i in range(num_joints)],
                "parents": parents,
                "children": children,
                "offsets": offsets
            }
            
            all_mocap_data.append({
                "frame_rate": frame_rate,
                "rot_sequence": [0, 1, 2],
                "skeleton": skeleton,
                "motion": {
                    "times": {j: times for j in skeleton["joints"]},
                    "pos_world": pos_world,
                    "pos_local": pos_local,
                    "rot_world": rot_world,
                    "rot_local": rot_local
                }
            })
            
        return all_mocap_data

    def mocap_to_bvh(self, mocap_data):
        
        bvh_data = bvh.BVH_Data()
        bvh_data.framerate = 1.0 / mocap_data["frame_rate"]
        bvh_data.root_name = mocap_data["skeleton"]["root"]
        
        bvh_channel_names, bvh_channels = self._create_bvh_channel_names(mocap_data)
        bvh_data.channel_names = bvh_channel_names
        
        bvh_skeleton = self._create_bvh_skeleton(mocap_data, bvh_channel_names)
        bvh_data.skeleton = bvh_skeleton
        
        bvh_values = self._create_bvh_values(mocap_data, bvh_channel_names)
        bvh_data.values = bvh_values
        
        return bvh_data

    def mocap_to_fbx(self, mocap_data):

        all_fbx_data = []
        
        for skel_mocap_data in mocap_data:

            fbx_data = fbx.FBX_Mocap_Data()
            
            fbx_data.skeleton_root = skel_mocap_data["skeleton"]["root"]
            fbx_data.skeleton_joints = skel_mocap_data["skeleton"]["joints"]
            fbx_data.skeleton_parents = skel_mocap_data["skeleton"]["parents"]
            fbx_data.skeleton_children = skel_mocap_data["skeleton"]["children"]
            fbx_data.skeleton_joint_offsets = skel_mocap_data["skeleton"]["offsets"]
            
            fbx_data.motion_frame_rate = skel_mocap_data["frame_rate"]
            fbx_data.motion_rot_sequence = skel_mocap_data["rot_sequence"]

            fbx_data.motion_pos_local = skel_mocap_data["motion"]["pos_local"]
            fbx_data.motion_rot_local_euler = skel_mocap_data["motion"]["rot_local_euler"]

            # Reverse the map for exporting if variable times are present
            fbx_data.motion_times = {}
            if "times" in skel_mocap_data["motion"]:
                for j_idx, j_name in enumerate(skel_mocap_data["skeleton"]["joints"]):
                    if j_name in skel_mocap_data["motion"]["times"]:
                        fbx_data.motion_times[j_idx] = skel_mocap_data["motion"]["times"][j_name]
            
            all_fbx_data.append(fbx_data)
            
        return all_fbx_data

    def get_frame(self, mocap_data, frame_index):
        
        frame_mocap_data = copy.deepcopy(mocap_data)
        
        pos_local = frame_mocap_data["motion"]["pos_local"][frame_index:frame_index+1, ...]
        rot_local_euler = frame_mocap_data["motion"]["rot_local_euler"][frame_index:frame_index+1, ...]
        
        frame_mocap_data["motion"]["pos_local"] = pos_local
        frame_mocap_data["motion"]["rot_local_euler"] = rot_local_euler

        if "pos_world" in frame_mocap_data["motion"]:
            pos_world = frame_mocap_data["motion"]["pos_world"][frame_index:frame_index+1, ...]
            frame_mocap_data["motion"]["pos_world"] = pos_world
            
        if "rot_world" in frame_mocap_data["motion"]:
            rot_world = frame_mocap_data["motion"]["rot_world"][frame_index:frame_index+1, ...]
            frame_mocap_data["motion"]["rot_world"] = rot_world

        return frame_mocap_data

    def set_frame(self, mocap_data, frame_mocap_data, frame_index):
        
        mocap_data["motion"]["pos_local"][frame_index:frame_index+1, ...] = frame_mocap_data["motion"]["pos_local"]
        mocap_data["motion"]["rot_local_euler"][frame_index:frame_index+1, ...] = frame_mocap_data["motion"]["rot_local_euler"]
        
        if "pos_world" in frame_mocap_data["motion"] and "pos_world" in mocap_data["motion"]:
            mocap_data["motion"]["pos_world"][frame_index:frame_index+1, ...] = frame_mocap_data["motion"]["pos_world"] 

        if "rot_world" in frame_mocap_data["motion"] and "rot_world" in mocap_data["motion"]:
            mocap_data["motion"]["rot_world"][frame_index:frame_index+1, ...] = frame_mocap_data["motion"]["rot_world"]

    def _create_bvh_channel_names(self, mocap_data):

        bvh_channels = []
        bvh_channel_names = []

        skeleton_data = mocap_data["skeleton"]

        joints = skeleton_data["joints"]
        
        for joint in joints:
            if joint == joints[0]:
                bvh_channel_names.append(joint + "_Xposition")
                bvh_channel_names.append(joint + "_Yposition")
                bvh_channel_names.append(joint + "_Zposition")
                bvh_channel_names.append(joint + "_Xrotation")
                bvh_channel_names.append(joint + "_Yrotation")
                bvh_channel_names.append(joint + "_Zrotation")
                
                bvh_channels.append((joint, "Xposition"))
                bvh_channels.append((joint, "Yposition"))
                bvh_channels.append((joint, "Zposition"))
                bvh_channels.append((joint, "Xrotation"))
                bvh_channels.append((joint, "Yrotation"))
                bvh_channels.append((joint, "Zrotation"))
            else:
                bvh_channel_names.append(joint + "_Xrotation")
                bvh_channel_names.append(joint + "_Yrotation")
                bvh_channel_names.append(joint + "_Zrotation")
                
                bvh_channels.append((joint, "Xrotation"))
                bvh_channels.append((joint, "Yrotation"))
                bvh_channels.append((joint, "Zrotation"))

        return bvh_channel_names, bvh_channels
                
    def _create_bvh_skeleton(self, mocap_data, channel_names):
        
        bvh_skeleton = {}
        
        skeleton_data = mocap_data["skeleton"]
        
        joints = skeleton_data["joints"]
        offsets = skeleton_data["offsets"]
        children = skeleton_data["children"]
        
        for index in range(len(joints)):
            
            joint = joints[index]
            
            joint_data = {}
            joint_data["name"] = joint
            joint_data["offset"] = offsets[index].tolist()
            
            if index == 0:
                joint_data["channels"] = ["Xposition", "Yposition", "Zposition", "Xrotation", "Yrotation", "Zrotation"]
            else:
                joint_data["channels"] = ["Xrotation", "Yrotation", "Zrotation"]
                
            joint_children = []
            
            for child_index in children[index]:
                joint_children.append(joints[child_index])
                
            joint_data["children"] = joint_children
            
            bvh_skeleton[joint] = joint_data

        return bvh_skeleton
    
    def _create_bvh_values(self, mocap_data, channel_names):
        
        motion_data = mocap_data["motion"]
        
        pos_local = motion_data["pos_local"]
        rot_local_euler = motion_data["rot_local_euler"]

        frame_count = pos_local.shape[0]
        
        bvh_values = np.zeros((frame_count, len(channel_names)))

        for frame_index in range(frame_count):
            for joint_index in range(pos_local.shape[1]):
                
                if joint_index == 0:
                    bvh_values[frame_index, 0:3] = pos_local[frame_index, joint_index, :]
                    bvh_values[frame_index, 3:6] = rot_local_euler[frame_index, joint_index, :]
                else:
                    col_index = 6 + (joint_index - 1) * 3
                    bvh_values[frame_index, col_index:col_index+3] = rot_local_euler[frame_index, joint_index, :]

        return pandas.DataFrame(bvh_values, columns=channel_names)

    def _create_skeleton_data(self, bvh_data, mocap_data):
        
        skeleton = {}
        
        bvh_skeleton = bvh_data.skeleton
        joint_names = list(bvh_skeleton.keys())
        joint_count = len(joint_names)
        
        parents = [-1] * joint_count
        children = [[] for i in range(joint_count)]
        offsets = np.zeros((joint_count, 3))
        
        for joint_name in bvh_skeleton.keys():
            
            joint_index = joint_names.index(joint_name)
            
            offsets[joint_index] = bvh_skeleton[joint_name]["offset"]
            
            for child_name in bvh_skeleton[joint_name]["children"]:
                
                # BUGFIX: avoid end effectors since they aren't explicit joints in the hierarchy
                if child_name not in joint_names:
                    continue

                child_index = joint_names.index(child_name)

                parents[child_index] = joint_index
                children[joint_index].append(child_index)
                
        skeleton["root"] = bvh_data.root_name
        skeleton["joints"] = joint_names
        skeleton["parents"] = parents
        skeleton["children"] = children
        skeleton["offsets"] = offsets
        
        mocap_data["skeleton"] = skeleton

    def _create_motion_data(self, bvh_data, mocap_data):
        
        motion = {}
        
        joint_names = mocap_data["skeleton"]["joints"]
        joint_count = len(joint_names)
        rot_sequence = mocap_data["rot_sequence"]
        
        bvh_frames = bvh_data.values
        frame_count = bvh_frames.shape[0]
        
        bvh_frames_column_names = [ column for column in bvh_data.values.columns ]
        bvh_channels = set(bvh_data.channel_names)
        bvh_channel_joint_names = set([channel[0] for channel in bvh_channels])
        bvh_channel_value_names = ["Xposition", "Yposition", "Zposition", "Xrotation", "Yrotation", "Zrotation"]
        
        motion_translation = []
        motion_euler_rotation = []

        for joint_name in joint_names:

            if joint_name in bvh_channel_joint_names:
                joint_frames_combined = []      
            
                for i, value_name in enumerate(bvh_channel_value_names):
                    column_name = joint_name + "_" + value_name

                    if column_name in bvh_frames_column_names:
                        joint_frames = bvh_frames[column_name].values.reshape((-1, 1))
                        
                    else:
                        joint_frames = np.zeros((frame_count, 1))
                        
                    joint_frames_combined.append(joint_frames)

                joint_frames_combined = np.concatenate(joint_frames_combined, axis=1)

            else:
                joint_frames_combined = np.zeros((frame_count, 6))
                
            motion_translation.append(joint_frames_combined[:, 0:3])
            motion_euler_rotation.append(joint_frames_combined[:, 3:6])
        
        # move joint dimension to second axis
        motion["pos_local"] = np.swapaxes(np.stack(motion_translation, axis=2), 1, 2)
        motion["rot_local_euler"] = np.swapaxes(np.stack(motion_euler_rotation, axis=2), 1, 2)
        
        mocap_data["motion"] = motion

    def quat_to_euler(self, rot_quat, rot_sequence):
        
        frame_count = rot_quat.shape[0]
        joint_count = rot_quat.shape[1]
        
        rot_euler = np.zeros((frame_count, joint_count, 3))
        
        # 0 = x, 1 = y, 2 = z
        rot_seq_string = ""
        for i in range(3):
            if rot_sequence[i] == 0:
                rot_seq_string += "X"
            elif rot_sequence[i] == 1:
                rot_seq_string += "Y"
            elif rot_sequence[i] == 2:
                rot_seq_string += "Z"
        
        for fI in range(frame_count):
            for jI in range(joint_count):
                
                # BUGFIX for Transforms3d conversion
                qw = rot_quat[fI, jI, 0]
                qx = rot_quat[fI, jI, 1]
                qy = rot_quat[fI, jI, 2]
                qz = rot_quat[fI, jI, 3]

                ai, aj, ak = t3d.euler.quat2mat([qw, qx, qy, qz], rot_seq_string.lower())
                
                rot_euler[fI, jI, 0] = math.degrees(ai)
                rot_euler[fI, jI, 1] = math.degrees(aj)
                rot_euler[fI, jI, 2] = math.degrees(ak)
                
        return rot_euler

    def euler_to_quat(self, rot_euler, rot_sequence):
        
        # In FBX, RotationOrder is an enum/int where:
        # 0 = XYZ, 1 = XZY, 2 = YZX, 3 = YXZ, 4 = ZXY, 5 = ZYX, 6 = SphericXYZ
        # We need to map this directly to the string scipy expects
        
        # If it's a raw FBX Enum object, extract its integer value
        if hasattr(rot_sequence, 'value'):
            rot_val = rot_sequence.value
        elif isinstance(rot_sequence, int):
            rot_val = rot_sequence
        else:
            # Fallback if it was somehow provided as a list from older BVH
            rot_val = 0
            
        rot_mapping = {
            0: "xyz",
            1: "xzy",
            2: "yzx",
            3: "yxz",
            4: "zxy",
            5: "zyx",
            6: "xyz" 
        }
        
        rot_seq_string = rot_mapping.get(rot_val, "xyz")

        # Now process the euler array (or list of arrays)
        if isinstance(rot_euler, list):
            rot_quat = []
            for joint_euler in rot_euler:
                if len(joint_euler) == 0:
                    rot_quat.append(np.zeros((0, 4)))
                    continue
                rot_scipy = Rotation.from_euler(rot_seq_string, joint_euler, degrees=True)
                quat_scipy = rot_scipy.as_quat() # returns x, y, z, w
                jq = np.zeros((joint_euler.shape[0], 4))
                jq[:, 0] = quat_scipy[:, 3] # w
                jq[:, 1] = quat_scipy[:, 0] # x
                jq[:, 2] = quat_scipy[:, 1] # y
                jq[:, 3] = quat_scipy[:, 2] # z
                rot_quat.append(jq)
            return rot_quat
        else:
            frame_count = rot_euler.shape[0]
            joint_count = rot_euler.shape[1]
            rot_quat = np.zeros((frame_count, joint_count, 4))
            for jI in range(joint_count):
                rot_scipy = Rotation.from_euler(rot_seq_string, rot_euler[:, jI, :], degrees=True)
                quat_scipy = rot_scipy.as_quat() 
                rot_quat[:, jI, 0] = quat_scipy[:, 3] # w
                rot_quat[:, jI, 1] = quat_scipy[:, 0] # x
                rot_quat[:, jI, 2] = quat_scipy[:, 1] # y
                rot_quat[:, jI, 3] = quat_scipy[:, 2] # z
            return rot_quat
    
    def euler_to_quat_bvh(self, rot_euler, rot_sequence):
        
        frame_count = rot_euler.shape[0]
        joint_count = rot_euler.shape[1]
        
        rot_quat = np.zeros((frame_count, joint_count, 4))
        
        for fI in range(frame_count):
            for jI in range(joint_count):
                
                # get single joint euler rotation
                rx = math.radians(rot_euler[fI, jI, 0])
                ry = math.radians(rot_euler[fI, jI, 1])
                rz = math.radians(rot_euler[fI, jI, 2])
                
                qx = [math.cos(rx/2.0), math.sin(rx/2.0), 0.0, 0.0]
                qy = [math.cos(ry/2.0), 0.0, math.sin(ry/2.0), 0.0]
                qz = [math.cos(rz/2.0), 0.0, 0.0, math.sin(rz/2.0)]
                
                qs = [qx, qy, qz]
                
                qr = t3d.quaternions.qmult(qs[rot_sequence[0]], qs[rot_sequence[1]])
                qr = t3d.quaternions.qmult(qr, qs[rot_sequence[2]])
                
                # BUGFIX: The math above returns w,x,y,z natively from transforms3d
                rot_quat[fI, jI, 0] = qr[0]
                rot_quat[fI, jI, 1] = qr[1]
                rot_quat[fI, jI, 2] = qr[2]
                rot_quat[fI, jI, 3] = qr[3]

        return rot_quat

    def local_to_world(self, rot_local, pos_local, skeleton):
        
        root_name = skeleton["root"]
        joint_names = skeleton["joints"]
        root_index = joint_names.index(root_name)
        parents = skeleton["parents"]
        children = skeleton["children"]
        offsets = skeleton["offsets"]
        
        root_positions = pos_local[:, root_index, :]

        frame_count = rot_local.shape[0]
        joint_count = rot_local.shape[1]
        
        positions_world = []
        rotations_world = []
        
        for fI in range(frame_count):
            
            frame_positions_world = []
            frame_rotations_world = []
            
            for jI in range(joint_count):
                
                if parents[jI] == -1:
                    frame_positions_world.append(root_positions[fI])
                    frame_rotations_world.append(rot_local[fI, 0])
                else:
                    frame_positions_world.append(t3d.quaternions.rotate_vector(offsets[jI], frame_rotations_world[parents[jI]]) + frame_positions_world[parents[jI]])
                
                    if len(children[jI]) > 0:
                        frame_rotations_world.append(t3d.quaternions.qmult(frame_rotations_world[parents[jI]], rot_local[fI, jI]))
                    else:
                        frame_rotations_world.append(t3d.quaternions.qeye())
                        
            positions_world.append(frame_positions_world)
            rotations_world.append(frame_rotations_world)
            
        return np.array(positions_world), np.array(rotations_world)