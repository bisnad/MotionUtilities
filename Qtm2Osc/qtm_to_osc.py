"""
    Minimal usage example
    Connects to QTM and streams 3D data forever
    (start QTM first, load file, Play->Play with Real-Time output)
"""

import asyncio
import qtm
from qtm.packet import QRTComponentType
from pythonosc import dispatcher
from pythonosc import osc_server
from pythonosc import udp_client

#osc_tracking_send_address = "169.254.65.161"
osc_tracking_send_address = "2.0.0.31"
osc_tracking_send_port = 23456
#osc_tracking_send_address2 = "169.254.65.238"
#osc_tracking_send_address2 = "2.0.0.32"
osc_tracking_send_address2 = "127.0.0.1"
osc_tracking_send_port2 = 23456

osc_send_address_marker_positions = "/mocap/marker/pos"
osc_send_address_joint_positions = "/mocap/joint/pos"
osc_send_address_joint_rotations = "/mocap/joint/rot"

subject_name = "MUR"

joint_pos_scale = 1.0 / 10.0
marker_pos_scale = 1.0 / 100.0

subject_marker_positions = { subject_name: [] }
subject_joint_positions = { subject_name: [] }
subject_joint_rotations = { subject_name: [] }


"""
setup osc communication - osc send
"""

def create_osc_sender(osc_address, osc_port):
    # start osc client
    sender = udp_client.SimpleUDPClient(osc_address, osc_port)
    
    return sender
  
osc_tracking_sender = create_osc_sender(osc_tracking_send_address, osc_tracking_send_port)
osc_tracking_sender2 = create_osc_sender(osc_tracking_send_address2, osc_tracking_send_port2)    

def osc_send_marker_positions(subject_name):
    marker_positions = subject_marker_positions[subject_name]
    osc_tracking_sender.send_message(osc_send_address_marker_positions, marker_positions)
    osc_tracking_sender2.send_message(osc_send_address_marker_positions, marker_positions)
        
def osc_send_joint_positions(subject_name):
    if subject_name in subject_joint_positions:
        joint_positions = subject_joint_positions[subject_name]
        osc_tracking_sender.send_message(osc_send_address_joint_positions, joint_positions)
        osc_tracking_sender2.send_message(osc_send_address_joint_positions, joint_positions)
        
def osc_send_joint_rotations(subject_name):
    if subject_name in subject_joint_rotations:
        joint_rotations = subject_joint_rotations[subject_name]
        osc_tracking_sender.send_message(osc_send_address_joint_rotations, joint_rotations)
        osc_tracking_sender2.send_message(osc_send_address_joint_rotations, joint_rotations)


def on_packet(packet):
    """ Callback function that is called everytime a data packet arrives from QTM """
    #print("Framenumber: {}".format(packet.framenumber))
    
    if QRTComponentType.Component3d in packet.components:
        
        global subject_marker_positions
        
        #print("markers ", markers)
        header, markers = packet.get_3d_markers()
    
        marker_count = len(markers)
        
        if marker_count == 0:
            return
        
        if len(subject_marker_positions[subject_name]) != marker_count * 3:
            subject_marker_positions[subject_name] = [0.0] * marker_count * 3
            
        marker_positions = subject_marker_positions[subject_name]
        
        pos_index = 0
        
        for marker_index, marker_pos in enumerate(markers):
            
            marker_positions[pos_index] = marker_pos.x * joint_pos_scale
            marker_positions[pos_index+1] = marker_pos.y * joint_pos_scale
            marker_positions[pos_index+2] = marker_pos.z * joint_pos_scale

            pos_index += 3

            
            """
            print("joint_id ", joint_id)
            print("joint_pos ", joint_pos.x, " ", joint_pos.y, " ", joint_pos.z)
            print("joint_rot ", joint_rot.x, " ", joint_rot.y, " ", joint_rot.z, " ", joint_rot.w)
            """
            
        osc_send_marker_positions(subject_name)
        
    if QRTComponentType.ComponentSkeleton in packet.components:
        
        global subject_joint_positions
        global subject_joint_rotations
        
        skeleton = []
        
        #print("skeleton")
        header, skeletons = packet.get_skeletons()

        skeletonCount = len(skeletons)
        if skeletonCount == 0:
            return
        
        for skI, sk in enumerate(skeletons):
            if len(sk) > 0:
                skeleton = sk

        
        #skeleton = skeletons[0]
        
        jointCount = len(skeleton)
        
        if jointCount == 0:
            return
        
        if len(subject_joint_positions[subject_name]) != jointCount * 3:
            subject_joint_positions[subject_name] = [0.0] * jointCount * 3
        if len(subject_joint_rotations[subject_name]) != jointCount * 4:
            subject_joint_rotations[subject_name] = [0.0] * jointCount * 4
            
        joint_positions = subject_joint_positions[subject_name]
        joint_rotations = subject_joint_rotations[subject_name]
        
        pos_index = 0
        rot_index = 0
        
        #print("Framenumber: {}".format(packet.framenumber))
        
        for joint_index, joint in enumerate(skeleton):
            joint_id, joint_pos, joint_rot = joint
            
            joint_positions[pos_index] = joint_pos.x * joint_pos_scale
            joint_positions[pos_index+1] = joint_pos.y * joint_pos_scale
            joint_positions[pos_index+2] = joint_pos.z * joint_pos_scale
            
            joint_rotations[rot_index] = joint_rot.x
            joint_rotations[rot_index+1] = joint_rot.y
            joint_rotations[rot_index+2] = joint_rot.z
            joint_rotations[rot_index+3] = joint_rot.w
            
            pos_index += 3
            rot_index += 4
            
            #print("joint ", joint_id ," rot ", joint_rot.x, " ", joint_rot.y, " ", joint_rot.z, " ", joint_rot.w)
            
            """
            print("joint_id ", joint_id)
            print("joint_pos ", joint_pos.x, " ", joint_pos.y, " ", joint_pos.z)
            print("joint_rot ", joint_rot.x, " ", joint_rot.y, " ", joint_rot.z, " ", joint_rot.w)
            """
            
        osc_send_joint_positions(subject_name)
        osc_send_joint_rotations(subject_name)


async def setup():
    """ Main function """
    connection = await qtm.connect("127.0.0.1")
    if connection is None:
        return

    await connection.stream_frames(components=["skeleton:global", "3d"], frames=["frequency:50"], on_packet=on_packet)
    #await connection.stream_frames(components=["skeleton", "3d"], frames=["frequency:50"], on_packet=on_packet)


if __name__ == "__main__":
    asyncio.ensure_future(setup())
    asyncio.get_event_loop().run_forever()
    

"""
asyncio.ensure_future(setup())
asyncio.get_event_loop().run_forever()
"""