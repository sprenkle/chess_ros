#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import sys
import rospy

from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel


def spawn_sdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e)) 

def spawn_sdf_model(name, path, pose, reference_frame):
    # Load Model SDF
    description_xml = ''
    with open(path, "r") as model_file:
        description_xml = model_file.read().replace('\n', '')
        spawn_sdf(name, description_xml, pose,reference_frame)

def get_position(num):
    num * .03125
    -.109375 + x_org + (num * .03125)


if __name__ == "__main__":
    print("Hello World")
    blocks_table_name = "blocks_table"
    blocks_table_path = "table/model.sdf"
    blocks_table_pose = Pose(position=Point(x=0.75, y=0.0, z=0.01),  orientation=Quaternion(0,0,0,0))

    world_reference_frame = "world"

    x_org = 0
    y_org = .40
    board_height = 0
    static = "true"

    pose = Pose(position=Point(y=0 + x_org, x=0 + y_org, z=0))
    mo = '<?xml version="1.0"?><sdf version="1.6"><model name="checkerboard_plane"><static>true</static><link name="link"><visual name="visual"><geometry><mesh><scale>.125 .125 1 1 1</scale><uri>model://checkerboard_plane/meshes/checkerboard_plane.dae</uri></mesh></geometry></visual></link></model></sdf>'
    spawn_sdf(f'board', mo, pose, world_reference_frame)

    for i in range(8):
        print(i)
        pose = Pose(position=Point(y=-.109375 + x_org + (i * .03125), x=-.078125 + y_org, z=board_height))
        mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_pawn"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewPawn.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
        spawn_sdf(f'white_pawn{i}', mo, pose, world_reference_frame)

        pose = Pose(position=Point(y=-.109375 + x_org + (i * .03125), x=.078125 + y_org, z=board_height))
        mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_pawn"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewPawn.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
        spawn_sdf(f'black_pawn{i}', mo, pose, world_reference_frame)


    # Rook
    pose = Pose(position=Point(y=-.109375 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_rook1"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewRook.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_rook1', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=.109375 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_rook2"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewRook.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_rook2', mo, pose, world_reference_frame)


    pose = Pose(position=Point(y=-.109375 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_rook1"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewRook.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_rook1', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=.109375 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_rook2"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewRook.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_rook2', mo, pose, world_reference_frame)


    # Knight
    pose = Pose(position=Point(y=-.078125 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_knight1"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewKnight.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_knight1', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=.078125 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_knight2"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewKnight.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_knight2', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=-.078125 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_knight1"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewKnight.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_knight1', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=.078125 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_knight2"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewKnight.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_knight2', mo, pose, world_reference_frame)


    # Bishop
    pose = Pose(position=Point(y=-.046875 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_bishop1"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewBishop.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_bishop1', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=.046875 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_bishop2"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewBishop.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_bishop2', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=-.046875 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_bishop1"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewBishop.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_bishop1', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=.046875 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_bishop2"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewBishop.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_bishop2', mo, pose, world_reference_frame)




    # King
    pose = Pose(position=Point(y=.015625 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_king"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewKing.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_king', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=.015625 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_king"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewKing.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_king', mo, pose, world_reference_frame)


    # Queen
    pose = Pose(position=Point(y=-.015625 + x_org, x=-.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="white_queen"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewQueen.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/White</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'white_queen', mo, pose, world_reference_frame)

    pose = Pose(position=Point(y=-.015625 + x_org, x=.109375 + y_org, z=board_height))
    mo = '<?xml version="1.0" ?><sdf version="1.5"> <model name="black_queen"><static>true</static><link name="body"><visual name="visual"><geometry><mesh><scale>.001 .001 .001 .001 .001</scale><uri>file:///home/david/models/ScrewQueen.stl</uri></mesh></geometry><material><lighting>1</lighting><script><uri>file://materials/scripts/chess.material</uri><name>Gazebo/Indigo</name></script><shader type="pixel"><normal_map>__default__</normal_map></shader></material></visual></link></model></sdf>'
    spawn_sdf(f'black_queen', mo, pose, world_reference_frame)

    #spawn_sdf_model("pawn", "/home/david/.gazebo/models/white_pawn/model.sdf", blocks_table_pose, world_reference_frame)
