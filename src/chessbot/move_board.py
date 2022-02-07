#!/usr/bin/env python3  

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from game_msgs.msg import GameStatus
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_common_modules import angle_manipulation as ang
import numpy as np
import time
from chessbot import fen

class MoveBoard:

    def __init__(self, start_node = False):
        self.piece_height = {6:0.0475, 5:0.0435, 4:.0295, 3:0.0325, 2:0.035, 1:.023}
        #self.piece_pickup = {6:1.5, 5:2.5, 4:3.5, 3:4.5, 2:5.5, 1:6.5}
        self.bot = InterbotixManipulatorXS("rx200", "arm", "gripper", gripper_pressure=1, init_node=start_node)
        self.bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
        self.bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
        self.bot.dxl.robot_set_motor_registers("single", "wrist_angle", "Position_P_Gain", 1500)
        self.bot.dxl.robot_set_motor_registers("single", "gripper", "Position_P_Gain", 1500)
        self.bot.dxl.robot_torque_enable("group", "arm", True)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        self.trans = self.get_transform(tfBuffer,'rx200/base_link', 'rx200/board')
        self.bot.arm.go_to_sleep_pose()
        self.pickupHeight = .11
        self.enable_move = False
        #rospy.Subscriber('/chess_status', GameStatus,self.chess_status_callback)

    def chess_status_callback(self, data):
        print(f'move_board - white bottom ={data.white_player == "human"}')
        self.fen = fen.Fen(data.board, data.white_player == "human")



    def get_transform(self, tfBuffer, target_frame, source_frame):
        # try:
        #     trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (target_frame, source_frame))
        #     return np.identity(4)
        # x = trans.transform.translation.x
        # y = trans.transform.translation.y
        # z = trans.transform.translation.z
        # quat = trans.transform.rotation
        # quat_list = [quat.x, quat.y, quat.z, quat.w]
        # rpy = euler_from_quaternion(quat_list)
        x = 0.252
        y = -0.12
        z = .08 # .875
        rpy = [-0.01, -.075, 0.01] # -.075 .019
        T_TargetSource = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])
        return T_TargetSource


    def set_pose(self, ar):
        #print(ar)
        #print('Start of Move')
        self.bot.arm.set_ee_pose_components(x=ar[0] * 1.02, y=ar[1] * 1.03, z=ar[2], blocking=True)
        #print('End of Move')

    def attract(self):
        #self.bot.gripper.open(delay=2)
        self.bot.gripper.attract()

    def release(self):
        #self.bot.gripper.close()
        self.bot.gripper.release()

    def get_board_coordinates(self, y, x):
        board_x = 0.015625 + (.03125 * x)
        board_y = 0.015625 + (.03125 * y)
        return board_x, board_y


    def pick_up(self, x, y, piece_hight):
        bx, by = self.get_board_coordinates(x, y)
        fromHigh = np.matmul(self.trans, np.array([bx, by, self.pickupHeight, 1]))
        fromPickup = np.matmul(self.trans, np.array([bx, by, piece_hight, 1]))

        self.attract()
        self.set_pose(fromHigh)
        self.set_pose(fromPickup)
        self.set_pose(fromHigh)


    def put_down(self, x, y, piece_hight):
        bx, by = self.get_board_coordinates(x, y)
        toHigh = np.matmul(self.trans, np.array([bx, by, self.pickupHeight, 1]))
        toPickup = np.matmul(self.trans, np.array([bx, by, piece_hight +.01, 1])) #.005
        
        self.set_pose(toHigh)
        self.set_pose(toPickup)
        self.release()
        self.set_pose(toHigh)

    def remove_piece(self, x, y, piece_hight):
        self.pick_up(x, y, piece_hight)
        self.bot.arm.set_ee_pose_components(x=0.3, y=-.2,z=0.25)
        self.release()

    def rest_position(self):
        self.bot.arm.go_to_sleep_pose()

    def castle(self, fromX, fromY, toX, toY):
        self.pick_up(fromX, fromY, self.piece_height[6])
        self.put_down(toX , toY, self.piece_height[6])

        if fromX < toX:
            rookX = 7
        else:
            rookX = 0

        self.pick_up(rookX, fromY, self.piece_height[4])
        
        if fromX < toX:
            self.put_down(toX - 1, toY, self.piece_height[4])
        else:
            self.put_down(toX + 1, toY, self.piece_height[4])

    def en_passant(self, fromX, fromY, toX, toY):
        self.remove_piece(toX, fromY, self.piece_height[1])
        piece_hight = self.piece_height[1]
        self.pick_up(fromX, fromY, piece_hight)
        self.put_down(toX , toY, piece_hight)
        self.rest_position()

    def move_alg(self, move, my_fen):
        self.fen = my_fen

        print(f'move_alg = {move}')
        
        fromX, fromY, toX, toY = self.fen.get_move_components(move) 
        
        if (fromX == 3 and (fromY == 0 or fromY == 7) and (toX == 1 or toX == 5) and self.fen.piece_at(fromX, fromY) == 6) or\
            (fromX == 4 and (fromY == 0 or fromY == 7) and (toX == 2 or toX == 6) and self.fen.piece_at(fromX, fromY) == 6):
            self.castle(fromX, fromY, toX, toY)
            return

        if move[2:4] ==self.fen.en_passant() and self.fen.piece_at(fromX, fromY) == 1 and fromX != toX:
            print(f'move[2:4] = {move[2:4]} ')
            self.en_passant(fromX, fromY, toX, toY)
            return

        self.move(fromX, fromY, toX, toY)
        self.rest_position()
        return True

    def move(self, fromX, fromY, toX, toY):
        print(f'Piece from {fromX} {fromY} {self.fen.piece_at(fromX, fromY)}  to {toX} {toY} {self.fen.piece_at(toX, toY)}')
        if self.fen.piece_at(toX, toY) != 0:
            self.remove_piece(toX, toY, self.piece_height[self.fen.piece_at(toX, toY)])
        piece_hight = self.piece_height[self.fen.piece_at(fromX, fromY)]
        self.pick_up(fromX, fromY, piece_hight)
        self.put_down(toX , toY, piece_hight)
        self.rest_position()
    
    def sleep(self):
        self.bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    print("move.py")
    chessBoard2 = MoveBoard(start_node=True)
    print("ddsc")
    game_status = GameStatus(board = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1", white_player = "human") 
    print("tesc")
    chessBoard2.chess_status_callback(game_status)
    print("bakc")
    chessBoard2.move_alg("d8d6")
    chessBoard2.move_alg("d1d3")
    #chessBoard2.move(1,3,2,3)
    # chessBoard2.move(6,0,1,0)
    # for i in range(8):
    #     chessBoard2.move(0, i, 7 ,i)
    #     chessBoard2.move(1, i, 6 ,i)
    # chessBoard2.sleep()
    # chessBoard2.move(0,3,7,3)
    # chessBoard2.move(7,3,0,3)
    #chessBoard2.move(0,4,7,4)
    #chessBoard2.move(7,4,0,4)
    #chessBoard2.bot.arm.set_ee_pose_components(x=0.3, y=-.2,z=0.25)