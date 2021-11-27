from chessbot import board
from game_msgs.msg import Whatboard
from game_msgs.msg import Chesspiece

class ChessBoard:

    def __init__(self):
        #rospy.Subscriber(topic_piece_detector, msg_Image, self.callback)
        self.board = board.Board(423, 320, 240, 1, 108, 472, 76, 439, 8, 8)
        #self.starting_position = [Chesspiece('King', 'White', 2, 4)]
        self.avg_piece_height = 44.6 #38
        self.current_position = [[0 for _ in range(8)] for _ in range(8)]
        self.current_position[3][3] = Chesspiece(Name = 'King', Color='White', Row = 3, Col=3) 
        self.state = 0


    def callback(self, data):
        # check if missing exactly one piece
        # check if exactly one piece moved
        pass

    def get_chess_pieces(self, pieces):
        whatboard = Whatboard(State = 0)
        match = 0
        empty_space = 0

        position = [[0 for _ in range(8)] for _ in range(8)]

        for piece in pieces:
            #print(pieces)
            px, py = self.board.image_to_absolute(self.avg_piece_height, piece[1], piece[2])
            x, y = self.board.get_position_mm(px, py)
            print(f'XY= {x} {y}')
            col, row = self.board.get_board_square(px, py)
            # if current_position[col, row] == 0:
            #     empty_space += 1
            # else:
            #     match += 1
            piece = Chesspiece(Name = 'King', Color='White', X=x, Y=y, Height=44.6, Row = row, Col=col, PixelX=int(px), PixelY=int(py))
            whatboard.Moved = piece
            whatboard.State = 0
            # position[col][row] = piece

        for row_index in range(0,7):
            for col_index in range(0, 7):
                pass
        
        return whatboard

    def is_piece_missing(self, pieces):
        pass

if __name__ == '__main__':
    print('Hello World')
