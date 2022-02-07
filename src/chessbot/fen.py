import numpy as np

class Fen():
    def __init__(self, fen, white_bottom):
        self.fen = fen
        self.values = fen.split(' ')
        self.array = self.create_array(fen, white_bottom)
        if white_bottom:
            self.alg_to_num = {'a':0, 'b':1, 'c':2, 'd':3, 'e':4, 'f':5, 'g':6, 'h':7, '1':7, '2':6, '3':5, '4':4, '5':3, '6':2, '7':1, '8':0}
        else:
            self.alg_to_num = {'a':7, 'b':6, 'c':5, 'd':4, 'e':3, 'f':2, 'g':1, 'h':0, '1':0, '2':1, '3':2, '4':3, '5':4, '6':5, '7':6, '8':7}



    def to_move(self):
        if self.values[1] == 'w':
            return 'white'
        else:
            return 'black'

    def move_num(self):
        m = (int(self.values[5]) - 1) * 2
        if self.values[1] == 'b':
            m = m + 1
        return m

    def piece_at(self, x, y):
        return self.array[x, y, 0]

    def en_passant(self):
        print(f'self.values {self.values}')
        return self.values[3]

    # Returns x, y with 0,0 being top left
    def get_move_components(self, alg_not):
        fromX = self.alg_to_num[alg_not[0:1]]
        fromY = self.alg_to_num[alg_not[1:2]]
        toX = self.alg_to_num[alg_not[2:3]]
        toY = self.alg_to_num[alg_not[3:4]]
        return fromX, fromY, toX, toY

    def create_array(self, data, white_bottom):
        col = 0
        row = 0
        old_pos = np.zeros((8,8,2),np.int32)
        b = 1
        w = 0
        for i in range(100):
            c = data[i:i+1]
            if c == '/':
                row = row + 1
                col = 0
            elif c.isnumeric():
                n = int(c)
                for pos in range(n):
                    old_pos[col,row, 0] = 0
                    col = col + 1
            elif c == 'p':
                old_pos[col, row, 0] = 1
                old_pos[col, row, 1] = b
                col += 1
            elif c == 'n':
                old_pos[col, row, 0] = 2
                old_pos[col, row, 1] = b
                col += 1
            elif c == 'b':
                old_pos[col, row, 0] = 3
                old_pos[col, row, 1] = b
                col += 1
            elif c == 'r':
                old_pos[col, row, 0] = 4
                old_pos[col, row, 1] = b
                col += 1
            elif c == 'q':
                old_pos[col, row, 0] = 5
                old_pos[col, row, 1] = b
                col += 1
            elif c == 'k':
                old_pos[col, row, 0] = 6
                old_pos[col, row, 1] = b
                col += 1
            elif c == 'P':
                old_pos[col, row, 0] = 1
                old_pos[col, row, 1] = w
                col += 1
            elif c == 'N':
                old_pos[col, row, 0] = 2
                old_pos[col, row, 1] = w
                col += 1
            elif c == 'B':
                old_pos[col, row, 0] = 3
                old_pos[col, row, 1] = w
                col += 1
            elif c == 'R':
                old_pos[col, row, 0] = 4
                old_pos[col, row, 1] = w
                col += 1
            elif c == 'Q':
                old_pos[col, row, 0] = 5
                old_pos[col, row, 1] = w
                col += 1
            elif c == 'K':
                old_pos[col, row, 0] = 6
                old_pos[col, row, 1] = w
                col += 1
            if row >= 7 and col > 7:
                break 
        if not white_bottom:
            old_pos = np.rot90(old_pos)
            old_pos = np.rot90(old_pos)

        return old_pos

if __name__ == '__main__':
    mfen = Fen("2k2b1r/p4ppp/2p1p3/2N5/1B6/7P/5PPK/1q6 b - - 2 23", True)
