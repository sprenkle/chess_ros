import math


class Board:

    def __init__(self, camera_height, image_center_x, image_center_y, pixel_width, left, right, top, bottom, rows, cols):
        self.camera_height = camera_height
        self.image_center_x = image_center_x
        self.image_center_y = image_center_y
        self.pixel_x_width = (125.2*2)/(right - left)
        self.pixel_y_width = (125.2*2)/(bottom - top)

        self.org_x_mm = 290.0 #173  # 273 
        self.org_y_mm = -160.5 #-120.5
        
        self.pixel_width_mm = pixel_width #200 / 340
        
        self.org_board_pixel_x = left
        self.org_board_pixel_y = top
        self.rows = rows
        self.cols = cols
        self.left = left
        self.right = right
        self.bottom = bottom
        self.top = top

    def image_to_absolute(self, height, x: int, y: int):
        # Getting the shadow distance
        # In mm
        b = math.sqrt(math.pow(self.image_center_x - x, 2) + math.pow(self.image_center_y - y, 2))  * self.pixel_width_mm 
        angle_A = math.atan(b/self.camera_height)
        hght = height
        angl = math.tan(angle_A)

        sb = (hght * angl) #/ self.pixel_width

        # determining actual center of piece on image
        # In pixels
        if x - self.image_center_x == 0:
            board_angle = 0
        else:
            board_angle = math.atan(abs((y - self.image_center_y)/(x - self.image_center_x)))
        
        
        x_adjust = math.cos(board_angle) * sb
        y_adjust = math.sin(board_angle) * sb
        return x + (x_adjust * self.mult_adjust_x(x)), y + (y_adjust * self.mult_adjust_y(y))

    def mult_adjust_x(self, value):
        if value > self.image_center_x:
            return -1
        else:
            return 1

    def mult_adjust_y(self, value):
        if value > self.image_center_y:
            return -1
        else:
            return 1

    def get_position(self, height: float, x: int, y: int):
        xp, yp = self.image_to_absolute(height, x, y)
        xm = (xp - self.org_board_pixel_x) * self.pixel_x_width + self.org_x_mm 
        ym = (yp - self.org_board_pixel_y) * self.pixel_y_width + self.org_y_mm
        #print(f'x = {x} xp = {xp} xm = {xm}'  )
        #print(f'org_board_pixel_x {self.org_board_pixel_x}  pixel_width {self.pixel_x_width}  org_x_mm {self.org_x_mm}'  )
        return xm , ym 

    def get_position_mm(self, x: int, y: int):
        xm = (x - self.org_board_pixel_x) * self.pixel_x_width + self.org_x_mm  #+ .5#+ ((x - 82) / 82) 
        ym = (y - self.org_board_pixel_y) * self.pixel_y_width + self.org_y_mm  #- (((x - 102) / x) * 160)
        #print(f'x={x} y={y} xm={xm} ym={ym} org_x_m={self.org_x_mm} org_y_mm={self.org_y_mm}')
        return xm * .001, ym * .001 

    def get_board_square(self, px: int, py: int):
        square_width = int(((self.right - self.left) + (self.bottom - self.top))/16)
        row = int((py - self.org_board_pixel_y) / square_width)
        col = int((px - self.org_board_pixel_x) / square_width)
        #print(f'square_width={square_width}  org_boar_pixel_y={self.org_board_pixel_y} py={py} row = {row}')
        return col, row


# camera height = 310
#chess_board = Board(700, 320, 240, .456)
if __name__ == '__main__':
    chess_board = Board(423, 320, 240, 1.4558, 83, 445, 100, 464)
    print(chess_board.image_to_absolute(44.7, 378, 306))
