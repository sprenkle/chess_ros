import math


class Board:

    def __init__(self, camera_height, image_width, image_height, left, right, top, bottom):
        self.camera_height = camera_height
        self.image_center_x = image_width / 2
        self.image_center_y = image_height / 2
        self.pixel_x_width = 250/(right - left)
        self.pixel_y_width = 250/(bottom - top)

        #self.org_x_mm =  s
        #self.org_y_mm = -160.5 #-120.5
        
        self.pixel_width_mm = (self.pixel_x_width +  self.pixel_y_width)/2  #200 / 340
        
        self.org_board_pixel_x = left
        self.org_board_pixel_y = top
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
        height = 36.5
        xp, yp = self.image_to_absolute(height, x, y)
        if xp < self.left * .9 or xp > self.right * 1.1 or yp < self.top * .9 or yp > self.bottom * 1.1 :
            return -1, -1
        #print(f'xp = {xp}  yp={yp}'  )
        xm =  int((xp - self.left)/(self.right - self.left) / 0.125) #        (xp - self.org_board_pixel_x) * self.pixel_x_width + self.org_x_mm 
        ym = int((yp - self.top)/(self.bottom - self.top) / 0.125) #(yp - self.org_board_pixel_y) * self.pixel_y_width + self.org_y_mm
        return xm , ym 


# camera height = 310
#chess_board = Board(700, 320, 240, .456)
if __name__ == '__main__':
    chess_board = Board(423, 640, 480, 85, 463, 79, 455)
