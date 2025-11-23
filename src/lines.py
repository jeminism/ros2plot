from drawable import Drawable

class VerticalLine(Drawable):
    def __init__(self, height:int):
        super().__init__(1, height)
        for i in range(height):
            self.set_value(self.to_index(0, i), "|")

class HorizontalLine(Drawable):
    def __init__(self, length:int):
        super().__init__(length, 1)
        for i in range(length):
            self.set_value(self.to_index(i, 0), "-")