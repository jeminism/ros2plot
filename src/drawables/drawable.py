from utils.grid import Grid
from asciimatics.screen import Screen

# Implement a generic Drawable class which manages a buffer of characters for rendering.
# The buffer is represented as the underlying _grid structure inherited from Grid class

class Drawable(Grid):
    def __init__(self, width:int, height:int):
        super().__init__(width, height)

    def draw(self, screen: Screen, x: int, y: int, colour=7):
        buf = []
        buf_start = -1
        for i in range(self._width*self._height):
            # write the existing buffer when changing line or if end of an existing buffer
            if (i != 0 and i % self._width == 0) or self._grid[i] == None:
                if len(buf) > 0:
                    dx, dy = self.to_xy(buf_start)
                    screen.print_at(self.get_buffer(buf), x+dx, y+dy, colour=colour)
                    buf = []
            
            # re initialize the buffer if the current char is not empty
            if self._grid[i] != None:
                if len(buf) == 0:
                    buf_start = i
                buf.append(self._grid[i])
            
        if len(buf) > 0:
            dx, dy = self.to_xy(buf_start)
            screen.print_at(self.get_buffer(buf), x+dx, y+dy, colour=colour)
                
    
    def get_buffer(self, buf: list[str]) -> str:
        return "".join(buf)
