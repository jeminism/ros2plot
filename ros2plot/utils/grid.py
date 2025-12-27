# Implement a generic Grid class which manages a buffer of characters for rendering.
# The buffer is represented as a grid
class Grid:
    def __init__(self, width:int, height:int):
        self._width = width
        self._height = height
        self._grid = [None]*width*height

    def to_index(self, x: int, y: int, width=None) -> int:
        if x < 0:
            raise ValueError(f"[Grid] x must be non-negative, got {x}")
        if y < 0:
            raise ValueError(f"[Grid] y must be non-negative, got {y}")
        if width == None:
            width = self._width
        return x + y*width
    
    def to_xy(self, index: int, width=None) -> tuple[int, int]:
        if index < 0:
            raise ValueError(f"[Grid] index must be non-negative, got {index}")
        if width == None:
            width = self._width
        return index % width, index //width

    def clear(self):
        self._grid = [None]*self._width*self._height

    # def set_value(self, x: int, y: int, value):
    #     self.set(to_index(x, y), value)

    def set_value(self, index: int, value):
        if index >= len(self._grid):
            raise ValueError(f"[Grid] index out of range ({len(self._grid)}), got {index}")
        self._grid[index] = value
    
    def at(self, index):
        if index >= len(self._grid):
            raise ValueError(f"[Grid] index out of range ({len(self._grid)}), got {index}")
        return self._grid[index]

