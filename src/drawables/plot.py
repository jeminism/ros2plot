from drawables.drawable import Drawable
from utils.grid import Grid
from utils.graph_math import get_mapped_value, min_max, bresenham


class PlotXY(Drawable):
    def __init__(self, width:int, height:int, do_interpolation=True):
        super().__init__(width, height)
        self._interpolate = do_interpolation

    def plot(self, x_values: list, y_values: list, x_bound_min, x_bound_max, y_bound_min, y_bound_max):
        self.clear()
        if not all(isinstance(x, (int, float)) for x in x_values) and not all(isinstance(y, (int, float)) for y in y_values):
            raise TypeError("All elements must be numeric")
        
        if len(x_values) != len(y_values):
            raise ValueError("X and Y axis data must be of same length")
        
        if x_bound_min == x_bound_max:
            raise ValueError("X axis bound invalid! Min value == max value")
        if y_bound_min == y_bound_max:
            raise ValueError("Y axis bound invalid! Min value == max value")

        if len(x_values) == 0:
            return
        
        self._plot_impl(x_values, y_values, x_bound_min, x_bound_max, y_bound_min, y_bound_max)

    def _plot_impl(self, x_values: list, y_values: list, x_bound_min, x_bound_max, y_bound_min, y_bound_max):
        n_vals = len(x_values)
        prior_x = -1
        prior_y = -1
        for i in range(n_vals):
            y_index = get_mapped_value(y_values[i], y_bound_max, 0, y_bound_min, self._height-1) # flipped min and max because asciimatics y=0 is the topmost row of terminal.
            x_index = get_mapped_value(x_values[i], x_bound_max, self._width-1, x_bound_min, 0)
            # if x_index > self._width-1 or x_index < 0:
            #     raise ValueError(f"X Mapped value error. mapped value: {x_index}. ref value: {x_values[i]}, max_val: {x_bound_max}, max_i: {self._width-1}, min_val: {x_bound_min}, min_i: 0.")
            # if y_index > self._height-1 or y_index < 0:
            #     raise ValueError(f"Y Mapped value error. mapped value: {y_index}. ref value: {y_values[i]}, max_val: {y_bound_max}, max_i: {self._height-1}, min_val: {y_bound_min}, min_i: 0.")
            if (i > 0 and self._interpolate):
                for pt in bresenham(x_index, y_index, prior_x, prior_y):
                    p_y = pt[1]
                    p_x = pt[0]
                    # if p_x > self._width-1 or p_x < 0:
                    #     raise ValueError(f"X Mapped value error. mapped value: {p_x}. y0: {prior_y}, x0: {prior_x}, y1: {y_index}, x1: {x_index}")
                    # if p_y > self._height-1 or p_y < 0:
                    #     raise ValueError(f"Y Mapped value error. mapped value: {p_y}. y0: {prior_y}, x0: {prior_x}, y1: {y_index}, x1: {x_index}")
                    self.set_value(self.to_index(pt[0], pt[1]), "+")
            else:
                self.set_value(self.to_index(x_index, y_index), "+")
            prior_x = x_index
            prior_y = y_index


class PlotBraille(PlotXY):
    def __init__(self, width:int, height:int, do_interpolation=True):
        super().__init__(width, height, do_interpolation)
    
    def braille(self, dots):
        code = 0x2800
        for d in dots:
            code += 1 << (d - 1)
        return chr(code)

    def _plot_impl(self, x_values: list, y_values: list, x_bound_min, x_bound_max, y_bound_min, y_bound_max):
        n_vals = len(x_values)

        width = self._width*2
        height = self._height*4
        braille_grid = Grid(width, height)
        prior_x = -1
        prior_y = -1
        for i in range(n_vals):
            y_index = get_mapped_value(y_values[i], y_bound_max, 0, y_bound_min, height-1) # flipped min and max because asciimatics y=0 is the topmost row of terminal.
            x_index = get_mapped_value(x_values[i], x_bound_max, width-1, x_bound_min, 0)
            if (i > 0 and self._interpolate):
                for pt in bresenham(x_index, y_index, prior_x, prior_y):
                    braille_grid.set_value(braille_grid.to_index(pt[0], pt[1]), True)
            else:
                braille_grid.set_value(braille_grid.to_index(x_index, y_index), True)
            prior_x = x_index
            prior_y = y_index
        
        # parse braille chars
        for i in range(self._width):
            for j in range(self._height):
                b_x = i*2
                b_y = j*4
                braille_cells = []

                dot_offsets = [
                    (0, 0), (0, 1), (0, 2),
                    (1, 0), (1, 1), (1, 2),
                    (0, 3), (1, 3),
                ]

                for dot_num, (dx, dy) in enumerate(dot_offsets, start=1):
                    if braille_grid.at(braille_grid.to_index(b_x + dx, b_y + dy)):
                        braille_cells.append(dot_num)

                if len(braille_cells) > 0:
                    self.set_value(self.to_index(i, j), self.braille(braille_cells))
