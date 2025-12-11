# from drawables.lines import VerticalLine, HorizontalLine
# from drawables.plot import PlotBraille, PlotXY
from utils.graph_math import get_mapped_value, min_max, multi_min_max, bresenham
from utils.braille import braille_char
from utils.colour_palette import COLOURS, NUM_COLOURS
from utils.grid import Grid

from asciimatics.screen import Screen
from effects.effect_base import EffectBase, DrawOffsets

# from typing import TypedDict, List
import attrs

@attrs.define
class GraphData:
    x_values: list = attrs.field(default=[])
    y_values: list[list] = attrs.field(default=[])
    colours: list = attrs.field(default=[])
    paused: bool = attrs.field(default=False)

MAX_GRAPH_PTS = 1000


@attrs.define
class GraphConfigs:
    x: int = attrs.field(default=0) #origin position, where axis intersect
    y: int = attrs.field(default=0)
    width: int = attrs.field(default=0) #width of the plot area, in pixels
    height: int = attrs.field(default=0) #height of the plot area, in pixels
    y_min_value: int = attrs.field(default=0) # y value corresponding to pixel (x, height-1)
    y_max_value: int = attrs.field(default=0) # y value corresponding to pixel (x, 0)
    x_min_value: int = attrs.field(default=0) # x value corresponding to pixel (0, y)
    x_max_value: int = attrs.field(default=0) # x value corresponding to pixel (width-1, y)


class YAxis(EffectBase):
    def __init__(self, screen: Screen, cfg: GraphConfigs, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, offsets)
        self._cfg = cfg
    
    def _draw(self, frame_no):
        if self._cfg.height == 0:
            raise ValueError("Tried to draw YAxis with length = 0!")
        for i in range(self._cfg.height):
            self.e_print("|", self._cfg.x, i)
        # draw min and max labels
        s_min = f"{self._cfg.y_min_value:3.2f}"
        self.e_print(s_min, self._cfg.x-min(self._offsets.x, len(s_min)), self._cfg.height-1)

        s_max = f"{self._cfg.y_max_value:3.2f}"
        self.e_print(s_max, self._cfg.x-min(self._offsets.x, len(s_max)), 0)

class XAxis(EffectBase):
    def __init__(self, screen: Screen, cfg: GraphConfigs, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, offsets)
        self._cfg = cfg
    
    def _draw(self, frame_no):
        if self._cfg.width == 0:
            raise ValueError("Tried to draw XAxis with length = 0!")
        for i in range(self._cfg.width):
            self.e_print("-", i, self._cfg.y)
        # draw min and max labels
        self.e_print(f"{self._cfg.x_min_value:5.2f}", 0, self._cfg.y+1)
        self.e_print(f"{self._cfg.x_max_value:5.2f}", self._cfg.width-1, self._cfg.y+1)

# represents the plot of a single data set
@attrs.define
class PlotData:
    x_data: list = attrs.field(default=[])
    y_data: list = attrs.field(default=[])
    colour: int = attrs.field(default=7)
    interpolate: bool = attrs.field(default=True)
    high_def: bool = attrs.field(default=True)

def new_plot_data(x_values: list, y_values: list, colour: int, interpolate: bool=True, high_def: bool=True):
    p = PlotData()
    p.x_data = x_values
    p.y_data = y_values
    p.colour = colour
    p.interpolate = interpolate
    p.high_def = high_def
    return p

class Plot(EffectBase):
    def __init__(self, screen: Screen, cfg: GraphConfigs, data: PlotData, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, offsets)
        self._cfg = cfg
        self._data = data
    
    def _draw(self, frame_no):
        if len(self._data.x_data) == 0:
            return
            
        if not all(isinstance(x, (int, float)) for x in self._data.x_data) and not all(isinstance(y, (int, float)) for y in self._data.y_data):
            raise TypeError("All elements must be numeric")
        
        if len(self._data.x_data) != len(self._data.y_data):
            raise ValueError("X and Y axis data must be of same length")
        
        if self._cfg.x_min_value == self._cfg.x_max_value:
            raise ValueError("X axis bound invalid! Min value == max value")
        if self._cfg.y_min_value == self._cfg.y_max_value:
            raise ValueError("Y axis bound invalid! Min value == max value")

        self._draw_impl()


    def _draw_impl(self):
        n_vals = len(self._data.x_data)
        use_braille = self._data.high_def

        width = self._cfg.width*2 if use_braille else self._cfg.width
        height = self._cfg.height*4 if use_braille else self._cfg.height

        grid = Grid(width, height)
        prior_x = -1
        prior_y = -1
        for i in range(n_vals):
            y_index = get_mapped_value(self._data.y_data[i], self._cfg.y_max_value, 0, self._cfg.y_min_value, height-1) # flipped min and max because asciimatics y=0 is the topmost row of terminal.
            x_index = get_mapped_value(self._data.x_data[i], self._cfg.x_max_value, width-1, self._cfg.x_min_value, 0)
            if x_index > width-1 or x_index < 0:
                continue
            if y_index > height-1 or y_index < 0:
                continue
            if (i > 0 and self._data.interpolate):
                for pt in bresenham(x_index, y_index, prior_x, prior_y):
                    if pt[0] > width-1 or pt[0] < 0:
                        continue
                    if pt[1] > height-1 or pt[1] < 0:
                        continue
                    grid.set_value(grid.to_index(pt[0], pt[1]), True)
            else:
                grid.set_value(grid.to_index(x_index, y_index), True)
            prior_x = x_index
            prior_y = y_index
        
        # parse chars
        for i in range(self._cfg.width):
            for j in range(self._cfg.height):
                if use_braille:
                    b_x = i*2
                    b_y = j*4
                    braille_cells = []

                    dot_offsets = [
                        (0, 0), (0, 1), (0, 2),
                        (1, 0), (1, 1), (1, 2),
                        (0, 3), (1, 3),
                    ]

                    for dot_num, (dx, dy) in enumerate(dot_offsets, start=1):
                        if grid.at(grid.to_index(b_x + dx, b_y + dy)):
                            braille_cells.append(dot_num)

                    if len(braille_cells) > 0:
                        self.e_print(braille_char(braille_cells), i, j, self._data.colour)
                else:
                    if grid.at(grid.to_index(i, j)):
                        self.e_print("*", i, j, self._data.colour)

        # print last value
        x_latest_location = min(get_mapped_value(self._data.x_data[-1], self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0), self._cfg.width)
        y_latest_location = get_mapped_value(self._data.y_data[-1], self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height-1)
        self.e_print(f"{self._data.y_data[-1]:3.2f}", x_latest_location, y_latest_location, self._data.colour)