from drawables.lines import VerticalLine, HorizontalLine
from drawables.plot import PlotBraille, PlotXY
from utils.graph_math import get_mapped_value, min_max, multi_min_max
from utils.colour_palette import COLOURS, NUM_COLOURS
from asciimatics.screen import Screen
from asciimatics.effects import Effect

from typing import TypedDict, List

class GraphData(TypedDict):
    x_values: List
    y_values: List[List]
    paused: bool

class GraphXY(Effect):

    def __init__(self, screen: Screen, x_pos: int, y_pos: int, width:int, height:int, data: GraphData, plot_hd = True, draw_lines = True, y_label_padding=5):
        super().__init__(screen)
        self._y_axis = VerticalLine(height)
        self._x_axis = HorizontalLine(width)
        self._plot = PlotBraille(width, height, draw_lines) if plot_hd else PlotXY(width, height, draw_lines)
        self._y_label_padding = 5
        self._height = height
        self._width = width
        self._data = data
        self._x_origin = x_pos
        self._y_origin = y_pos

    def reset(self):
        self._screen.clear()
    @property
    def stop_frame(self):
        return 10**12  # arbitrarily long duration  

    
    def _update(self, frame_no):
        if self._data["paused"]:
            return
        self._draw(self._x_origin, self._y_origin, self._data["x_values"], self._data["y_values"])

    def _draw(self, x_draw: int, y_draw: int, x_values: list, y_values: list[list]):
        if not all(isinstance(x, (int, float)) for x in x_values) and not (all(isinstance(y, (int, float)) for y in y_data) for y_data in y_values):
            raise TypeError("All elements must be numeric")
        
        # if len(x_values) != len(y_values):
        #     raise ValueError("X and Y axis data must be of same length")

        x_val_min, x_val_max = min_max(x_values)
        y_val_min, y_val_max = multi_min_max(y_values)
        
        if x_val_min == x_val_max:
            if x_val_min < 0:
                x_val_min += x_val_min
            elif x_val_min > 0:
                x_val_max += x_val_max
            else:
                x_val_min = -1
                x_val_max = 1

        if y_val_min == y_val_max:
            if y_val_min < 0:
                y_val_min += y_val_min
            elif y_val_min > 0:
                y_val_max += y_val_max
            else:
                y_val_min = -1
                y_val_max = 1

        # if not x_bound_min == None:
        #     x_val_min = x_bound_min
        # if (x_bound_max):
        #     x_val_max = x_bound_max
        # if (y_bound_min):
        #     y_val_min = y_bound_min
        # if (y_bound_max):
        #     y_val_max = y_bound_max
        
        for i in range(len(y_values)):
            c = COLOURS[i%NUM_COLOURS]
            #---- update plot data
            self._plot.plot(x_values, y_values[i], x_val_min, x_val_max, y_val_min, y_val_max)
            #---- draw stuff
            self._plot.draw(self._screen, x_draw, y_draw, colour=c)

        y_axis_location = get_mapped_value(0 if x_val_min < 0 else x_val_min, x_val_max, self._width-1, x_val_min, 0)
        x_axis_location = get_mapped_value(0 if y_val_min < 0 else y_val_min, y_val_max, 0, y_val_min, self._height-1)
        self._y_axis.draw(self._screen, x_draw+y_axis_location, y_draw)
        self._x_axis.draw(self._screen, x_draw, y_draw+x_axis_location)

        #---- draw labels. only print latest, min, max
        y_label_x_location = max(x_draw, x_draw+y_axis_location-2)
        for y_data in y_values:
            y_latest_location = get_mapped_value(y_data[-1], y_val_max, 0, y_val_min, self._height-1)
            self._screen.print_at(f"{y_data[-1]:3.2f}", y_label_x_location, y_draw+y_latest_location)

        self._screen.print_at(f"{y_val_max:3.2f}", y_label_x_location, y_draw)
        self._screen.print_at(f"{y_val_min:3.2f}", y_label_x_location, y_draw+self._height-1)

        x_latest_location = get_mapped_value(x_values[-1], x_val_max, self._width-1, x_val_min, 0)
        x_label_y_location = min(y_draw+self._height-1, y_draw+x_axis_location+1)
        self._screen.print_at(f"{x_values[-1]:3.2f}", x_draw+x_latest_location, x_label_y_location)
        self._screen.print_at(f"{x_val_max:3.2f}", x_draw+self._width-1, x_label_y_location)
        self._screen.print_at(f"{x_val_min:3.2f}", x_draw, x_label_y_location)


        
