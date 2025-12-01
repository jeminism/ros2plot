from drawables.lines import VerticalLine, HorizontalLine
from drawables.plot import PlotBraille, PlotXY
from utils.graph_math import get_mapped_value, min_max, multi_min_max
from utils.colour_palette import COLOURS, NUM_COLOURS
from asciimatics.screen import Screen
from asciimatics.effects import Effect

# from typing import TypedDict, List
import attrs

@attrs.define
class GraphData:
    x_values: list = attrs.field(default=[])
    y_values: list[list] = attrs.field(default=[])
    colours: list = attrs.field(default=[])
    paused: bool = attrs.field(default=False)

MAX_GRAPH_PTS = 1000

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
        self._edited = []
        self._last_len = -1

    def reset(self):
        self._screen.clear()
    @property
    def stop_frame(self):
        return 10**12  # arbitrarily long duration  

    def custom_print(self, string, x, y, colour=7):
        self._edited.append(([len(s) for s in string.split("\n")],x,y))
        self._screen.print_at(string, x, y, colour)
    
    def custom_clear(self):
        for ls, x, y in self._edited:
            self._screen.print_at("\n".join([" "*n for n in ls]), x, y)
        self._edited = []
    
    def _update(self, frame_no):
        n = len(self._data.x_values)
        if self._data.paused or self._last_len == len(self._data.x_values) or n == 0:
            #self.custom_print("NO UPDATE", self._x_origin+self._width//2, self._y_origin+self._height//2)
            return
        self._last_len = n
        self.custom_clear()
        # self._screen.clear()
        # self.custom_print("DO UPDATE", self._x_origin+self._width//2, self._y_origin+self._height//2)
        if n < MAX_GRAPH_PTS:
            self._draw(self._x_origin, self._y_origin, self._data.x_values, self._data.y_values, self._data.colours)
        else:
            self._draw(self._x_origin, self._y_origin, self._data.x_values[-MAX_GRAPH_PTS:], [y[-MAX_GRAPH_PTS:] for y in self._data.y_values], self._data.colours)

    def _draw(self, x_draw: int, y_draw: int, x_values: list, y_values: list[list], colours: list):
        if not all(isinstance(x, (int, float)) for x in x_values) and not (all(isinstance(y, (int, float)) for y in y_data) for y_data in y_values):
            raise TypeError("All elements must be numeric")
        
        if len(colours) != len(y_values):
            raise ValueError("Colors and Y axis data must be of same length")

        if not all(len(x_values) == len(y) for y in y_values):
            raise ValueError(f"X({len(x_values)}) and Y ({len(y_values[0])}) axis data must be of same length")

        if len(x_values) == 0:
            raise ValueError(f"X({len(x_values)}) and Y ({len(y_values[0])}) axis data must be of same length")
        
        x_val_min, x_val_max = min_max(x_values)
        y_val_min, y_val_max = multi_min_max(y_values)
        # self.custom_print(f"{x_values}, min x {x_val_min}, max x: {x_val_max}", 0, 0)
        
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
        
        #---- draw axes
        y_axis_location = get_mapped_value(0 if x_val_min < 0 else x_val_min, x_val_max, self._width-1, x_val_min, 0)
        x_axis_location = get_mapped_value(0 if y_val_min < 0 else y_val_min, y_val_max, 0, y_val_min, self._height-1)
        self._y_axis.draw(self._screen, x_draw+y_axis_location, y_draw, print_fn=self.custom_print)
        self._x_axis.draw(self._screen, x_draw, y_draw+x_axis_location, print_fn=self.custom_print)

        #---- y label location on x
        y_label_x_location = max(x_draw, x_draw+y_axis_location-2)

        for i in range(len(y_values)):
            c = colours[i]
            #---- update plot data
            self._plot.plot(x_values, y_values[i], x_val_min, x_val_max, y_val_min, y_val_max)
            #---- draw stuff
            self._plot.draw(self._screen, x_draw, y_draw, colour=c, print_fn=self.custom_print)
            #---- draw latest y value label
            y_latest_location = get_mapped_value(y_values[i][-1], y_val_max, 0, y_val_min, self._height-1)
            self.custom_print(f"{y_values[i][-1]:3.2f}", y_label_x_location, y_draw+y_latest_location, colour=c)


        # #---- draw labels. only print latest, min, max
        # for y_data in y_values:
        #     y_latest_location = get_mapped_value(y_data[-1], y_val_max, 0, y_val_min, self._height-1)
        #     self.custom_print(f"{y_data[-1]:3.2f}", y_label_x_location, y_draw+y_latest_location)

        #---- draw remaining y labels -> min and max values
        self.custom_print(f"{y_val_max:3.2f}", y_label_x_location, y_draw)
        self.custom_print(f"{y_val_min:3.2f}", y_label_x_location, y_draw+self._height-1)

        #---- draw x labels
        x_latest_location = get_mapped_value(x_values[-1], x_val_max, self._width-1, x_val_min, 0)
        x_label_y_location = min(y_draw+self._height-1, y_draw+x_axis_location+1)
        self.custom_print(f"{x_values[-1]:3.2f}", x_draw+x_latest_location, x_label_y_location)
        self.custom_print(f"{x_val_max:3.2f}", x_draw+self._width-1, x_label_y_location)
        if x_val_min != 0:
            self.custom_print(f"{x_val_min:3.2f}", x_draw, x_label_y_location)


        
