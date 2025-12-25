# from drawables.lines import VerticalLine, HorizontalLine
# from drawables.plot import PlotBraille, PlotXY
from utils.graph_math import get_mapped_value, min_max, multi_min_max, bresenham
from utils.braille import braille_char
from utils.colour_palette import COLOURS, NUM_COLOURS
from utils.grid import Grid

from asciimatics.screen import Screen
from effects.effect_base import EffectBase, DrawOffsets
from asciimatics.event import KeyboardEvent, MouseEvent

import utils.key_codes as KEY_CODES
from utils.graph_data import GraphConfigs, PlotData

# from typing import TypedDict, List
import attrs
import math

MAX_GRAPH_PTS = 1000

#helper class
class GraphPoint():
    def __init__(self, x, y):
        self._x = x
        self._y = y
    @property
    def x(self):
        return self._x
    @x.setter
    def x(self, new_x):
        self._x = new_x
    @property
    def y(self):
        return self._y
    @y.setter
    def y(self, new_y):
        self._y = new_y
    def set(self, x, y):
        self._x = x
        self._y = y

class GraphEffect(EffectBase):
    def __init__(self, screen: Screen, cfg: GraphConfigs, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, offsets)
        self._cfg = cfg

class GraphZoomSelector(GraphEffect):
    def __init__(self, screen: Screen, cfg: GraphConfigs, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, cfg, offsets)
        self.pt_1 = GraphPoint(self._cfg.x_min_value, self._cfg.y_min_value)
        self.pt_2 = GraphPoint(self._cfg.x_max_value, self._cfg.y_max_value)
        self._focus = 0
    
    def reset(self):
        # x_quat = abs(self._cfg.x_max_value - self._cfg.x_min_value) / 4
        # y_quat = abs(self._cfg.y_max_value - self._cfg.y_min_value) / 4
        # self.pt_1.set(self._cfg.x_min_value+x_quat, self._cfg.y_min_value+y_quat)
        # self.pt_2.set(self._cfg.x_max_value-x_quat, self._cfg.y_max_value-y_quat)
        self.pt_1.set(self._cfg.x_min_value, self._cfg.y_min_value)
        self.pt_2.set(self._cfg.x_max_value, self._cfg.y_max_value )
        self._focus = 0

    def _draw(self, frame_no):
        x_1_index = get_mapped_value(self.pt_1.x, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
        y_1_index = get_mapped_value(self.pt_1.y, self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)
        x_2_index = get_mapped_value(self.pt_2.x, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
        y_2_index = get_mapped_value(self.pt_2.y, self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)

        y_min, y_max = min_max([y_1_index, y_2_index])
        x_min, x_max = min_max([x_1_index, x_2_index])
        if y_min != y_max:
            for y in range(y_min, y_max):
                self.e_print("╎", x_min, y)
                self.e_print("╎", x_max, y)
        if x_min != x_max:
            for x in range(x_min, x_max):
                self.e_print("╌", x, y_min)
                self.e_print("╌", x, y_max)
        
        
        self.e_print("X" if self._focus!=2 else "x", x_1_index, y_1_index, COLOURS[1] if self._focus!=2 else 7)
        self.e_print("X" if self._focus!=1 else "x", x_2_index, y_2_index, COLOURS[1] if self._focus!=1 else 7)
        self.e_print("┌", x_min, y_min)
        self.e_print("┘", x_max, y_max)
    
    def get_points_string(self):
        return f"point_1: [{self.pt_1.x:5}, {self.pt_1.y:5}], point_2: [{self.pt_2.x:5}, {self.pt_2.y:5}]"
        
    def tooltip(self):
        return f"TAB : Cycle point control | ↑←↓→ : move controlled points | CTRL+move : Move slower"

    def scroll_up_x(self, step=100):
        d = (self._cfg.x_max_value - self._cfg.x_min_value) / step
        if self._focus!=2:
            self.move_pt(self.pt_1, d, 0)
        if self._focus!=1:
            self.move_pt(self.pt_2, d, 0)
    
    def scroll_down_x(self, step=100):
        d = (self._cfg.x_max_value - self._cfg.x_min_value) / step
        if self._focus!=2:
            self.move_pt(self.pt_1, -d, 0)
        if self._focus!=1:
            self.move_pt(self.pt_2, -d, 0)

    def scroll_up_y(self, step=100):
        d = (self._cfg.y_max_value - self._cfg.y_min_value) / step
        if self._focus!=2:
            self.move_pt(self.pt_1, 0, d)
        if self._focus!=1:
            self.move_pt(self.pt_2, 0, d)
    
    def scroll_down_y(self, step=100):
        d = (self._cfg.y_max_value - self._cfg.y_min_value) / step
        if self._focus!=2:
            self.move_pt(self.pt_1, 0, -d)
        if self._focus!=1:
            self.move_pt(self.pt_2, 0, -d)

    def move_pt(self, point, dx, dy):
        cand_x = point.x + dx
        cand_y = point.y + dy
        if cand_x < self._cfg.x_min_value:
            self._cfg.x_min_value = cand_x
            # point.x = self._cfg.x_min_value
        elif cand_x > self._cfg.x_max_value:
            self._cfg.x_max_value = cand_x
            # point.x = self._cfg.x_max_value
        # else:
        
        point.x = cand_x
            
        if cand_y < self._cfg.y_min_value:
            self._cfg.y_min_value = cand_y
            # point.y = self._cfg.y_min_value
        elif cand_y > self._cfg.y_max_value:
            self._cfg.y_max_value = cand_y
            # point.y = self._cfg.y_max_value
        # else:
            # point.y = cand_y
        point.y = cand_y
    
    def resize_plot(self):
        self._cfg.y_min_value, self._cfg.y_max_value = min_max([self.pt_1.y, self.pt_2.y])
        self._cfg.x_min_value, self._cfg.x_max_value = min_max([self.pt_1.x, self.pt_2.x])
        

    def process_event(self, event):
        if isinstance(event, KeyboardEvent):
            if event.key_code == KEY_CODES.ENTER: # ENTER
                self.resize_plot()
                return None
            if event.key_code == KEY_CODES.TAB: # TAB
                self._focus = (self._focus+1)%3
                return None
            if event.key_code == KEY_CODES.LEFT: # LEFT ARROW
                self.scroll_down_x(self._cfg.width/2)
                return None
            if event.key_code == KEY_CODES.RIGHT: # RIGHT ARROW
                self.scroll_up_x(self._cfg.width/2)
                return None
            if event.key_code == KEY_CODES.CTRL_LEFT: # CTRL + LEFT ARROW
                self.scroll_down_x(self._cfg.width*10)
                return None
            if event.key_code == KEY_CODES.CTRL_RIGHT: # CTRL + RIGHT ARROW
                self.scroll_up_x(self._cfg.width*10)
                return None
            if event.key_code == KEY_CODES.UP: # UP ARROW
                self.scroll_up_y(self._cfg.height)
                return None
            if event.key_code == KEY_CODES.DOWN: # DOWN ARROW
                self.scroll_down_y(self._cfg.height)
                return None
            if event.key_code == KEY_CODES.CTRL_UP: # CTRL + UP ARROW
                self.scroll_up_y(self._cfg.height*6)
                return None
            if event.key_code == KEY_CODES.CTRL_DOWN: # CTRL + DOWN ARROW
                self.scroll_down_y(self._cfg.height*6)
                return None

        return event

class GraphInspector(GraphEffect):
    def __init__(self, screen: Screen, cfg: GraphConfigs, plot_data: dict[str, PlotData], initial_x_value=None, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, cfg, offsets)
        self._plot_data = plot_data
        # self._plot_visibility = plot_visibility
        # self._x_key = x_key
        self._x_value = initial_x_value

    def _draw(self, frame_no):
        if self._x_value == None:
            return
        #print line indicating current scroll position
        x = get_mapped_value(self._x_value, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
        for y in range(self._cfg.height):
            self.e_print("│", x, y)

        for field_name, plot in self._plot_data.items():
            if len(plot.data) == 0:
                #return # why return?
                continue

            if not plot.visible:
                continue

            x_key = plot.x_key

            if x_key not in self._plot_data:
                continue

            index = self.get_closest_index(self._x_value, self._plot_data[x_key].data)
            x_data = get_mapped_value(self._plot_data[x_key].data[index], self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
            y_val = plot.data[index]
            y_data = get_mapped_value(y_val, self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)
            self.e_print(f"⮾ {y_val}", x_data, y_data, plot.colour)
    
    def set_x_value(self, x_val=None):
        self._x_value = x_val if x_val != None else (self._cfg.x_max_value + self._cfg.x_min_value) / 2
    
    def get_x_value(self):
        return self._x_value

    # def set_x_key(self, x_key):
    #     self._x_key = x_key
    
    def tooltip(self):
        return f"← : Move Left | → : Move Right | CTRL+move : Move slower"
    
    def get_closest_index(self, val, data):
        err = math.inf
        res = -1
        for i in range(len(data)):
            tmp = abs(data[i] - val)
            if tmp < err:
                err = tmp
                res = i
            if tmp > err:
                break #break early, just do first match
        return res
    
    def scroll_up_x(self, step=100):
        d = (self._cfg.x_max_value - self._cfg.x_min_value) / step
        n_val = self._x_value + d
        if self._x_value < n_val:
            self._x_value = min(n_val, self._cfg.x_max_value)
        else:
            self._x_value = max(n_val, self._cfg.x_max_value)
        

    def scroll_down_x(self, step=100):
        d = (self._cfg.x_max_value - self._cfg.x_min_value) / step
        n_val = self._x_value - d
        if self._x_value < n_val:
            self._x_value = min(n_val, self._cfg.x_min_value)
        else:
            self._x_value = max(n_val, self._cfg.x_min_value)
        

    def process_event(self, event):
        if isinstance(event, KeyboardEvent):
            if event.key_code == KEY_CODES.LEFT: # LEFT ARROW
                self.scroll_down_x(self._cfg.width/2)
                return None
            if event.key_code == KEY_CODES.RIGHT: # RIGHT ARROW
                self.scroll_up_x(self._cfg.width/2)
                return None
            if event.key_code == KEY_CODES.CTRL_LEFT: # CTRL + LEFT ARROW
                self.scroll_down_x(self._cfg.width*10)
                return None
            if event.key_code == KEY_CODES.CTRL_RIGHT: # CTRL + RIGHT ARROW
                self.scroll_up_x(self._cfg.width*10)
                return None

        return event
        




class YAxis(GraphEffect):
    
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

class XAxis(GraphEffect):
    
    def _draw(self, frame_no):
        if self._cfg.width == 0:
            raise ValueError("Tried to draw XAxis with length = 0!")
        for i in range(self._cfg.width):
            self.e_print("-", i, self._cfg.y)
        # draw min and max labels
        self.e_print(f"{self._cfg.x_min_value:3.2f}", 0, self._cfg.y+1)
        self.e_print(f"{self._cfg.x_max_value:3.2f}", self._cfg.width-1, self._cfg.y+1)

class Plot(GraphEffect):
    def __init__(self, screen: Screen, cfg: GraphConfigs, db: dict[str, PlotData], y_key:str=None, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, cfg, offsets)
        self._db = db #dictionary db of field vs field data
        self._plt = None
        self._y_key = None
        self.set_data_key(y_key)

    def set_data_key(self, y_data: str):
        if y_data not in self._db:
            raise ValueError(f"Unable to plot graph with Y-Axis data {y_data} but this key does not exist in the DB!")
        self._plt = self._db[y_data]
    
    def lookup_data(self, key):
        return self._db[key].data
    
    def _draw(self, frame_no):
        if self._plt == None:
            # not setup yet, just quietly return as it may be due to application logic
            return

        if self._plt.x_key not in self._db:
            raise ValueError(f"Unable to plot graph with X-Axis data {self._plt.x_key} but this key does not exist in the DB!")

        y_data = self._plt.data
        x_data = self.lookup_data(self._plt.x_key)

        if len(x_data) == 0:
            return
            
        if not all(isinstance(x, (int, float)) for x in x_data) and not all(isinstance(y, (int, float)) for y in y_data):
            raise TypeError("All elements must be numeric")
        
        if len(x_data) != len(y_data):
            # return #just fail instead in the cas of mismatched x and y values
            raise ValueError(f"X and Y axis data must be of same length. got X length: {len(x_data)}, Y length: {len(y_data)}")
        
        if self._cfg.x_min_value == self._cfg.x_max_value:
            raise ValueError("X axis bound invalid! Min value == max value")
        if self._cfg.y_min_value == self._cfg.y_max_value:
            raise ValueError("Y axis bound invalid! Min value == max value")

        self.do_plot(y_data, x_data)


    def do_plot(self, y_data, x_data):
        n_vals = len(x_data)
        use_braille = self._plt.high_def

        width = self._cfg.width*2 if use_braille else self._cfg.width
        height = self._cfg.height*4 if use_braille else self._cfg.height

        grid = Grid(width, height)
        prior_x = -1
        prior_y = -1
        last = -1
        for i in range(n_vals):
            y_index = get_mapped_value(y_data[i], self._cfg.y_max_value, 0, self._cfg.y_min_value, height-1) # flipped min and max because asciimatics y=0 is the topmost row of terminal.
            x_index = get_mapped_value(x_data[i], self._cfg.x_max_value, width-1, self._cfg.x_min_value, 0)
            if x_index > width-1 or x_index < 0 or y_index > height-1 or y_index < 0:
                continue

            if self._plt.interpolate:
                for pt in bresenham(x_index, y_index, prior_x, prior_y):
                    if pt[0] > width-1 or pt[0] < 0 or pt[1] > height-1 or pt[1] < 0:
                        continue
                    grid.set_value(grid.to_index(pt[0], pt[1]), True)
            else:
                grid.set_value(grid.to_index(x_index, y_index), True)
            prior_x = x_index
            prior_y = y_index
            last = i
        
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
                        self.e_print(braille_char(braille_cells), i, j, self._plt.colour)
                else:
                    if grid.at(grid.to_index(i, j)):
                        self.e_print("*", i, j, self._plt.colour)

        # print last value
        if last != -1:
            x_latest_location = min(get_mapped_value(x_data[last], self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0), self._cfg.width)
            y_latest_location = get_mapped_value(y_data[last], self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)
            self.e_print(f"{y_data[last]:3.2f}", x_latest_location+1, y_latest_location, self._plt.colour)