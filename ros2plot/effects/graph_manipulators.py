


from ..utils import get_mapped_value, min_max, GraphConfigs, PlotData, KEY_CODES, COLOURS
from .effect_base import GraphEffect, DrawOffsets

from asciimatics.screen import Screen
from asciimatics.event import KeyboardEvent, MouseEvent

import math

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

class GraphZoomSelector(GraphEffect):
    def __init__(self, screen: Screen, cfg: GraphConfigs, offsets: DrawOffsets=DrawOffsets()):
        super().__init__(screen, cfg, offsets, redraw_on_pause=False)
        self.pt_1 = GraphPoint(self._cfg.x_min_value, self._cfg.y_min_value)
        self.pt_2 = GraphPoint(self._cfg.x_max_value, self._cfg.y_max_value)
        self._focus = 0
    
    def reset(self):
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
        
        
        self.e_print("X" if self._focus!=2 else "x", x_1_index, y_1_index, COLOURS.GREEN if self._focus!=2 else COLOURS.DEFAULT)
        self.e_print("X" if self._focus!=1 else "x", x_2_index, y_2_index, COLOURS.GREEN if self._focus!=1 else COLOURS.DEFAULT)
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
        elif cand_x > self._cfg.x_max_value:
            self._cfg.x_max_value = cand_x
        point.x = cand_x
            
        if cand_y < self._cfg.y_min_value:
            self._cfg.y_min_value = cand_y
        elif cand_y > self._cfg.y_max_value:
            self._cfg.y_max_value = cand_y
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
        super().__init__(screen, cfg, offsets, redraw_on_pause=False)
        self._plot_data = plot_data
        self._x_value = initial_x_value

    def _draw(self, frame_no):
        if self._x_value == None:
            return
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
        



