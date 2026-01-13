


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
        self._inspection_data = {} #dict of field name : [[GraphPoint] by column]
        self._matched_pts = {} #dict of field name : GraphPoint
        self._last_x_min = None
        self._last_x_max = None
        self._last_y_min = None
        self._last_y_max = None
        self._moved = False

    def _draw(self, frame_no):
        if self._x_value == None:
            return
        x = get_mapped_value(self._x_value, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
        for y in range(self._cfg.height):
            self.e_print("│", x, y)
        
        if self.resized():
            self._inspection_data.clear() # regenerate these on any resize
        self._last_x_min = self._cfg.x_min_value
        self._last_x_max = self._cfg.x_max_value
        self._last_y_min = self._cfg.y_min_value
        self._last_y_max = self._cfg.y_max_value

        new = []
        # just check if visibility has changed. update the data chunks as needed for visible graphs
        for field_name, plot in self._plot_data.items():
            if len(plot.data) == 0:
                #return # why return?
                continue

            if not plot.visible:
                if field_name in self._inspection_data:
                    del self._inspection_data[field_name]
                    del self._matched_pts[field_name]
                continue

            x_key = plot.x_key

            if x_key not in self._plot_data:
                continue
                
            if field_name not in self._inspection_data:
                self._inspection_data[field_name]  = [[]]*self._cfg.width
                self._matched_pts[field_name] = None
                new.append(field_name)
                self.populate_column_vals(plot.data.values(), self._plot_data[x_key].data.values(), self._inspection_data[field_name])
            else: #field name in inspection_data
                self.populate_column_vals(plot.data.latest(), self._plot_data[x_key].data.latest(), self._inspection_data[field_name])
                if self._moved:
                    new.append(field_name)

            #self.print_values_at_x(plot.data.values(), self._plot_data[x_key].data.values(), plot.colour)
        #only update the ones marked as new
        for field_name in new:
            self.get_matched_pt_at_x(field_name)
        
        self.print_matched_pts()
        self._moved = False
        
    def resized(self):
        return not (self._last_x_min == self._cfg.x_min_value and self._last_x_max == self._cfg.x_max_value and self._last_y_min == self._cfg.y_min_value and self._last_y_max == self._cfg.y_max_value)
    
    def set_x_value(self, x_val=None):
        self._x_value = x_val if x_val != None else (self._cfg.x_max_value + self._cfg.x_min_value) / 2
    
    def get_x_value(self):
        return self._x_value
    
    def tooltip(self):
        return f"← : Move Left | → : Move Right | CTRL+move : Move slower"
    
    def populate_column_vals(self, y_values:list, x_values:list, column_ref:list[list]):
        for x,y in zip(x_values, y_values):
            col_id = get_mapped_value(x, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
            # y_pix = get_mapped_value(y_val, self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)
            if col_id < 0 or col_id >= self._cfg.width:
                continue
            column_ref[col_id].append(GraphPoint(x,y))
    
    def get_matched_pt_at_x(self, field_name):
        col_id = get_mapped_value(self._x_value, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
        field_columns = self._inspection_data[field_name]
        pts = field_columns[col_id]
        i = 0
        ok = True
        #extend the search to the surrounding columns to find the best one
        while len(pts) == 0 and ok:
            i+=1
            lower = col_id-i
            higher = col_id+i
            ok = False
            if lower >= 0:
                pts += field_columns[lower]
                ok = True
            if higher < self._cfg.width:
                pts += field_columns[higher]
                ok = True

        #break early if no points.
        if len(pts) == 0:
            return
        
        # find the single best point
        err = math.inf
        best = None
        for pt in pts:
            tmp = abs(self._x_value - pt.x)
            if tmp < err:
                err = tmp
                best = pt
        
        # add it to the best pts
        if best != None:
            self._matched_pts[field_name] = best

    def print_matched_pts(self):
        for field_name, pt in self._matched_pts.items():
            x_pix = get_mapped_value(pt.x, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
            y_pix = get_mapped_value(pt.y, self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)
            self.e_print(f"⮾ {pt.y}", x_pix, y_pix, self._plot_data[field_name].colour)

    # def print_values_at_x(self, y_data, x_data, colour=7):
    #     ref_x_pix = get_mapped_value(self._x_value, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
    #     err = math.inf
    #     best = None
    #     found = False

    #     for pt in zip(x_data, y_data):
    #         x_pix = get_mapped_value(x_val, self._cfg.x_max_value, self._cfg.width-1, self._cfg.x_min_value, 0)
    #         y_pix = get_mapped_value(y_val, self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)
    #         if x_pix < 0 or x_pix >= self._cfg.width or y_pix < 0 or y_pix > self._cfg.height:
    #             continue

    #         if x_pix == ref_x_pix:
    #             self.e_print(f"⮾ {y_val}", x_pix, y_pix, colour)
    #             found = True

    #         if found:
    #             continue
    #         tmp = abs(self._x_value - x_val)
    #         if tmp < err:
    #             err = tmp
    #             best = (y_val, x_pix, y_pix)

    #     if not found and best != None:
    #         self.e_print(f"⮾ {best[0]}", best[1], best[2], colour)

    
    def scroll_up_x(self, step=100):
        d = (self._cfg.x_max_value - self._cfg.x_min_value) / step
        n_val = self._x_value + d
        if self._x_value < n_val:
            return min(n_val, self._cfg.x_max_value)
        else:
            return max(n_val, self._cfg.x_max_value)
        

    def scroll_down_x(self, step=100):
        d = (self._cfg.x_max_value - self._cfg.x_min_value) / step
        n_val = self._x_value - d
        if self._x_value < n_val:
            return min(n_val, self._cfg.x_min_value)
        else:
            return max(n_val, self._cfg.x_min_value)
    
    def scroll_x(self, up:bool, step):
        res = self.scroll_up_x(step) if up else self.scroll_down_x(step)
        if res != self._x_value:
            self._moved = True
        self._x_value = res
        

    def process_event(self, event):
        if isinstance(event, KeyboardEvent):
            if event.key_code == KEY_CODES.LEFT: # LEFT ARROW
                self.scroll_x(False, self._cfg.width/2)
                return None
            if event.key_code == KEY_CODES.RIGHT: # RIGHT ARROW
                self.scroll_x(True, self._cfg.width/2)
                return None
            if event.key_code == KEY_CODES.CTRL_LEFT: # CTRL + LEFT ARROW
                self.scroll_x(False, self._cfg.width*10)
                return None
            if event.key_code == KEY_CODES.CTRL_RIGHT: # CTRL + RIGHT ARROW
                self.scroll_x(True, self._cfg.width*10)
                return None

        return event
        



