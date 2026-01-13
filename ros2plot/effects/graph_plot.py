
from ..utils import get_mapped_value, bresenham, braille_char, GraphConfigs, PlotData, Grid
from .effect_base import GraphEffect, DrawOffsets

from asciimatics.screen import Screen

import math
import attrs
import time

# define a scanline run as a compressed representation of a continuous segment of raw data which occupies the same column
@attrs.define
class ScanlineRun:
    column_index: int = None
    first: float = None
    last: float = None
    minimum: float = math.inf
    maximum: float = -math.inf

class Plot(GraphEffect):
    def __init__(self, screen: Screen, cfg: GraphConfigs, db: dict[str, PlotData], y_key:str=None, offsets: DrawOffsets=DrawOffsets(), debug_fn=None):
        super().__init__(screen, cfg, offsets)
        self._db = db #dictionary db of field vs field data
        self._plt = None
        self._y_key = None
        self.set_data_key(y_key)
        self._debug_fn = debug_fn

        self._scanlines = [] # store parsed scanlines
        self._grid = None # ground truth buffer for screen refresh
        self._plot_cell_buffer = [] # buffer of grid cells which need to be re-rendered every frame

        # variables needed for minimal scanline updates at each timestep
        self._last_data_index = None
        self._last_scanline_index = None
        self._scanline_resolution = None
        self._last_x_min = None
        self._last_x_max = None
        self._last_y_min = None
        self._last_y_max = None

        self._counts = [0,0,0]
    
    @property 
    def plot_width(self):
        return self._cfg.width*2 if self._plt.high_def else self._cfg.width #braille conversion
    
    @property
    def plot_height(self):
        return self._cfg.height*4 if self._plt.high_def else self._cfg.height #braille conversion
    
    def debug_print(self, s):
        if self._debug_fn == None:
            return
        self._debug_fn(s)

    def set_data_key(self, y_data: str):
        if y_data not in self._db:
            raise ValueError(f"Unable to plot graph with Y-Axis data {y_data} but this key does not exist in the DB!")
        self._plt = self._db[y_data]
    
    def lookup_data(self, key):
        return self._db[key].data
    
    def _draw(self, frame_no):
        start_time = time.time()

        if self._plt == None:
            # not setup yet, just quietly return as it may be due to application logic
            return

        if self._plt.x_key not in self._db:
            raise ValueError(f"Unable to plot graph with X-Axis data {self._plt.x_key} but this key does not exist in the DB!")

        y_data = self._plt.data
        x_data = self.lookup_data(self._plt.x_key)
        data_size = len(x_data)
        data_time = time.time() - start_time
        if data_size == 0:
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

        # data_time = time.time() - start_time
        start_time = time.time()
        generate_scanlines_time = 0

        redraw_all = False
        if self.x_bounds_changed() or self.scanline_shift_needed():
            # self._counts[0] += 1
            self._scanlines.clear()
            self.generate_scanlines(y_data, x_data)
            generate_scanlines_time = time.time()-start_time
            # self._last_data_index = data_size-1
            self._scanline_resolution = (self._cfg.x_max_value - self._cfg.x_min_value) / self.plot_width
            self._plot_cell_buffer.clear()
            redraw_all = True
        elif self.y_bounds_changed():
            # no need to clear scanlines here, but we need to regenerate the plot buffer
            self._plot_cell_buffer.clear()
            redraw_all = True
        
        
        self.generate_scanlines(y_data.latest(), x_data.latest())

        # if self._last_data_index < data_size-1:
        #     # self._counts[2] += 1
        #     self.generate_scanlines(y_data.latest(), x_data.latest())
        #     self._last_data_index = data_size-1
        

        self._last_x_min = self._cfg.x_min_value
        self._last_x_max = self._cfg.x_max_value

        start_time = time.time()
        self.update_grid(refresh_grid=redraw_all)
        update_time = time.time() - start_time
        
        start_time = time.time()
        self.do_plot()

        # self.debug_print(f"scanlines size: {len(self._scanlines)}, data : {data_time:.5f}, update scanlines : {generate_scanlines_time:.5f}, grid update : {update_time:.5f}, end-end : {time.time() - start_time:.5f}")
    
    def generate_scanlines(self, y_data, x_data, start_index=0):
        width = self.plot_width
        # for i in range(start_index, len(x_data)):
        #     x = x_data[i]
        #     y = y_data[i]
        for x,y in zip(x_data, y_data):
            # get the x index. this will be the column index.
            x_index = get_mapped_value(x, self._cfg.x_max_value, width-1, self._cfg.x_min_value, 0)
            if x_index > width-1 or x_index < 0:
                continue

            # initialize a new scanline if empty or if not matching
            if len(self._scanlines) == 0 or self._scanlines[-1].column_index != x_index:
                self._scanlines.append(ScanlineRun())
                self._scanlines[-1].column_index = x_index
                
            latest = self._scanlines[-1]

            # update the scanline with this latest y
            if latest.first == None:
                latest.first = y
            if y > latest.maximum:
                latest.maximum = y
            if y < latest.minimum:
                latest.minimum = y
            latest.last = y

        # self.debug_print(f"{counts}, scanlines size: {len(self._scanlines)}. width: {width}")
    
    
    def shift_scanlines(self):
        delta = math.ceil((self._cfg.x_max_value - self._cfg.x_min_value) / self._scanline_resolution - self.plot_width)
        
        for scanline in self._scanlines:
            scanline.column_index = scanline.column_index-delta if scanline.column_index > delta else 0


    def scanline_shift_needed(self):
        # using the previous column resolution, check if a new column needs to be created. if so, then it is time to shift the existing scanlines left
        return ((self._cfg.x_max_value - self._cfg.x_min_value) / self._scanline_resolution - self.plot_width) > 0.9
                           
    def x_bounds_changed(self):
        return self._last_x_min != self._cfg.x_min_value or self._last_x_max > self._cfg.x_max_value
    
    def y_bounds_changed(self):
        return self._last_y_min != self._cfg.y_min_value or self._last_y_max != self._cfg.y_max_value
    
    def update_grid(self, refresh_grid:bool=True):
        if refresh_grid:
            #reinitialize the grid
            self._grid = Grid(self.plot_width, self.plot_height)
            self._last_scanline_index = 0

        width = self.plot_width
        height = self.plot_height
        prior_x = None
        prior_y = None
        changed_cells = []
        grid = self._grid
        for scanline in self._scanlines[self._last_scanline_index:]:
            x_index = scanline.column_index
            # note, these y values are automatically clipped by the check in bresenham later
            y_first = get_mapped_value(scanline.first, self._cfg.y_max_value, 0, self._cfg.y_min_value, height-1)
            y_last = get_mapped_value(scanline.last, self._cfg.y_max_value, 0, self._cfg.y_min_value, height-1)
            y_min = get_mapped_value(scanline.minimum, self._cfg.y_max_value, 0, self._cfg.y_min_value, height-1)
            y_max = get_mapped_value(scanline.maximum, self._cfg.y_max_value, 0, self._cfg.y_min_value, height-1)

            # draw connecting line from the previous scanline.last
            if self._plt.interpolate and prior_x != None and prior_y != None:
                for pt in bresenham(x_index, y_first, prior_x, prior_y):
                    if pt[0] > width-1 or pt[0] < 0 or pt[1] > height-1 or pt[1] < 0:
                        continue
                    i = grid.to_index(pt[0], pt[1])
                    if grid.at(i) != True:
                        self._plot_cell_buffer.append((pt[0], pt[1]))
                    grid.set_value(i, True)
            
            # draw line within column
            for pt in bresenham(x_index, y_min, x_index, y_max):
                if pt[0] > width-1 or pt[0] < 0 or pt[1] > height-1 or pt[1] < 0:
                    continue
                i = grid.to_index(pt[0], pt[1])
                if grid.at(i) != True:
                    self._plot_cell_buffer.append((pt[0], pt[1]))
                grid.set_value(i, True)

            prior_x = x_index
            prior_y = y_last
        
        #update the last scanline index done
        self._last_scanline_index = len(self._scanlines)-1
        
    
    def do_plot(self):
        grid = self._grid
        # self.debug_print(f"plotting {len(changed_grid_cells)} grid cells")
        done_ledger = set() # set to store done cells so we ignore overlaps

        dot_offsets = [
            (0, 0), (0, 1), (0, 2),
            (1, 0), (1, 1), (1, 2),
            (0, 3), (1, 3),
        ]

        for i,j in self._plot_cell_buffer:

            if self._plt.high_def:
                # get root cell in screen coordinates first, then find the braille character corresponding to this screen cell
                x = i//2
                y = j//4
                index = x + y*self.plot_width
                if index in done_ledger:
                    continue
                done_ledger.add(index)

                bx = x*2
                by = y*4
                braille_cells = []

                for dot_num, (dx, dy) in enumerate(dot_offsets, start=1):
                    if grid.at(grid.to_index(bx + dx, by + dy)):
                        braille_cells.append(dot_num)
                self.e_print(braille_char(braille_cells), x, y, self._plt.colour)
            else:
                index = i + j*self.plot_width
                if index in done_ledger:
                    continue
                done_ledger.add(index)
                self.e_print("*", i, j, self._plt.colour)

        last_x_index = self._scanlines[-1].column_index//2 if self._plt.high_def else self._scanlines[-1].column_index
        last_y_index = get_mapped_value(self._scanlines[-1].last, self._cfg.y_max_value, 0, self._cfg.y_min_value, self._cfg.height)
        if last_y_index < self._cfg.height+1 and last_y_index >= 0:
            self.e_print(f"{self._scanlines[-1].last:3.2f}", last_x_index+1, last_y_index, self._plt.colour)
