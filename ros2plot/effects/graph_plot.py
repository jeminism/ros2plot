
from ..utils import get_mapped_value, bresenham, braille_char, GraphConfigs, PlotData, Grid
from .effect_base import GraphEffect, DrawOffsets

from asciimatics.screen import Screen


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

            if self._plt.interpolate and last != -1:
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