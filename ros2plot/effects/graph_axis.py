
from .effect_base import GraphEffect

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
