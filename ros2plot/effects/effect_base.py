
from ..utils import GraphConfigs

from asciimatics.screen import Screen
from asciimatics.effects import Effect

import attrs

@attrs.define
class DrawOffsets:
    x: int = attrs.field(default=0)
    y: int = attrs.field(default=0)

# speclization to draw on screen with respect to input offsets. Utility functions to automatically clear its own trash only to clean up the screen
class EffectBase(Effect):
    def __init__(self, screen: Screen, offsets: DrawOffsets):
        super().__init__(screen)
        self._edited = []
        self._offsets = offsets
    
    def reset(self):
        self._screen.clear()

    def stop_frame(self):
        return -1
    
    def e_print(self, string, x_in, y_in, colour=7):
        x = x_in + self._offsets.x
        y = y_in + self._offsets.y
        self._edited.append(([s for s in string.split("\n")],x,y,colour))
        self._screen.print_at(string, x, y, colour)
    
    def e_clear(self):
        for ls, x, y, c in self._edited:
            self._screen.print_at("\n".join([" "*len(n) for n in ls]), x, y)
        self._edited = []

    def _update(self, frame_no):
        self.e_clear()
        self._draw(frame_no)
    
    def _draw(self, frame_no):
        return


# minor specialization to store graph config
class GraphEffect(EffectBase):
    def __init__(self, screen: Screen, cfg: GraphConfigs, offsets: DrawOffsets=DrawOffsets(), redraw_on_pause: bool=True):
        super().__init__(screen, offsets)
        self._cfg = cfg
        self._redraw_on_pause = redraw_on_pause

    def _redraw(self):
        for ls, x, y, c in self._edited:
            self._screen.print_at("\n".join([n for n in ls]), x, y, c)
        
    def _update(self, frame_no):
        if self._cfg.pause and self._redraw_on_pause:
            self._redraw()
        else:
            super()._update(frame_no)
