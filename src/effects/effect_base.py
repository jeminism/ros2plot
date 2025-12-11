
from asciimatics.screen import Screen
from asciimatics.effects import Effect
import attrs

class DrawOffsets:
    x: int = attrs.field(default=0)
    y: int = attrs.field(default=0)

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
        self._edited.append(([len(s) for s in string.split("\n")],x,y))
        self._screen.print_at(string, x, y, colour)
    
    def e_clear(self):
        for ls, x, y in self._edited:
            self._screen.print_at("\n".join([" "*n for n in ls]), x, y)
        self._edited = []

    def _update(self, frame_no):
        self.e_clear()
        self._draw(frame_no)
    
    def _draw(self, frame_no):
        return