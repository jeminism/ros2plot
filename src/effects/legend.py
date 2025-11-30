
from asciimatics.effects import Effect
from asciimatics.widgets.layout import Layout
from asciimatics.widgets.checkbox import CheckBox
from asciimatics.widgets.text import Text
from asciimatics.widgets.label import Label
from asciimatics.widgets.popupdialog import PopUpDialog 
from asciimatics.widgets.frame import Frame 
from asciimatics.screen import Screen, ManagedScreen
from asciimatics.event import MouseEvent, KeyboardEvent, Event
from utils.graph_math import min_max

from asciimatics.widgets.utilities import _split_text
from typing import Optional

class LegendLabel(Label):
    GREEN = Screen.COLOUR_GREEN
    RED = Screen.COLOUR_RED
    def __init__(self, label_text: str, height: int=1, colour = None, align: str = "<", name: Optional[str] = None):
        s = "• " + label_text
        super().__init__(s, height, align, name)
        self._colour = colour
        self._show = True

    def update(self, frame_no: int):
        assert self._frame
        (colour, attr, background) = self._frame.palette[self._pick_palette_key("label",
                                                                                    selected=False,
                                                                                    allow_input_state=False)]

        if self._has_focus:
            attr = Screen.A_BOLD

        if self._colour != None:
            colour = self._colour
        
        label = self._text[2:]
        self._frame.canvas.paint("• ",
                                 self._x,
                                 self._y,
                                 self.GREEN if self._show else self.RED,
                                 attr,
                                 background)                                
        for i, text in enumerate(_split_text(label, self._w, self._h, self._frame.canvas.unicode_aware)):
            self._frame.canvas.paint(f"{text:{self._align}{self._w}}",
                                     self._x + 2,
                                     self._y + i,
                                     colour,
                                     attr,
                                     background)

class GraphLegend(Frame):
    def __init__(self, screen, x, y, labels: list, colours=None, max_height=None):
        if x >= screen.width:
            raise ValueError(f"X coordinate exceeds screen bounds ({x} >= {screen.width})")
        
        if colours != None:
            if len(labels) != len(colours):
                raise ValueError(f"Colour list length ({len(colours)}) must match the number of labels provided ({len(labels)}!)")
       
        m, max_len = min_max([len(s) for s in labels])
        length = min(max_len, screen.width-y-1)
        self._width = length+4
        self._height = len(labels)+2
        if max_height != None:
            self._height = min(self._height, max_height)
        self._x = x
        self._y = y
        super().__init__(screen, self._height, self._width, x=self._x, y=self._y, has_border=True, can_scroll=True)
        
        self.set_theme("monochrome")
        self._selections = [True]*len(labels)
        self._labels = {}
        layout = Layout([length])
        self.add_layout(layout)
        for i in range(len(labels)):
            l = labels[i]
            self._labels[l] = LegendLabel(l, colour=colours[i])
            # self._labels[l] = Label(l)
            layout.add_widget(self._labels[l])
        self.fix()

    def close(self):
        for y in range(self._y, self._y+self._height):
            self._screen.print_at("  "*self._width, self._x, y)

        