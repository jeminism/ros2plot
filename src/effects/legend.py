
from asciimatics.effects import Effect
from asciimatics.widgets.layout import Layout
from asciimatics.widgets.checkbox import CheckBox
from asciimatics.widgets.text import Text
from asciimatics.widgets.label import Label
from asciimatics.widgets.popupdialog import PopUpDialog 
from asciimatics.widgets.frame import Frame 
from asciimatics.screen import Screen, ManagedScreen
from asciimatics.event import MouseEvent
from utils.graph_math import min_max

class PopUpLegend(Frame):
    def __init__(self, screen, x, y, labels: list):
        m, max_len = min_max([len(s) for s in labels])
        length = min(max_len, screen.width/2)
        self._width = length+2
        self._height = len(labels)+2
        self._x = x
        self._y = y
        super().__init__(screen, self._height, self._width, x=self._x, y=self._y, can_scroll=True)

        self._selections = [True]*len(labels)
        self._labels = {}
        layout = Layout([length])
        self.add_layout(layout)
        for l in labels:
            self._labels[l] = Label(l)
            layout.add_widget(self._labels[l])
        self.fix()

    def close(self):
        for y in range(self._y, self._y+self._height):
            self._screen.print_at("  "*self._width, self._x, y)
