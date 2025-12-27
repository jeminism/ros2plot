
from .frame_base import GenericFrame
from ..utils import PlotData, TIMESTAMP_KEY

from asciimatics.widgets.layout import Layout
from asciimatics.widgets.checkbox import CheckBox 
from asciimatics.widgets.dropdownlist import DropdownList 
from asciimatics.widgets.divider import Divider 

from asciimatics.event import KeyboardEvent, MouseEvent

class Selector(GenericFrame):
    def __init__(self, screen, x_key, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        self._layout = Layout([1])
        self.add_layout(self._layout)
        # self._drop_down = None
        self._drop_down = DropdownList([], label="X-Axis Data")
        self._options = []
        self._x_key = x_key
    
    def get_x_field_selection(self):
        v = self._drop_down.value
        for value, i in self._options:
            if i == v:
                return value if value != "Time" else None
        return None

    def set_plots(self, plot_data: dict[str, PlotData], current_x_key=None):
        self._layout.clear_widgets()

        self._options = [("Time", 1)]
        i = 1
        n = 2
        for field in plot_data:
            self._options.append((field, n))
            if field == current_x_key and current_x_key != None:
                i = n
            n+=1
        self._drop_down.options = self._options
        # self._drop_down._line = i
        self._drop_down.value = i
        self._layout.add_widget(self._drop_down)
        self._layout.add_widget(Divider())

        def toggle_visibility_gen(key):
            def toggle_visibility():
                plot_data[key].visible = not plot_data[key].visible
            return toggle_visibility

        for field in plot_data:
            if TIMESTAMP_KEY in field:
                # hardcoded ignore of TIMESTAMP_KEY fields
                continue
            c = CheckBox(field, on_change=toggle_visibility_gen(field))
            c._value = plot_data[field].visible
            self._layout.add_widget(c)
        self.fix()
