
from asciimatics.effects import Effect
from asciimatics.widgets.layout import Layout
from asciimatics.widgets.frame import Frame 
from asciimatics.widgets.text import Text 
from asciimatics.widgets.label import Label 
from asciimatics.widgets.checkbox import CheckBox 
from asciimatics.widgets.dropdownlist import DropdownList 
from asciimatics.widgets.divider import Divider 
from asciimatics.widgets.utilities import _split_text

from asciimatics.event import KeyboardEvent, MouseEvent
from collections import deque

from utils.graph_data import PlotData, TIMESTAMP_KEY

class GenericFrame(Frame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        if width == None:
            width = screen._width
        if height == None:
            height = screen._height

        self._x = x
        self._y = y
        self._width = width
        self._height = height
        super().__init__(screen, height, width, x=x, y=y, has_border=True, hover_focus=True, can_scroll=True)


    def cleanup(self):
        for y in range(self._y, self._y+self._height):
            self._screen.print_at("  "*self._width, self._x, y)


class TextInput(GenericFrame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        
        layout = Layout([1, 1])
        self.add_layout(layout)
        layout.add_widget(Text(name="input"))
        self.fix()

        self._history = deque(maxlen=100)
        self._last = 0

    def clear(self):
        self._last = 0
        text = self.find_widget("input")
        text.value=""
    
    def value(self):
        text = self.find_widget("input")
        return text.value
    
    def set_value(self, value):
        text = self.find_widget("input")
        text.value = value

    def save_to_history(self):
        val = self.value()
        if val == None:
            return
        self._history.append(val)
    
    def go_down_history(self):
        n = len(self._history)
        if self._last + n <= 1:
            self._last = -n
        else:
            self._last -= 1
        self.set_value(self._history[self._last+n])

    def go_up_history(self):
        if self._last >= 0:
            self._last = 0
        else:
            self._last += 1

        if self._last<0:
            self.set_value(self._history[self._last])

    def process_event(self, event):
        if isinstance(event, KeyboardEvent):
            if event.key_code == -204: # UP ARROW
                self.go_down_history()
                return None
            if event.key_code == -206: # DOWN ARROW
                self.go_up_history()
                return None
            if event.key_code in (10, 13):
                self.save_to_history()
                return None
        
        return super().process_event(event)
        



class TextLabel(GenericFrame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        
        layout = Layout([1])
        self.add_layout(layout)
        layout.add_widget(Label("", name="label"))
        self.fix()

    def clear(self):
        label = self.find_widget("label")
        label.text=""
    
    def set_value(self, value):
        label = self.find_widget("label")
        label.text = value

class ColouredLabel(Label):
    def __init__(self, label_text: str, height: int=1, colour = None, align: str = "<", name: str = None):
        super().__init__(label_text, height, align, name)
        self._colour = colour

    def update(self, frame_no: int):
        assert self._frame
        (colour, attr, background) = self._frame.palette[self._pick_palette_key("label",
                                                                                selected=False,
                                                                                allow_input_state=False)]

        if self._colour != None:
            colour = self._colour
                               
        for i, text in enumerate(_split_text(self._text, self._w, self._h, self._frame.canvas.unicode_aware)):
            self._frame.canvas.paint(f"{text:{self._align}{self._w}}",
                                     self._x,
                                     self._y + i,
                                     colour,
                                     attr,
                                     background)

class Legend(GenericFrame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        self._layout = Layout([1])
        self.add_layout(self._layout)
        self.set_plots({})
    
    def set_plots(self, plot_data: dict[str, PlotData]):
        self._layout.clear_widgets()
        for field, plt in plot_data.items():
            if plt.visible:
                self._layout.add_widget(ColouredLabel("• " + field, colour=plt.colour))
        self.fix()

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
                continue
            c = CheckBox(field, on_change=toggle_visibility_gen(field))
            c._value = plot_data[field].visible
            self._layout.add_widget(c)
        self.fix()
