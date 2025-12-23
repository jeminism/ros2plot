
from asciimatics.effects import Effect
from asciimatics.widgets.layout import Layout
from asciimatics.widgets.frame import Frame 
from asciimatics.widgets.text import Text 
from asciimatics.widgets.label import Label 
from asciimatics.widgets.checkbox import CheckBox 
from asciimatics.widgets.utilities import _split_text

from asciimatics.event import KeyboardEvent, MouseEvent
from collections import deque

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

PLOT_VISIBILITY_KEY = "visible"
PLOT_COLOUR_KEY = "colour"
class Legend(GenericFrame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        self._layout = Layout([1])
        self.add_layout(self._layout)
        self.set_plots({"Missing List initialization!": {PLOT_VISIBILITY_KEY:True}})
    
    def set_plots(self, plot_visibility: dict):
        # if graph_colours != None:
        #     if len(graph_names)!=len(graph_colours):
        #         raise ValueError("Length of graph colours must be same length as the list of graph names!")
        if not isinstance(plot_visibility, dict):
            raise ValueError("Expected dictionary value when populating legend") 

        if not all((isinstance(key, str) and isinstance(plot_visibility[key], dict)) for key in plot_visibility):
            raise ValueError("Expected dictionary of type: dict[str, dict]")
            
        if not all((PLOT_VISIBILITY_KEY in plot_visibility[key]) for key in plot_visibility):
            raise ValueError(f"Expected key '{PLOT_VISIBILITY_KEY}' in graph visibility dictionaries but it is missing")

        self._layout.clear_widgets()
        for i in plot_visibility:
            if plot_visibility[i][PLOT_VISIBILITY_KEY]:
                self._layout.add_widget(ColouredLabel("• " + i, colour=(plot_visibility[i][PLOT_COLOUR_KEY] if PLOT_COLOUR_KEY in plot_visibility[i] else None)))
        self.fix()

class Selector(GenericFrame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        self._layout = Layout([1])
        self.add_layout(self._layout)

    def set_plots(self, plot_visibility: dict):
        if not isinstance(plot_visibility, dict):
            raise ValueError("Expected dictionary value when populating legend") 

        if not all((isinstance(key, str) and isinstance(plot_visibility[key], dict)) for key in plot_visibility):
            raise ValueError("Expected dictionary of type: dict[str, dict]")
            
        if not all((PLOT_VISIBILITY_KEY in plot_visibility[key]) for key in plot_visibility):
            raise ValueError(f"Expected key '{PLOT_VISIBILITY_KEY}' in graph visibility dictionaries but it is missing")
        self._layout.clear_widgets()

        def toggle_visibility_gen(key):
            def toggle_visibility():
                plot_visibility[key][PLOT_VISIBILITY_KEY] = not plot_visibility[key][PLOT_VISIBILITY_KEY]
            return toggle_visibility

        for i in plot_visibility:
            c = CheckBox(i, on_change=toggle_visibility_gen(i))
            c._value = plot_visibility[i][PLOT_VISIBILITY_KEY]
            self._layout.add_widget(c)
        self.fix()
        