
from asciimatics.effects import Effect
from asciimatics.widgets.layout import Layout
from asciimatics.widgets.frame import Frame 
from asciimatics.widgets.text import Text 
from asciimatics.widgets.label import Label 

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
        
        layout = Layout([1])
        self.add_layout(layout)
        layout.add_widget(Text(name="input"))
        self.fix()

    def clear(self):
        text = self.find_widget("input")
        text.value=""
    
    def value(self):
        text = self.find_widget("input")
        return text.value


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
