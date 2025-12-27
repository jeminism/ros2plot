
from .frame_base import GenericFrame

from asciimatics.widgets.layout import Layout
from asciimatics.widgets.label import Label
from asciimatics.widgets.utilities import _split_text


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

class TextLabel(GenericFrame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        
        layout = Layout([1])
        self.add_layout(layout)
        layout.add_widget(ColouredLabel("", name="label"))
        self.fix()

    def clear(self):
        label = self.find_widget("label")
        label.text=""
    
    def set_value(self, value):
        label = self.find_widget("label")
        label.text = value
