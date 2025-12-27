
from .frame_base import GenericFrame

from asciimatics.widgets.layout import Layout
from asciimatics.widgets.text import Text

from asciimatics.event import KeyboardEvent, MouseEvent

from collections import deque


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
        