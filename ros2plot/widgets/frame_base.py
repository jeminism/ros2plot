
from asciimatics.widgets.frame import Frame 

#just a base for all widgets defined in this package. primary helper addition of a cleanup function so that the main program can clean the widget areas upon deletion
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
        self._cleanup_impl()

    def _cleanup_impl(self):
        return