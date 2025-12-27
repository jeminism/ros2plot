
from .frame_base import GenericFrame
from .text_label import ColouredLabel
from ..utils import PlotData

from asciimatics.widgets.layout import Layout

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
