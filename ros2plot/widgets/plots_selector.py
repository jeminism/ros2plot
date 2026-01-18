
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
    
    def _cleanup_impl(self):
        # manual clearing of any artefacts left behind because we do not have control over any clearing functionality of the dropdownlist child widgets
        # TODO: if the list size exceeds the frame height, artefacts still remain even after list selection for some reason even if this function 
        #       is registered as on_change for the dropdownlist. investigate.
        if self._drop_down._child and self._drop_down._child in self._scene.effects:
            self._drop_down._child.close()
        self._screen.clear()
        
    def get_x_field_selection(self):
        v = self._drop_down.value
        for value, i in self._options:
            if i == v:
                return value if value != "Default" else None
        return None

    def set_plots(self, plot_data: dict[str, PlotData], current_x_key=None):
        self._layout.clear_widgets()

        self._options = [("Default", 1)]
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
                x_key = plot_data[key].x_key
                if x_key in plot_data and len(plot_data[key].data) == len(plot_data[x_key].data):
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

class PlotConfigurator(GenericFrame):
    def __init__(self, screen, width=None, height=None, x=0, y=0):
        super().__init__(screen, width, height, x, y)
        self.set_theme("monochrome")
        self._layout = Layout([1])
        self.add_layout(self._layout)

    # re-setup every time because we want to show the latest values in case they're changed elsewhere. 
    def setup_data(self, plot_data: dict[str, PlotData]):
        self._layout.clear_widgets()
        ref_cfg = None
        if len(plot_data) > 0:
            ref_cfg = next(iter(plot_data.values())) # all configs restricted to have the same values, so just reference the first one only

        # this implementation isnt very nice, but lazy to do better
        def toggle_interpolate():
            for field in plot_data:
                plot_data[field].interpolate = not plot_data[field].interpolate
                
        def toggle_high_def():
            for field in plot_data:
                plot_data[field].high_def = not plot_data[field].high_def
        
        def toggle_mean_plots():
            for field in plot_data:
                plot_data[field].plot_mean = not plot_data[field].plot_mean

        interpolate = self._layout.add_widget(CheckBox("Interpolate Across Columns", on_change=toggle_interpolate))
        interpolate._value = ref_cfg.interpolate if ref_cfg != None else False

        hd = self._layout.add_widget(CheckBox("High Definition Plots", on_change=toggle_high_def))
        hd._value = ref_cfg.high_def if ref_cfg != None else False

        mean = self._layout.add_widget(CheckBox("Plot Mean Values", on_change=toggle_mean_plots))
        mean._value = ref_cfg.plot_mean if ref_cfg != None else False

        self.fix()

        # interpolate = self._layout.add_widget(CheckBox("Interpolate Across Columns"))
        # interpolate._value = ref_cfg.interpolate if ref_cfg != None else False

        # hd = self._layout.add_widget(CheckBox("High Definition Plots"))
        # hd._value = ref_cfg.high_def if ref_cfg != None else False

        # mean = self._layout.add_widget(CheckBox("Plot Mean Values"))
        # mean._value = ref_cfg.plot_mean if ref_cfg != None else False
        
        # self.fix()
        
        # interpolate._on_change = toggle_interpolate
        # hd._on_change = toggle_high_def
        # mean._on_change = toggle_mean_plots
