
from asciimatics.screen import Screen
from asciimatics.scene import Scene
from asciimatics.event import KeyboardEvent, MouseEvent
from asciimatics.exceptions import ResizeScreenError, StopApplication

# from effects.graph import GraphXY, GraphData
from effects.effect_base import DrawOffsets
from effects.graph_axis import XAxis, YAxis
from effects.graph_plot import Plot
from effects.graph_manipulators import GraphInspector, GraphZoomSelector

from widgets.plots_legend import Legend
from widgets.plots_selector import Selector
from widgets.text_input import TextInput
from widgets.text_label import TextLabel

from utils.colour_palette import COLOURS, NUM_COLOURS
from utils.graph_math import min_max, get_mapped_value
from utils.graph_data import GraphConfigs, PlotData, RosPlotDataHandler

import time
import math

from ros.multisub import MultiSubscriber

class Ros2Plot(RosPlotDataHandler):
    def __init__(self, screen: Screen, header_bar_height:int, padding: int, multi_subscriber: MultiSubscriber):
        super().__init__()
        self._scene = Scene([], duration=-1)
        self._screen = screen
        self._screen.set_scenes([self._scene])
        self._padding = padding
        self._header_bar_height = header_bar_height
        self._effects = {}
        self._graph_config = GraphConfigs()
        self._draw_offsets = DrawOffsets()

        self._multi_subscriber = multi_subscriber
        self._start_time = self._multi_subscriber.get_time() #sync with ros time
        self._x_key = None

        self._plot_count = 0

        self.update_draw_offsets(padding + 6, padding + header_bar_height+1)
        self.update_graph_config()

        self.setup_info_bar(self._screen.width-2*padding, header_bar_height, padding, padding)
        # self.setup_tooltip(self._screen.width-2*padding, 3, padding, self._screen.height-3-padding)
        self.setup_plot()
        
        self.add_base_effects()
        # self.update_tooltip(self.tooltip())
        self.update_info_message("Ros2Plot Initialized!")
    
    def set_screen(self, screen):
        readd = []
        for name in self._effects:
            if self._effects[name] in self._scene.effects:
                readd.append(name)

        self._screen = screen
        self._scene = Scene([], duration=-1)
        self._screen.set_scenes([self._scene])
        self.setup_info_bar(self._screen.width-2*self._padding, self._header_bar_height, self._padding, self._padding)
        self.setup_plot()
        self._plot_count = 0
        self.initialize_plots()

        for e in readd:
            self.add_effect(e)
        self.update_info_message("Ros2Plot RE-Initialized!")
        self._screen.refresh()

    def add_base_effects(self):
        self.add_effect("header_label")
        self.add_effect("x_axis")
        self.add_effect("y_axis")
        # self.add_effect("tooltip")


    def setup_info_bar(self, width, height, x, y):
        self._effects["header_label"] = TextLabel(self._screen, width, height, x, y)
        self._effects["header_input"] = TextInput(self._screen, width, height, x, y)
    
    # def setup_tooltip(self, width, height, x, y):
    #     self._effects["tooltip"] = TextLabel(self._screen, width, height, x, y)

    def setup_plot(self):
        self._effects["y_axis"] = YAxis(self._screen, self._graph_config, self._draw_offsets)
        self._effects["x_axis"] = XAxis(self._screen, self._graph_config, self._draw_offsets)
        w = min(self._screen.width // 2, 20)
        h = min(self._screen.height // 2, 10)
        self._effects["legend"] = Legend(self._screen, w, h, self._screen.width-w-1, self._draw_offsets.y)
        self._effects["selector"] = Selector(self._screen, self._x_key, self._screen.width//2, self._screen.height//2, self._screen.width//4, self._draw_offsets.y)
        self._effects["inspector"] = GraphInspector(self._screen, self._graph_config, self.data, offsets=self._draw_offsets)
        self._effects["zoom_selector"] = GraphZoomSelector(self._screen, self._graph_config, self._draw_offsets)

    
    def initialize_plots(self, topic_filter: str = None, auto_add_display:bool=True):
        for field in self.data:
            if topic_filter != None:
                if topic_filter not in field:
                    continue

            if field == self.timestamp_key_from_field(field):
                continue

            if field not in self._effects:
                self.initialize_effect(field, Plot(self._screen, self._graph_config, self.data, y_key=field, offsets=self._draw_offsets))
                self.data[field].colour = COLOURS[self._plot_count%NUM_COLOURS]
                self.set_plot_x_axis_key(field, self._x_key)
                
                self._plot_count += 1

                #just add it to the scene for now
                if auto_add_display:
                    self.add_plot(field)
    
    def set_x_axis_key(self, x_key=None):
        self._x_key = x_key
        for field in self.data:
            self.set_plot_x_axis_key(field, x_key)

    def set_plot_x_axis_key(self, field, x_key:str=None):
        self.data[field].x_key = x_key if x_key != None else self.timestamp_key_from_field(field)
    
    def get_ros_time(self):
        return self._multi_subscriber.get_time()
        
    def min_max_visible_y(self):
        res_min = math.inf
        res_max = -math.inf
        for plot_data in self.data.values():
            if not plot_data.visible:
                continue
            valid = True
            t_min, t_max = min_max(plot_data.data)
            if t_min < res_min:
                res_min = t_min
            if t_max > res_max:
                res_max = t_max
        if res_min != math.inf:
            return res_min, res_max  
        return 0,0

    def update_draw_offsets(self, x, y):
        self._draw_offsets.x = x # 6 is standard padding to allow for y value axis labels
        self._draw_offsets.y = y

    def update_graph_config(self):
        self._graph_config.width = self._screen.width-self._draw_offsets.x-self._padding-6 # 6 is the size limit of value labels exetending past the max width of the graph
        self._graph_config.height = self._screen.height-self._draw_offsets.y-self._padding #-4 #3+1 for tooltip footer
        
        if len(self.data) > 0 and len(next(iter(self.data.values())).data) > 0:
            self._graph_config.y_min_value, self._graph_config.y_max_value = self.min_max_visible_y()
            if self._x_key == None:
                first_field_key = next(iter(self.data.keys()))
                first_time_data = self.data[self.timestamp_key_from_field(first_field_key)].data
                self._graph_config.x_min_value = first_time_data[0] if len(first_time_data) > 0 else self._start_time
                self._graph_config.x_max_value = self.get_ros_time()
            else:
                self._graph_config.x_min_value, self._graph_config.x_max_value = min_max(self.data[self._x_key].data)
        else:
            self._graph_config.y_min_value = self._graph_config.y_max_value = self._graph_config.y_min_value = self._graph_config.y_max_value = 0
        
        if self._graph_config.y_min_value == self._graph_config.y_max_value:
            self._graph_config.y_min_value -= 1
            self._graph_config.y_max_value += 1
        if self._graph_config.x_min_value == self._graph_config.x_max_value:
            self._graph_config.x_min_value -= 1
            self._graph_config.x_max_value += 1

        x_0 = 0 
        if self._graph_config.x_min_value < 0 and self._graph_config.x_max_value < 0:
            x_0 = self._graph_config.x_max_value
        elif self._graph_config.x_min_value > 0 and self._graph_config.x_max_value > 0:
            x_0 = self._graph_config.x_min_value

        y_0 = 0 
        if self._graph_config.y_min_value < 0 and self._graph_config.y_max_value < 0:
            y_0 = self._graph_config.y_max_value
        elif self._graph_config.y_min_value > 0 and self._graph_config.y_max_value > 0:
            y_0 = self._graph_config.y_min_value
        
        self._graph_config.x = get_mapped_value(x_0, self._graph_config.x_max_value, self._graph_config.width-1, self._graph_config.x_min_value, 0)
        self._graph_config.y = get_mapped_value(y_0, self._graph_config.y_max_value, 0, self._graph_config.y_min_value, self._graph_config.height-1)
        
        # try:    
        # except Exception as e:
        #     print(f"{e}. {0 if self._graph_config.y_min_value < 0 else self._graph_config.y_min_value}, y_min: {self._graph_config.y_min_value}, y_max: {self._graph_config.y_max_value}")

    def initialize_effect(self, name, effect=None):
        if name in self._effects:
            if self._effects[name] != None:
                self.update_info_message(f"[NON-UNIQUE EFFECT NAME] Tried to initialize an effect '{name}' but this effect already exists")
                return

        self._effects[name] = effect
    
    def delete_effect(self, name):
        if name in self._effects:
            if self._effects[name] in self._scene.effects:
                self.update_info_message(f"[INVALID DELETE] Unable to delete the effect '{name}' because it is present in the scene")

            if self._effects[name] != None:
                self._effects[name] = None

    def add_effect(self, name):
        if name not in self._effects or self._effects[name] == None:
            # self.update_info_message(f"[NON-EXISTENT EFFECT] Tried to add an effect '{name}' to the scene but this effect doesnt exist")
            return
        
        if self._effects[name] in self._scene.effects:
            #self.update_info_message(f"[EFFECT IN SCENE] Tried to add an effect '{name}' to the scene but this effect is already in the scene")
            return
        
        self._scene.add_effect(self._effects[name])

    def remove_effect(self, name):
        if name not in self._effects:
            # self.update_info_message(f"[NON-EXISTENT EFFECT] Tried to remove an effect '{name}' from the scene but this effect doesnt exist")
            return
        
        if self._effects[name] not in self._scene.effects:
            #self.update_info_message(f"[EFFECT NOT IN SCENE] Tried to remove an effect '{name}' from the scene but this effect is not in the scene")
            return
           
        self._scene.remove_effect(self._effects[name])
    
    #specializations for plots
    def add_plot(self, field_name):
        self.add_effect(field_name)
        self.data[field_name].visible = True

    def remove_plot(self, field_name):
        self.remove_effect(field_name)
        self.data[field_name].visible = False

    def update_info_message(self, msg):
        self._effects["header_label"].set_value(msg)
    
    # def update_tooltip(self, msg):
    #     self._effects["tooltip"].set_value(msg)

    def handle_text_input(self, input_val: str):
        ls_split = input_val.split(" ")
        n = len(ls_split)
        if n == 2:
            # empty input do nothing
            return
        if n > 2:
            self.update_info_message(f"Expected at most two input values of <topic> <topic-type (optional)>. Instead received '{input_val}'")
            return
        self.add_subscriber(ls_split[0], ls_split[1] if n>1 else None)

    def add_subscriber(self, topic:str, topic_type:str=None, field_filter:list=None):
        topic = topic.lstrip("/")
        self._multi_subscriber.add_subscriber(self.get_ros_data_handler(topic), topic, topic_type)
        self.update_info_message(self._multi_subscriber.get_info_msg())
        self.initialize_plots(topic_filter=topic, auto_add_display=True if field_filter == None else False)
        if field_filter != None:
            fails = []
            for field in field_filter:
                field_name = topic+"/"+field.lstrip("/")
                if field_name in self._effects:
                    self.add_plot(field_name)
                else:
                    fails.append(field)
            if len(fails) > 0:
                self.update_info_message(f"Tried to add plot for fields '{fails}' but these are invalid fields in topic '{topic}'")

    def show_legend(self):
        self._effects["legend"].set_plots(self.data)
        self.add_effect("legend")

    def show_selector(self):
        self._effects["selector"].set_plots(self.data, self._x_key)
        self.add_effect("selector")
        
    def show_inspector(self):
        self._effects["inspector"].set_x_value()
        # self.update_tooltip(self._effects["inspector"].tooltip())
        self.add_effect("inspector")

    def show_zoom(self):
        self.update_info_message(f"[ZOOM INSPECTOR] {self._effects["zoom_selector"].get_points_string()}")
        # self.update_tooltip(self._effects["zoom_selector"].tooltip())
        self._effects["zoom_selector"].reset()
        self.add_effect("zoom_selector")

    def show_plots(self):
        for field in self.data:
            if self.data[field].visible:
                self.add_effect(field) 
            else:
                if field in self._effects:
                    self._effects[field].e_clear()
                self.remove_effect(field)

    def _handle_event(self, event):
        self._scene.process_event(event)
        self._handle_display_controls(event)

    def _handle_display_controls(self, event):
        if isinstance(event, KeyboardEvent):
            if self._effects["header_input"] in self._scene.effects:
                if event.key_code in (10, 13):
                    self._effects["header_input"].cleanup()
                    self.remove_effect("header_input")
                    self.handle_text_input(self._effects["header_input"].value())
            elif self._effects["selector"] in self._scene.effects:
                if event.key_code == ord('s'):
                    self._effects["selector"].cleanup()
                    self.remove_effect("selector")
                    self.show_plots()
                    self.set_x_axis_key(self._effects["selector"].get_x_field_selection())
            else:
                if event.key_code == ord('p'):
                    print(self.data)
                    if self._effects["inspector"] not in self._scene.effects:
                        self._graph_config.pause = not self._graph_config.pause
                elif event.key_code == ord('l'):
                    if self._effects["legend"] in self._scene.effects:
                        self._effects["legend"].cleanup()
                        self.remove_effect("legend")
                    else:
                        self.show_legend()
                elif event.key_code == ord('s'):
                    self.show_selector()
                elif event.key_code == ord('/'):
                    self._effects["header_input"].clear()
                    self.add_effect("header_input")
                elif event.key_code == ord('i'):
                    if self._effects["inspector"] in self._scene.effects:
                        self._effects["inspector"].e_clear()
                        self.remove_effect("inspector")
                        # self.update_tooltip(self.tooltip())
                    else:
                        self._graph_config.pause = True
                        self.show_inspector()
                elif event.key_code == ord('z'):
                    if self._effects["zoom_selector"] in self._scene.effects:
                        self._effects["zoom_selector"].e_clear()
                        self.remove_effect("zoom_selector")
                        # self.update_tooltip(self.tooltip())
                    else:
                        self._graph_config.pause = True
                        self.show_zoom()
                else:
                    self.update_info_message(f"Unhandled Key press '{event.key_code}'")
    
    def tooltip(self):
        return "p : Pause plot rendering | l : show legend | s : toggle plot visibility | i : open value inspector | z : open window resizer | / : open subscription configurator"        


    def run(self, shutdown):
        while not shutdown:
            self.update_info_message(f"current x key : {self._x_key}")
            try:
                if not self._graph_config.pause:
                    self.update_graph_config()
                
                if self._effects["zoom_selector"] in self._scene.effects:
                    self.update_info_message(f"[ZOOM INSPECTOR] {self._effects["zoom_selector"].get_points_string()}")

                if self._effects["inspector"] in self._scene.effects:
                    self.update_info_message(f"[INSPECTION] X = {self._effects["inspector"].get_x_value():f}")

                self._screen.draw_next_frame()
                # self._handle_event()
                event = self._screen.get_event()
                if not (isinstance(event, KeyboardEvent) or isinstance(event, MouseEvent)):
                    continue
                self._handle_event(event)

            except StopApplication:
                shutdown = True
                break
            except ResizeScreenError:
                pass
            if self._screen.has_resized():
                break
            time.sleep(0.033)
