import argparse
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from asciimatics.screen import Screen, ManagedScreen
from asciimatics.scene import Scene
from asciimatics.event import KeyboardEvent, MouseEvent
from asciimatics.exceptions import ResizeScreenError, StopApplication
from asciimatics.widgets.popupdialog import PopUpDialog
import sys

# from effects.graph import GraphXY, GraphData
from effects.effect_base import DrawOffsets
from effects.graph_components import XAxis, YAxis, Plot, GraphConfigs
from effects.legend import GraphLegend
from effects.frames import TextInput, TextLabel

from utils.colour_palette import COLOURS, NUM_COLOURS
from utils.graph_math import min_max, multi_min_max, get_mapped_value

import threading
import time

from anysub import MultiSubscriber, TopicData, CALLBACK_TIMESTAMP_KEY

class Ros2Plot():
    def __init__(self, screen: Screen, header_bar_height:int, padding: int, multi_subscriber: MultiSubscriber):
        self._screen = screen
        self._scene = Scene([], duration=-1)
        self._padding = padding
        # self._header_bar_height = header_bar_height
        self._effects = {}
        self._graph_config = GraphConfigs()
        self._draw_offsets = DrawOffsets()

        self._multi_subscriber = multi_subscriber
        self._start_time = self._multi_subscriber.get_time() #sync with ros time
        self._x_key = None

        self._plot_data = self._multi_subscriber.get_data()
        self._plot_count = 0

        self.update_draw_offsets(padding + 6, padding + header_bar_height+1)
        self.update_graph_config([], None)

        self.setup_info_bar(self._screen.width-2*padding, header_bar_height, padding, padding)
        self.setup_plot()

    def setup_info_bar(self, width, height, x, y):
        self._effects["header_label"] = TextLabel(self._screen, width, height, x, y)
        self._effects["header_input"] = TextInput(self._screen, width, height, x, y)
    
    def setup_plot(self):
        self._effects["y_axis"] = YAxis(self._screen, self._graph_config, self._draw_offsets)
        self._effects["x_axis"] = XAxis(self._screen, self._graph_config, self._draw_offsets)
    
    def update_draw_offsets(self, x, y):
        self._draw_offsets.x = x # 6 is standard padding to allow for y value axis labels
        self._draw_offsets.y = y

    def update_graph_config(self, y_data: list[list], x_values: list=None):
        self._graph_config.width = self._screen.width-self._draw_offsets.x-self._padding-6 # 6 is the size limit of value labels exetending past the max width of the graph
        self._graph_config.height = self._screen.height-self._draw_offsets.y-self._padding
        
        if len(y_data) > 0 and len(y_data[0]) > 0:
            self._graph_config.y_min_value, self._graph_config.y_max_value = multi_min_max(y_data)
            if x_values == None:
                self._graph_config.x_min_value = self._start_time
                self._graph_config.x_max_value = self._multi_subscriber.get_time()
            else:
                self._graph_config.x_min_value, self._graph_config.x_max_value = min_max(x_values)
        else:
            self._graph_config.y_min_value = self._graph_config.y_max_value = self._graph_config.y_min_value = self._graph_config.y_max_value = 0
        
        if self._graph_config.y_min_value == self._graph_config.y_max_value:
            self._graph_config.y_min_value -= 1
            self._graph_config.y_max_value += 1
        if self._graph_config.x_min_value == self._graph_config.x_max_value:
            self._graph_config.x_min_value -= 1
            self._graph_config.x_max_value += 1
            
        self._graph_config.x = get_mapped_value(0 if self._graph_config.x_min_value < 0 else self._graph_config.x_min_value, self._graph_config.x_max_value, self._graph_config.width-1, self._graph_config.x_min_value, 0)
        self._graph_config.y = get_mapped_value(0 if self._graph_config.y_min_value < 0 else self._graph_config.y_min_value, self._graph_config.y_max_value, 0, self._graph_config.y_min_value, self._graph_config.height-1)

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
            self.update_info_message(f"[NON-EXISTENT EFFECT] Tried to add an effect '{name}' to the scene but this effect doesnt exist")
            return
        
        if self._effects[name] in self._scene.effects:
            self.update_info_message(f"[EFFECT IN SCENE] Tried to add an effect '{name}' to the scene but this effect is already in the scene")
            return
        
        self._scene.add_effect(self._effects[name])

    def remove_effect(self, name):
        if name not in self._effects:
            self.update_info_message(f"[NON-EXISTENT EFFECT] Tried to remove an effect '{name}' from the scene but this effect doesnt exist")
            return
        
        if self._effects[name] not in self._scene.effects:
            self.update_info_message(f"[EFFECT NOT IN SCENE] Tried to remove an effect '{name}' from the scene but this effect is not in the scene")
            return
            
        self._scene.remove_effect(self._effects[name])

    def update_info_message(self, msg):
        self._effects["header_label"].set_value(msg)

    def handle_text_input(self, input_val: str):
        ls_split = input_val.split(" ")
        n = len(ls_split)
        if n == 2:
            # empty input do nothing
            return
        if n > 2:
            self.update_info_message(f"Expected at most two input values of <topic> <topic-type (optional)>. Instead received '{input_val}'")
            return

        self._multi_subscriber.add_subscriber(ls_split[0], ls_split[1] if n>1 else None)
        self.update_info_message(self._multi_subscriber.get_info_msg())
        self.initialize_plots()
    
    def initialize_plots(self, topic_filter: str = None):
        for i in range(len(self._plot_data.field_keys)):
            field = self._plot_data.field_keys[i]
            self.update_info_message(field)
            if topic_filter != None:
                if topic_filter not in field:
                    continue

            if field not in self._effects:
                topic_name = field.split("/")[0]
                data = self._plot_data.field_data[i]
                self.initialize_effect(field, Plot(self._screen, self._graph_config, data, self._plot_data.timestamps[topic_name], offsets=self._draw_offsets))
                c = COLOURS[self._plot_count%NUM_COLOURS]
                self._effects[field].set_configs(True, True, c)
                self._plot_count += 1

                #just add it to the scene for now
                self.add_effect(field)

    def _handle_event(self, event):
        # while event:
        # event = self._screen.get_event()
        if isinstance(event, KeyboardEvent):
            if self._effects["header_input"] in self._scene.effects:
                if event.key_code in (10, 13):
                    self._effects["header_input"].cleanup()
                    self.remove_effect("header_input")
                    self.handle_text_input(self._effects["header_input"].value())
                else:
                    while event:
                        self._effects["header_input"].process_event(event)
                        event = self._screen.get_event()
            else:
                if event.key_code == ord('p'):
                    self._graph_config.pause = not self._graph_config.pause
                elif event.key_code == ord('/'):
                    self._effects["header_input"].clear()
                    self.add_effect("header_input")
                else:
                    self.update_info_message(f"Unrecognized input: '{event.key_code}'")

        


    def run(self, shutdown):
        self._screen.set_scenes([self._scene])
        self.add_effect("header_label")
        self.add_effect("x_axis")
        self.add_effect("y_axis")
        # self.update_info_message("Ros2Plot Initialized!")
        count = 0
        while not shutdown:
            # self.update_info_message(f"{count}")
            count +=1
            try:
                self.update_graph_config(self._plot_data.field_data)
                self._screen.draw_next_frame()
                # self._handle_event()
                event = self._screen.get_event()
                if not (isinstance(event, KeyboardEvent) or isinstance(event, MouseEvent)):
                    continue
                self._handle_event(event)

            except StopApplication:
                break
            except ResizeScreenError:
                pass
            time.sleep(0.033)

def ros_run(node, shutdown):
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    try:
        # executor.spin()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        shutdown = True
        return

def set_args(parser):
    parser.add_argument('topic_name', nargs="?", help='Name of the topic to subscribe')
    parser.add_argument('topic_type', nargs="?", default=None, help='Type of topic to subscribe to. If missing, will internally attempt to automatically determine the topic type.')
    parser.add_argument('--fields', nargs='*', help='Specific fields to plot. Expects directory style path.')
    parser.add_argument('--x-field', nargs=1, help='Specific field to use as x axis. Expects directory style path. If missing, will default to system time')
    parser.add_argument('--ignore-fields', nargs='*', help='Y value fields to ignore, even if it falls under the specified fields. Expects directory style path.')

def main():
    parser = argparse.ArgumentParser()
    set_args(parser)

    args = vars(parser.parse_args(sys.argv[1:]))
    print(args)

    rclpy.init()

    # try:
    #     topic_name, topic_type = validate_topic(args["topic_name"], args["topic_type"])
    # except ValueError as e:
    #     print(e)
    #     return
    # fields = ["/"+x for x in args["fields"]] if args["fields"]!=None else None
    # x_key = "/"+args["x_field"][0] if args["x_field"]!=None else None
    # if fields != None and x_key != None and x_key not in fields:
    #     fields.append(x_key)
    # print(fields)
    
    shutdown = False
    anysub = MultiSubscriber()
    with ManagedScreen() as screen:
        display = Ros2Plot(screen, 3, 2, anysub)

        t = threading.Thread(target=ros_run, args=(anysub,shutdown), daemon=True)
        t.start()
        display.run(shutdown)
        t.join()

if __name__ == '__main__':
    main()
