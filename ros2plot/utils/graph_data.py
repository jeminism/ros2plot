
from rclpy.node import Node
from . import colour_palette as COLOURS
from .memory_bounded_deque import MemoryBoundedDeque

import queue
import threading
import attrs
import math

TIMESTAMP_KEY="/callback_timestamp"

@attrs.define
class GraphConfigs:
    x: int = attrs.field(default=0) #origin position, where axis intersect
    y: int = attrs.field(default=0)
    width: int = attrs.field(default=0) #width of the plot area, in pixels
    height: int = attrs.field(default=0) #height of the plot area, in pixels
    y_min_value: int = attrs.field(default=0) # y value corresponding to pixel (x, height-1)
    y_max_value: int = attrs.field(default=0) # y value corresponding to pixel (x, 0)
    x_min_value: int = attrs.field(default=0) # x value corresponding to pixel (0, y)
    x_max_value: int = attrs.field(default=0) # x value corresponding to pixel (width-1, y)
    pause: bool = attrs.field(default=False) # flag to dictate if the effect should pause drawing or no

@attrs.define
class PlotData:
    data: MemoryBoundedDeque = attrs.field(factory=MemoryBoundedDeque) #bound to 1% available memory
    x_key: str = attrs.field(default="")
    visible: bool = attrs.field(default=False)
    interpolate: bool = attrs.field(default=True)
    high_def: bool = attrs.field(default=True)
    plot_mean: bool = attrs.field(default=True)
    colour: int = attrs.field(default=COLOURS.DEFAULT)
    minimum: float = attrs.field(default=math.inf)
    maximum: float = attrs.field(default=-math.inf)

class RosPlotDataHandler:
    def __init__(self):
        self._queue = queue.Queue() # queue to store data updates, to avoid lock overheads involved when updating data from each callback
        self._data = {} # dictionary to store [field : PlotData] pairs
        # individual locks inside each MemoryBoundedDeque is insufficient, as we cannot guarantee that all deques are updated before each draw, 
        # resulting in mismatch length error during plotting
        self._lock = threading.Lock() # solution: an overall lock is needed to guard data update and draw cycle

    @property
    def data(self):
        return self._data
    
    def timestamp_key_from_field(self, field_name):
        return field_name.split("/")[0]+TIMESTAMP_KEY
    
    def get_plot(self, name:str):
        if name not in self._data:
            return None
        return self._data[name]

    def get_ros_data_handler(self, topic_name):
        # dynamically generate data field handlers. this is to be passed into the anysub object so that it will dynamically append to the fields in _data on callback
        def ros_handler(node: Node, update_data: dict):
            self._queue.put((topic_name, node.get_time(), update_data)) #store tuple of the topic name and the latest update data
        return ros_handler

    def _add_to_data(self, key, value=None):
        if key not in self._data:
            self._data[key] = PlotData()
            self._data[key].data.set_configs(max_fraction=0.02, trim_fraction=0.05)
        if value != None:
            self._data[key].data.append(value)
            if value < self._data[key].minimum:
                self._data[key].minimum = value 
            if value > self._data[key].maximum:
                self._data[key].maximum = value 
    

    def _process_topic_update(self, topic_name, timestamp, update_data):
        v = None
        for key, value in update_data.items():
            self._add_to_data(topic_name + key, value)
            v=value
        self._add_to_data(topic_name+TIMESTAMP_KEY, timestamp if v!=None else None)

    
    def _process_data_queue(self):
        with self._lock:
            while not self._queue.empty():
                topic_name, timestamp, update_data = self._queue.get()
                self._process_topic_update(topic_name, timestamp, update_data)
    
    def _clear_latest_data(self):
        with self._lock:
            for key in self._data:
                self._data[key].data.clear_latest()
    
        
    