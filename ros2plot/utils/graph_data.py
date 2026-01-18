
from rclpy.node import Node
from . import colour_palette as COLOURS
from .memory_bounded_deque import MemoryBoundedDeque
from .csv_io import read_from_csv

import queue
import threading
import attrs
import math

TIMESTAMP_KEY="/callback_timestamp"
CSV_TIMESTAMP_KEY="timestamp"

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
    colour: int = attrs.field(default=COLOURS.DEFAULT)
    minimum: int = attrs.field(default=math.inf)
    maximum: int = attrs.field(default=-math.inf)

class RosPlotDataHandler:
    def __init__(self, csv_x=CSV_TIMESTAMP_KEY):
        self._queue = queue.Queue() # queue to store data updates, to avoid lock overheads involved when updating data from each callback
        self._data = {} # dictionary to store [field : PlotData] pairs
        # individual locks inside each MemoryBoundedDeque is insufficient, as we cannot guarantee that all deques are updated before each draw, 
        # resulting in mismatch length error during plotting
        self._lock = threading.Lock() # solution: an overall lock is needed to guard data update and draw cycle
        self._default_csv_x = csv_x

    @property
    def data(self):
        return self._data
    
    @property
    def default_csv_x(self):
        return self._default_csv_x

    @default_csv_x.setter
    def default_csv_x(self, value):
        self._default_csv_x = value
    
    def get_x_key_from_field(self, field_name, x_key=None): #check for similar named fields across all data sources. otherwise retrieve based on defaults
        data_source = field_name.split("/")[0]
        if x_key == None: #retrieve defaults
            return self._default_x_key_for_data(data_source)
        else:
            if len(x_key) > 0:
                x_key_source = x_key.split("/")[0]
                if data_source == x_key_source:
                    return x_key # return x_key directly if the data source is contained in itself as this is a specific x_key filter
                else:
                    has_slash = x_key[0] == "/"
                    return data_source + ("" if has_slash else "/") + x_key # otherwise return concantenation of the source and key
    
    def _default_x_key_for_data(self, data_source):
        if ".csv" in data_source:
            return data_source+"/"+self._default_csv_x        
        return data_source+TIMESTAMP_KEY

    
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
    
    def csv_to_plotdata(self, filename):
        try:
            rows = read_from_csv(filename)
            for row in rows:
                # print(row)
                for csv_field,value_str in row.items():
                    field = filename+"/"+csv_field
                    value = float(value_str) if "." in value_str else int(value_str)
                    if field not in self.data:
                        self.data[field] = PlotData()
                        self.data[field].data.set_configs(max_fraction=0.02, trim_fraction=0.05)
                    self.data[field].data.append(value)
                    if value < self.data[field].minimum:
                        self.data[field].minimum = value 
                    if value > self.data[field].maximum:
                        self.data[field].maximum = value 
        except Exception as e:
            raise Exception(f"{e}")
