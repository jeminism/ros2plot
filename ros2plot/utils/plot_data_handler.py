
from rclpy.node import Node
from .csv_io import read_from_csv
from .message_handlers import get_message_processor, is_3d_plottable, get_message_2d_plottable_fields
from .message_handler_definitions import FIELD_3D
from .graph_data import GraphConfigs, PlotData, PlotData3D

import numpy as np
import queue
import threading

TIMESTAMP_KEY="/callback_timestamp"
CSV_TIMESTAMP_KEY="timestamp"

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
        return None
    
    def _default_x_key_for_data(self, data_source):
        if ".csv" in data_source:
            return data_source+"/"+self._default_csv_x        
        return data_source+TIMESTAMP_KEY

    
    def get_plot(self, name:str):
        if name not in self._data:
            return None
        return self._data[name]

    def get_ros_data_handler(self, topic_name, topic_type):
        # dynamically generate data field handlers. this is to be passed into the anysub object so that it will dynamically append to the fields in _data on callback

        message_handler = get_message_processor(topic_type) #processor fn

        def ros_handler(timestamp, msg):
            self._queue.put((topic_name, timestamp, message_handler(msg))) #store tuple of the topic name and the latest update data
        return ros_handler

    def _add_to_data(self, key, value=None):
        if key not in self._data:
            raise ValueError(f"Tried to add data point for field '{key}', but this field is not present in the existing data.")
        if isinstance(self._data[key], PlotData3D):
            if not isinstance(value, np.ndarray):
                raise TypeError(f"Error while updating 3D data for topic '{key}'. Expected numpy array")
            self._data[key].data = value # this is an np.ndarray
        else:
            if value == None:
                return
            self._data[key].data.append(value)
            if value < self._data[key].minimum:
                self._data[key].minimum = value 
            if value > self._data[key].maximum:
                self._data[key].maximum = value 

    def init_plot_data(self, topic_name, topic_type):
        # handle 2d fields
        fields = get_message_2d_plottable_fields(topic_type)
        for f in fields:
            key = topic_name + "/" + f
            self._init_plot_data_2d(key)
        self._init_plot_data_2d(topic_name+TIMESTAMP_KEY)
        # handle 3d
        # plt_3d = get_message_3d_plotdata(topic_type)
        # if plt_3d != None:
        if is_3d_plottable(topic_type):
            self._data[topic_name] = PlotData3D()
            
    def _init_plot_data_2d(self, key):
        self._data[key] = PlotData()
        self._data[key].data.set_configs(max_fraction=0.02, trim_fraction=0.05)


    def _process_topic_update(self, topic_name, timestamp, update_data):
        # v = None
        for key, value in update_data.items():
            if key == FIELD_3D:
                self._add_to_data(topic_name, value)
            else:
                self._add_to_data(topic_name + key, value)
            # v=value
        # self._add_to_data(topic_name+TIMESTAMP_KEY, timestamp if v!=None else None)
        self._add_to_data(topic_name+TIMESTAMP_KEY, timestamp)

    
    def _process_data_queue(self):
        with self._lock:
            while not self._queue.empty():
                topic_name, timestamp, update_data = self._queue.get()
                self._process_topic_update(topic_name, timestamp, update_data)
    
    def _clear_latest_data(self):
        with self._lock:
            for key in self._data:
                if isinstance(self._data[key], PlotData):
                    self._data[key].data.clear_latest()
    
    def csv_to_plotdata(self, filename):
        try:
            rows = read_from_csv(filename)
            for row in rows:
                # print(row)
                for csv_field,value_str in row.items():
                    field = filename+"/"+csv_field
                    try:
                        value = float(value_str) if "." in value_str else int(value_str)
                    except:
                        continue
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
