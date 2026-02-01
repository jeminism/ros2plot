from ..graph_data import PlotData3D
from .handler_db import register_module, FIELD_3D

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

keys = ("x", "y", "z") # "intensity"

def processor(msg) -> dict:
    points_data = point_cloud2.read_points_numpy(msg, field_names=keys, skip_nans=True, reshape_organized_cloud=False)
    return {FIELD_3D:points_data}

# def plotdata_3d():
#     plt = PlotData3D()
#     plt.x_key = "x"
#     plt.y_key = "y"
#     plt.z_key = "z"
#     plt.colour_transformer_key = "intensity"
#     return plt

def is_3d_plottable():
    return True

def plottable_fields_2d(msg):
    return ("x", "y", "z", "intensity")

#register self
register_module(PointCloud2, __name__)