from ..graph_data import PlotData3D
from .handler_db import register_module

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

def processor(msg) -> dict:
    points_data = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True, reshape_organized_cloud=False)
    return points_data

def plotdata_3d(self):
    plt = PlotData3D()
    plt.x_key = "x"
    plt.y_key = "y"
    plt.z_key = "z"
    plt.colour_transformer_key = "intensity"
    return plt

def plottable_fields_2d(msg):
    return ("x", "y", "z", "intensity")

#register self
register_module(PointCloud2, __name__)