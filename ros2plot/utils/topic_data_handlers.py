

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, PointStamped, Point32
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid


NUMERIC_TYPES = (int, float, bool)

DATA_HANDLERS={
	PointCloud2 : PointCloud2Handler
}

def get_message_handler(type):
	if type in DATA_HANDLERS:
		return DATA_HANDLERS[type]
	return MessageHandler

# default processor just filters for numeric fields
def default_processor(data:dict):
	res = {}
	for d in data:
		if isinstance(data[d], NUMERIC_TYPES):
			res[d] = data[d]
	return res


#default message handler
class MessageHandler:
	def processor(self, data:dict):
		res = {}
		for d in data:
			if isinstance(data[d], NUMERIC_TYPES):
				res[d] = data[d]
		return res

	def get_3d_plot_data(self):
		return None


# define specific handlers which will process a dict of raw topic data into the fields needed for storage. 
from sensor_msgs_py import point_cloud2

class PointCloud2Handler(MessageHandler):
	def processor(self, data:dict):
		# reconstruct the msg from the dict
		m = PointCloud2
		# m.header. = data["header"]
		m.height = data["height"]
		m.width = data["width"]
		m.fields = data["fields"]
		m.is_bigendian = data["fields"]
		m.point_step = data["point_step"]
		m.row_step = data["row_step"]
		m.data = data["data"]
		m.is_dense = data["is_dense"]

		points_data = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z", "intensity"), skip_nans=True, reshape_organized_cloud=False)

		return points_data

	def get_3d_plot_data(self):
		plt = PlotData3D()
		plt.x_key = "x"
		plt.y_key = "y"
		plt.z_key = "z"
		plt.colour_transformer_key = "intensity"
		return plt