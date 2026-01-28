import numpy as np
import math


#numpy helpers. break out later

def get_rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr]
    ])

    Ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp]
    ])

    Rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


def get_transformation_matrix(x, y, z, roll, pitch, yaw):
    R = get_rotation_matrix(roll, pitch, yaw)
    t = np.array([x, y, z])

    view = np.eye(4)
    view[:3, :3] = R.T
    view[:3, 3] = -R.T @ t
    return view

####
class Frustum:
    def __init__(self, fov_y, aspect, near=0, far=math.inf):
        self.tan_half_fov = np.tan(fov_y / 2)
        self.aspect = aspect
        self.near = near
        self.far = far

    def contains(self, x, y, z):
        
        if z < self.near or z > self.far:
            return False

        y_limit = z * self.tan_half_fov
        x_limit = y_limit * self.aspect

        return (-x_limit <= x <= x_limit and
                -y_limit <= y <= y_limit)

    def get_pixel(self, x, y, z, width, height):
        
        if z < self.near or z > self.far:
            return -1,-1

        y_limit = z * self.tan_half_fov
        x_limit = y_limit * self.aspect

        if x > abs(x_limit) or y > abs(y_limit):
            return -1,-1

        return get_mapped_value(x, -x_limit, 0, x_limit, width-1), get_mapped_value(y, -y_limit, height-1, y_limit, 0)

class Plot3D(GraphEffect):
    def __init__(self, screen: Screen, cfg: GraphConfigs, db: dict[str, PlotData], offsets: DrawOffsets=DrawOffsets(), debug_fn=None):
        super().__init__(screen, cfg, offsets)
        self._db = db #dictionary db of field vs field data
        self._plt = None
        self._y_key = None
        self._z_key = None
        self.set_data_key(y_key, z_key)
        self._debug_fn = debug_fn
        self._frustum = Frustum(1.92, cfg.width/cfg.height)

    @property 
    def plot_width(self):
        return self._cfg.width*2 if self._plt.high_def else self._cfg.width #braille conversion
    
    @property
    def plot_height(self):
        return self._cfg.height*4 if self._plt.high_def else self._cfg.height #braille conversion

    def lookup_data(self, key):
        return self._db[key].data

    def get_screen_point(self, x, y, z):
        cfg = self._cfg
        T = get_transformation_matrix(cfg.x, cfg.y, cfg.z, cfg.roll, cfg.pitch, cfg.yaw)
        screen_point = T @ np.array([x, y, z, 1.0])
        return screen_point[0], screen_point[1], screen_point[2]

    def _draw(self, frame_no):
        
        y_data = self._plt.data
        x_data = self.lookup_data(self._plt.x_key)
        z_data = self.lookup_data(self._plt.z_key)
        
        closest_points = {}
        cfg = self._cfg

        for x,y,z in zip(x_data,y_data,z_data):
            x_screen, y_screen, z_screen = self.get_screen_point(x,y,z)
            x_pix, y_pix = self._frustum.get_pixel(x_screen, y_screen, z_screen, self.plot_width, self.plot_height)
            if x_pix < 0 or x_pix >= self.plot_width or y_pix < 0 or y_pix >= self.plot_height:
                continue
            pixel_index = x_pix + y_pix*self.plot_width
            if pixel_index not in closest_points or z_screen < closest_points[pixel_index]:
                closest_points[pixel_index] = z_screen
                self.e_print("*", x_pix, y_pix, self._plt.colour)

            
