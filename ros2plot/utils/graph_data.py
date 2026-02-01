
from . import colour_palette as COLOURS
from .memory_bounded_deque import MemoryBoundedDeque
import attrs
import math
import numpy as np

TWO_D=0
THREE_D=1

@attrs.define
class GraphConfigs:
    x: float = attrs.field(default=0.0) #origin position, where axis intersect
    y: float = attrs.field(default=0.0)
    z: float = attrs.field(default=0.0)
    roll: float = attrs.field(default=0.0)
    pitch: float = attrs.field(default=0.0)
    yaw: float = attrs.field(default=0.0)
    
    width: int = attrs.field(default=0) #width of the plot area, in pixels
    height: int = attrs.field(default=0) #height of the plot area, in pixels
    y_min_value: int = attrs.field(default=0) # y value corresponding to pixel (x, height-1)
    y_max_value: int = attrs.field(default=0) # y value corresponding to pixel (x, 0)
    x_min_value: int = attrs.field(default=0) # x value corresponding to pixel (0, y)
    x_max_value: int = attrs.field(default=0) # x value corresponding to pixel (width-1, y)
    pause: bool = attrs.field(default=False) # flag to dictate if the effect should pause drawing or no

    render_mode: int = attrs.field(default=THREE_D)


@attrs.define
class RawData:
    data: MemoryBoundedDeque = attrs.field(factory=MemoryBoundedDeque) #bound to 1% available memory

@attrs.define
class PlotData:
    data: MemoryBoundedDeque = attrs.field(factory=MemoryBoundedDeque) #bound to 1% available memory
    x_key: str = attrs.field(default="")
    visible: bool = attrs.field(default=False)
    interpolate: bool = attrs.field(default=True)
    high_def: bool = attrs.field(default=True)
    plot_mean: bool = attrs.field(default=False)
    colour: int = attrs.field(default=COLOURS.DEFAULT)
    minimum: float = attrs.field(default=math.inf)
    maximum: float = attrs.field(default=-math.inf)

@attrs.define
class PlotData3D:
    data: np.ndarray = attrs.field(factory=lambda: np.empty((0, 3), dtype=np.float32))
    # x_key: str = attrs.field(default="")
    # y_key: str = attrs.field(default="")
    # z_key: str = attrs.field(default="")
    colour_transformer_key: str = attrs.field(default="")
    visible: bool = attrs.field(default=False)
    interpolate: bool = attrs.field(default=True)
    high_def: bool = attrs.field(default=False)
    plot_mean: bool = attrs.field(default=False)
    colour: int = attrs.field(default=COLOURS.DEFAULT)
