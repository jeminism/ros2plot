
from .memory_bounded_deque import MemoryBoundedDeque
from . import colour_palette as COLOURS

import attrs
import math

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
    plot_mean: bool = attrs.field(default=False)
    colour: int = attrs.field(default=COLOURS.DEFAULT)
    minimum: float = attrs.field(default=math.inf)
    maximum: float = attrs.field(default=-math.inf)
