import math

def get_mapped_value(value, max_value, max_mapped, min_value, min_mapped):
    # if not value <= max_value and not value >= min_value:
    #     raise ValueError("Mapped value must be within bounds")
    if max_value == min_value:
        return max_value
    return type(max_mapped)((value - min_value)/(max_value - min_value) * (max_mapped - min_mapped) + min_mapped)

def min_max(values: list):
    min = math.inf
    max = -math.inf
    for val in values:
        if val < min:
            min = val
        if val > max:
            max = val
    return min, max

def multi_min_max(data: list[list]):
    min = math.inf
    max = -math.inf
    try:
        for values in data:
            for val in values:
                if val < min:
                    min = val
                if val > max:
                    max = val
    except TypeError as e:
        raise TypeError(f"{e}. val: {val}, min: {min}, max: {max}")
    return min, max

def bresenham(x1: int, y1: int, x0: int, y0: int) -> tuple[tuple[int, int]]:
    points = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    x, y = x0, y0
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1

    if dy <= dx:
        # Gentle slope (|slope| ≤ 1)
        err = dx // 2
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        points.append((x, y))  # final point
    else:
        # Steep slope (|slope| > 1)
        err = dy // 2
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        points.append((x, y))  # final point

    return points
        

