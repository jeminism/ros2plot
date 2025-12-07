from asciimatics.screen import Screen
import math
import time
import random
from datetime import datetime

def draw_graph(screen, x_vals, y_vals):
    if len(x_vals) != len(y_vals):
        return

    # ---- initialize variables ---- TODO: Abstract these into arguments or class data members to clean this up
    # -- define the axes position
    padding = 1
    y_axis = 5
    x_axis = 3
    
    # -- define the graph area
    min_x = padding + y_axis 
    max_x = screen.width - padding - 4
    min_y = screen.height - padding
    max_y = padding

    # -- get the min max values of x and y
    y_val_max = max(y_vals)
    y_val_min = min(y_vals)
    if y_val_max >= 0 and y_val_min >= 0:
        y_val_min = -1
    
    x_val_max = max(x_vals)
    x_val_min = min(x_vals)
    if x_val_max > 0 and x_val_min > 0:
        x_val_min = 0
    

    # ---- draw braille plot first in the viewable area ----
    plot_size_x = max_x - min_x - 1
    plot_size_y = abs(max_y - min_y) - 1

    braille_size_x = plot_size_x*2
    braille_size_y = plot_size_y*4
    braille_grid = [False]*(braille_size_x*braille_size_y)
    prior_x = -1
    prior_y = -1
    for i in range(len(x_vals)):
        pt_x = get_screen_index(x_vals[i], x_val_max, braille_size_x-1, x_val_min, 0)
        pt_y = get_screen_index(y_vals[i], y_val_max, 0, y_val_min, braille_size_y-1)
        braille_grid[coordinate_to_index(pt_x,pt_y,braille_size_x)] = True
        if i > 0:
            dx = pt_x - prior_x
            dy = pt_y - prior_y
            d = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx)
            c = 1
            while c < d:
                n_x = prior_x + int(c*math.cos(angle))
                n_y = prior_y + int(c*math.sin(angle))
                braille_grid[coordinate_to_index(n_x,n_y,braille_size_x)] = True
                c += 1
        prior_x = pt_x
        prior_y = pt_y
    
    for i in range(plot_size_x):
        for j in range(plot_size_y):
            b_x = i*2
            b_y = j*4
            braille_cells = []

            dot_offsets = [
                (0, 0), (0, 1), (0, 2),
                (1, 0), (1, 1), (1, 2),
                (0, 3), (1, 3),
            ]

            for dot_num, (dx, dy) in enumerate(dot_offsets, start=1):
                if braille_grid[coordinate_to_index(b_x + dx, b_y + dy, braille_size_x)]:
                    braille_cells.append(dot_num)

            if len(braille_cells) > 0:
                screen.print_at(braille(braille_cells), 
                                min_x + 1 + i, max_y + j) 

    
    # ---- Get key positions for labels on x and y axes
    x_0_pos = get_screen_index(0, x_val_max, max_x-1, x_val_min, min_x)
    y_0_pos = get_screen_index(0, y_val_max, max_y, y_val_min, min_y-1)
    x_latest_pos = get_screen_index(x_vals[-1], x_val_max, max_x-1, x_val_min, min_x)
    y_latest_pos = get_screen_index(y_vals[-1], y_val_max, max_y, y_val_min, min_y-1)

    n_x_labels = 10
    x_label_interval = plot_size_x / n_x_labels
    # x_label_pos = [int(i*x_label_interval) for i in range(1,10)] # intentionally skip 0 since 0 is indicated by the y axis position
    # x_label_pos.append(plot_size_x)

    # --- Draw Y-axis (vertical) ---
    for ypix in range(max_y, min_y):
        screen.print_at("|", x_0_pos, ypix)

    # --- Draw X-axis (horizontal) ---
    for xpix in range(min_x, max_x):
        screen.print_at("-", xpix, y_0_pos)

    # ---- Add Y Labels -----
    screen.print_at(f"{0.0:3.1f}", x_0_pos-y_axis, y_0_pos)
    screen.print_at(f"{y_val_max:3.1f}", x_0_pos-y_axis, max_y)
    screen.print_at(f"{y_val_min:3.1f}", x_0_pos-y_axis, min_y)
    screen.print_at(f"{y_vals[-1]:3.1f}", x_0_pos-y_axis, y_latest_pos)

    # ---- Add X Labels -----
    screen.print_at(f"{x_vals[-1]:3.1f}", x_latest_pos, y_0_pos+1)
    screen.print_at(f"{x_val_max:3.1f}", max_x-1, y_0_pos+1)
    # for xlabel in x_label_pos:
    #     screen.print_at(f"{(xlabel/plot_size_x)*(x_val_max-x_val_min) + x_val_min:3.3f}", xlabel, y_0_pos+1)


def coordinate_to_index(x, y, width):
    return x + y*width

def braille(dots):
    code = 0x2800
    for d in dots:
        code += 1 << (d - 1)
    return chr(code)
    

def get_screen_index(ref_val, max_val, max, min_val=0, min=0):
    if ref_val > max_val:
        return max
    elif ref_val < min_val:
        return min
    else:
        return int((ref_val-min_val)/(max_val-min_val) * (max - min)) + min

def time_series(screen: Screen):
    max_points = screen.width - 6    # Leave space for Y-axis labels

    t = 0.0
    x = []
    y = []
    now = datetime.now().timestamp()

    while True:
        screen.clear()

        if len(y) < 1000:
            if len(y) == 0:
                y.append(0.0)
            else:
                if y[-1] < 0:
                    y.append(y[-1] + random.randrange(-8, 10))
                else:
                    y.append(y[-1] + random.randrange(-10, 8))
            x.append(datetime.now().timestamp() - now)
        
        draw_graph(screen, x, y)

        screen.refresh()
        time.sleep(0.05)
        # if len(y) > 500:
        #     time.sleep(0.5)
        # else:
        #     time.sleep(0.01)

        # Quit on 'q'
        event = screen.get_key()
        if event in (ord('q'), ord('Q')):
            return

while True:
    try:
        Screen.wrapper(time_series)
        break
    except ResizeScreenError:
        continue