from asciimatics.screen import Screen
import math
import time
import random
from datetime import datetime

def plot(screen, x_vals, y_vals):
def draw_graph(screen, x_vals, y_vals):
    if len(x_vals) != len(y_vals):
        return

    padding = 2
    y_axis = 3
    x_axis = 3
    
    min_x = padding + y_axis 
    max_x = screen.width - padding - 1
    min_y = screen.height - padding - x_axis - 1
    max_y = padding

    y_val_max = max(y_vals)
    y_val_min = min(y_vals)
    if y_val_max >= 0 and y_val_min >= 0:
        y_val_min = -1
    # if len(y_vals) == 1:
    #     if y_val_max > 0:
    #         y_val_min = 0
    #     else:
    #         y_val_max = 0
    
    x_val_max = max(x_vals)
    x_val_min = min(x_vals)
    if x_val_max > 0 and x_val_min > 0:
        x_val_min = 0
    # if len(x_vals) == 1:
    #     if x_val_max > 0:
    #         x_val_min = 0
    #     else:
    #         x_val_max = 0
    

    y_label_interval = 8
    x_label_interval = 10
    n_y_labels = abs(max_y - min_y) // y_label_interval
    n_x_labels = abs(max_x - min_x) // x_label_interval

    # ---- draw plot --
    plot_size_x = max_x - min_x - 1
    plot_size_y = abs(max_y - min_y) - 1
    cum_val = 0
    cum_n = 0
    # bucket_interval = len(x_vals) / plot_size_x
    # bucket_i = 1
    # bucket_vals = [-1]*plot_size_x
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
            # braille_cells = [ coordinate_to_index(b_x, b_y   ,braille_size_x), coordinate_to_index(b_x+1, b_y   ,braille_size_x),
            #                   coordinate_to_index(b_x, b_y+1 ,braille_size_x), coordinate_to_index(b_x+1, b_y+1 ,braille_size_x),
            #                   coordinate_to_index(b_x, b_y+2 ,braille_size_x), coordinate_to_index(b_x+1, b_y+2 ,braille_size_x),
            #                   coordinate_to_index(b_x, b_y+3 ,braille_size_x), coordinate_to_index(b_x+1, b_y+3 ,braille_size_x) ]
            braille_cells = []

            if braille_grid[coordinate_to_index(b_x, b_y   ,braille_size_x)] == True:
                braille_cells.append(1) 
            if braille_grid[coordinate_to_index(b_x, b_y+1 ,braille_size_x)] == True:
                braille_cells.append(2) 
            if braille_grid[coordinate_to_index(b_x, b_y+2 ,braille_size_x)] == True:
                braille_cells.append(3) 
            if braille_grid[coordinate_to_index(b_x+1, b_y   ,braille_size_x)] == True:
                braille_cells.append(4) 
            if braille_grid[coordinate_to_index(b_x+1, b_y+1   ,braille_size_x)] == True:
                braille_cells.append(5) 
            if braille_grid[coordinate_to_index(b_x+1, b_y+2   ,braille_size_x)] == True:
                braille_cells.append(6) 
            if braille_grid[coordinate_to_index(b_x, b_y+3   ,braille_size_x)] == True:
                braille_cells.append(7) 
            if braille_grid[coordinate_to_index(b_x+1, b_y+3   ,braille_size_x)] == True:
                braille_cells.append(8) 

            if len(braille_cells) > 0:
                screen.print_at(braille(braille_cells), 
                                min_x + 1 + i, max_y + j) 
        # bucket_index = get_screen_index(x_vals[i], x_val_max, max_x, x_val_min, min_x)
        # screen.print_at("*", get_screen_index(x_vals[i], x_val_max, max_x, x_val_min, min_x), 
        #                      get_screen_index(y_vals[i], y_val_max, max_y, y_val_min, min_y))
        # if len(x_vals) > (max_x - min_x):
        #     if (math.floor(i / bucket_interval) > bucket_i):
        #         bucket_vals.[bucket_i-1] = cum_val/cum_n
        #         cum_val = y_vals[i]
        #         cum_n = 1
        #         bucket_i += 1
        #     else:
        #         cum_val += y_vals[i]
        #         cum_n += 1

        # else:
        #     bucket_vals[get_screen_index(x_vals[i], x_val_max, max_x, x_val_min, min_x)] = y_vals[i]

        
    # --- Draw Y-axis (vertical) ---
    for ypix in range(padding, screen.height-padding):
        screen.print_at("|", get_screen_index(0, x_val_max, max_x, x_val_min, min_x), ypix)
        if ypix % y_label_interval == 0:
            n = ypix / y_label_interval - 1
            # screen.print_at("-", get_screen_index(0, x_val_max, max_x, x_val_min, min_x), ypix)
            screen.print_at(f"{((n_y_labels-n)*(y_val_max - y_val_min)/n_y_labels + y_val_min):3.1f}", get_screen_index(0, x_val_max, max_x, x_val_min, min_x)-y_axis, ypix)
        # else:
        #     screen.print_at("|", get_screen_index(0, x_val_max, max_x, x_val_min, min_x), ypix)
    # --- Draw X-axis (horizontal) ---
    y_0_pos = get_screen_index(0, y_val_max, max_y, y_val_min, min_y)
    for xpix in range(padding, screen.width-padding):
        screen.print_at("-", xpix, y_0_pos)
        if xpix % x_label_interval == 0:
            n = xpix / x_label_interval + 1
            # screen.print_at("|", xpix, get_screen_index(0, y_val_max, max_y, y_val_min, min_y))
            screen.print_at(f"{(n*(x_val_max - x_val_min)/n_x_labels + x_val_min):3.1f}", xpix, y_0_pos+1)
        # else:
        #     screen.print_at("-", xpix, get_screen_index(0, y_val_max, max_y, y_val_min, min_y))

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
                    y.append(y[-1] + random.randrange(-5, 10))
                else:
                    y.append(y[-1] + random.randrange(-10, 10))
            x.append(datetime.now().timestamp() - now)
        
        plot(screen, x, y)

        screen.refresh()
        if len(y) > 500:
            time.sleep(0.5)
        else:
            time.sleep(0.01)

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