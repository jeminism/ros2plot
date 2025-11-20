from asciimatics.screen import Screen
import math
import time
import random
from datetime import datetime

def plot(screen, x_vals, y_vals):
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

    # --- Draw Y-axis (vertical) ---
    for ypix in range(padding, screen.height-padding):
        if ypix % y_label_interval == 0:
            n = ypix / y_label_interval - 1
            screen.print_at("-", get_screen_index(0, x_val_max, max_x, x_val_min, min_x), ypix)
            screen.print_at(f"{((n_y_labels-n)*(y_val_max - y_val_min)/n_y_labels + y_val_min):2.2}", get_screen_index(0, x_val_max, max_x, x_val_min, min_x)-2, ypix)
        else:
            screen.print_at("|", get_screen_index(0, x_val_max, max_x, x_val_min, min_x), ypix)
    # --- Draw X-axis (horizontal) ---
    for xpix in range(padding, screen.width-padding):
        if xpix % x_label_interval == 0:
            n = xpix / x_label_interval + 1
            screen.print_at("|", xpix, get_screen_index(0, y_val_max, max_y, y_val_min, min_y))
            screen.print_at(f"{(n*(x_val_max - x_val_min)/n_x_labels + x_val_min):3.5}", xpix, get_screen_index(0, y_val_max, max_y, y_val_min, min_y)+1)
        else:
            screen.print_at("-", xpix, get_screen_index(0, y_val_max, max_y, y_val_min, min_y))

    cum_val = 0
    cum_n = 0
    bucket_interval = len(x_vals) / (max_x-min_x)
    bucket_i = 1
    for i in range(len(x_vals)):
        # screen.print_at("*", get_screen_index(x_vals[i], x_val_max, max_x, x_val_min, min_x), 
        #                      get_screen_index(y_vals[i], y_val_max, max_y, y_val_min, min_y))
        if len(x_vals) > (max_x - min_x):
            if (math.floor(i / bucket_interval) > bucket_i):
                screen.print_at("*", bucket_i + min_x, 
                                     get_screen_index(cum_val/cum_n, y_val_max, max_y, y_val_min, min_y))
                cum_val = y_vals[i]
                cum_n = 1
                bucket_i += 1
            else:
                cum_val += y_vals[i]
                cum_n += 1

        else:
            screen.print_at("*", get_screen_index(x_vals[i], x_val_max, max_x, x_val_min, min_x), 
                                 get_screen_index(y_vals[i], y_val_max, max_y, y_val_min, min_y))

    

def get_screen_index(ref_val, max_val, max, min_val=0, min=0):
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