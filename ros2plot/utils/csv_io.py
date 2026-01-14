
from .graph_data import PlotData

import csv
from datetime import datetime

def csv_to_plotdata(filename, plot_data_dict):
    try:
        with open(filename, mode="r") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                # print(row)
                for field,value_str in row.items():
                    value = float(value_str) if "." in value_str else int(value_str)
                    if field not in plot_data_dict:
                        plot_data_dict[field] = PlotData()
                        plot_data_dict[field].data.set_configs(max_fraction=0.02, trim_fraction=0.05)
                    plot_data_dict[field].data.append(value)
                    if value < plot_data_dict[field].minimum:
                        plot_data_dict[field].minimum = value 
                    if value > plot_data_dict[field].maximum:
                        plot_data_dict[field].maximum = value 
    except Exception as e:
        raise Exception(f"{e}. field: {field}, value: {value}")

def filename_gen():
    return "ros2plot_stats_"+ datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + ".csv"

def write_to_csv(filename, data):
    with open(filename, mode="a") as f:
        writer = csv.writer(f)
        writer.writerow(data)
    
def init_plot_stats_csv():
    filename = filename_gen()
    headers = ["timestamp", "data_size", "num_plots", "frame_time", "screen_width", "screen_height"]
    write_to_csv(filename, headers)
    return filename

def write_plot_stats_to_csv(filename, timestamp, plot_data_dict, frame_time, screen_width, screen_height):
    data_size = 0
    num_plots = 0
    for field,plot_data in plot_data_dict.items():
        if not plot_data.visible:
            continue
        data_size += len(plot_data.data)
        num_plots += 1
    row_data = [timestamp, data_size, num_plots, frame_time, screen_width, screen_height]
    write_to_csv(filename, row_data)


