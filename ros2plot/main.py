
from .utils import get_args, TOPIC_NAME, TOPIC_TYPE, FIELDS, X_FIELD, CSV, LOG_STATS, csv_to_plotdata
from .ros import MultiSubscriber
from .ros2plot import Ros2Plot

from asciimatics.screen import Screen, ManagedScreen

# import argparse
import rclpy
import sys
import time
import threading

def ros_spin(node, shutdown):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        shutdown = True
        return

def main():
    args = get_args(sys.argv[1:])
    print(args)

    rclpy.init()

    topic_name = args[TOPIC_NAME]
    topic_type = args[TOPIC_TYPE]
    fields = args[FIELDS]
    x_key = args[X_FIELD]
    csv = args[CSV]
    log_stats = args[LOG_STATS]
    
    shutdown = False
    m_sub = MultiSubscriber()
    time.sleep(0.5)
    display = None
    t = threading.Thread(target=ros_spin, args=(m_sub,shutdown), daemon=True)

    t.start()
    while not shutdown:
        with ManagedScreen() as screen:
            if display == None:
                display = Ros2Plot(screen, 3, 2, m_sub, log_stats)
                if csv != None:
                    csv_to_plotdata(csv, display.data)
                    print(display.data)
                    display.initialize_plots(topic_filters=fields)
                    if x_key != None:
                        display.set_x_axis_key(x_key)
                    else:
                        if len(display.data) > 0:
                            first_field_key = next(iter(display.data.keys()))
                            display.set_x_axis_key(first_field_key)
                elif topic_name != None:
                    display.add_subscriber(topic_name, topic_type, fields)
                    if x_key != None:
                        display.set_x_axis_key(topic_name+"/"+x_key)
            else:
                display.set_screen(screen)

            display.run(shutdown)

    t.join()

if __name__ == '__main__':
    main()
