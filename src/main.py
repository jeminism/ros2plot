

from ros.multisub import MultiSubscriber
from ros2plot import Ros2Plot

from asciimatics.screen import Screen, ManagedScreen

import argparse
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

def set_args(parser):
    parser.add_argument('topic_name', nargs="?", default=None, help='Name of the topic to subscribe')
    parser.add_argument('topic_type', nargs="?", default=None, help='Type of topic to subscribe to. If missing, will internally attempt to automatically determine the topic type.')
    parser.add_argument('--fields', nargs='*', help='Specific fields to plot. Expects directory style path.')
    parser.add_argument('--x-field', nargs=1, help='Specific field to use as x axis. Expects directory style path. If missing, will default to system time')

def main():
    parser = argparse.ArgumentParser()
    set_args(parser)

    args = vars(parser.parse_args(sys.argv[1:]))
    print(args)

    rclpy.init()

    topic_name = args["topic_name"].lstrip("/") if args["topic_name"] != None else None
    topic_type = args["topic_type"]
    fields = [x for x in args["fields"]] if args["fields"]!=None else None
    x_key = args["x_field"][0] if args["x_field"]!=None else None

    
    shutdown = False
    m_sub = MultiSubscriber()
    time.sleep(0.5)
    display = None
    t = threading.Thread(target=ros_spin, args=(m_sub,shutdown), daemon=True)

    t.start()
    while not shutdown:
        with ManagedScreen() as screen:
            if display == None:
                display = Ros2Plot(screen, 3, 2, m_sub)
                if topic_name != None:
                    display.add_subscriber(topic_name, topic_type, fields)
                    if x_key != None:
                        display.set_x_axis_key(topic_name+"/"+x_key)
            else:
                display.set_screen(screen)

            display.run(shutdown)

    t.join()

if __name__ == '__main__':
    main()
