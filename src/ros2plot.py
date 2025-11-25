
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from asciimatics.screen import Screen, ManagedScreen
from asciimatics.exceptions import ResizeScreenError
import sys

from graph import GraphXY

NUMERIC_TYPES = (int, float)

class TopicIntrospector:
    def __init__(self, blacklist=[]):
        self._tree = {}
        self._blacklist = blacklist

    def introspect(self, msg, path):
        if not hasattr(msg, "__slots__"):
            #terminal branch
            if isinstance(msg, NUMERIC_TYPES):
                if path not in self._tree:
                    self._tree[path] = [msg]
                else:
                    self._tree[path].append(msg)
        
        else:
            for s in msg.__slots__:
                skip = False
                child_path = path + "/" + s
                for n in self._blacklist:
                    if n in child_path:
                        skip = True
                        break
                if skip:
                    continue
                else:
                    self.introspect(getattr(msg, s), child_path)
        
    
    def get_data(self):
        return self._tree



class AnySubscriber(Node):
    def __init__(self, screen: Screen, topic_name, topic_type):
        super().__init__("any_sub")
        self._introspector = TopicIntrospector()
        self._graph = GraphXY(screen.width-8, screen.height-2, plot_hd=True)
        self._timestamps = []
        self._screen = screen
        self._subscription = self.create_subscription(
                                topic_type,
                                topic_name,
                                self.listener_callback,
                                10)
        self._first_time = None

        self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(0.05, self.render_graph, callback_group=self._timer_callback_group)

    # def screen(self):
    #     try:
    #         Screen.wrapper(self.render_graph)
    #     except Exception as e:
    #         print(f"screen error: {e}")

    def render_graph(self):
        self._screen.clear()
        if len(self._timestamps)==0:
            self._screen.print_at("NO DATA RECEIVED!", 0,0)
            return
        
        # graph = GraphXY(screen.width-8, screen.height-2, plot_hd=True)
        data = self._introspector.get_data()
        if len(data) == 0:
            raise ValueError("No numeric fields found!")
        
        self._graph.draw(self._screen, 2, 1, self._timestamps, [data[d] for d in data])
        self._screen.refresh()

        # event = screen.get_key()
        # if event in (ord('q'), ord('Q')):
        #     return


    def listener_callback(self, msg):
        if self._first_time == None:
            self._first_time = self.get_clock().now().nanoseconds
        self._timestamps.append(self.get_clock().now().nanoseconds - self._first_time)
        self._introspector.introspect(msg, "")



def application(screen: Screen):
    args = sys.argv[1:]
    if len(args) != 2:
        raise ValueError(f"Expected exactly two input argument; topic_name and topic_type. instead got: [{args}]")
    topic_name = args[0]
    try:
        topic_type = get_message(args[1])
    except:
        raise ValueError(f"Input type {args[1]} does not exist!")

    rclpy.init()
    minimal_subscriber = AnySubscriber(screen, topic_name, topic_type)
    print("start spin!")
    rclpy.spin(minimal_subscriber)

def main():
    args = sys.argv[1:]
    if len(args) != 2:
        raise ValueError(f"Expected exactly two input argument; topic_name and topic_type. instead got: [{args}]")
    topic_name = args[0]
    try:
        topic_type = get_message(args[1])
    except:
        raise ValueError(f"Input type {args[1]} does not exist!")

    rclpy.init()
    with ManagedScreen() as screen:
        minimal_subscriber = AnySubscriber(screen, topic_name, topic_type)
        while True:
            try:
                rclpy.spin(minimal_subscriber)
            except ResizeScreenError:
                continue

if __name__ == '__main__':
    main()