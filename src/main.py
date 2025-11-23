from graph import GraphXY
from asciimatics.screen import Screen

import time
import random
from datetime import datetime

def time_series(screen: Screen):

    t = 0.0
    x = []
    y = []
    now = datetime.now().timestamp()

    graph = GraphXY(screen.width-8, screen.height-2, plot_hd=True)

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
        
        graph.draw(screen, 2, 1, x, y)
        #plot.draw(screen, 5, 4)

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