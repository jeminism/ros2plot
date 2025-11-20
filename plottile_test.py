import plotille as plt
import time
from datetime import datetime
import os

x = []
y = []
# plt.xlabel("datetime")
# plt.ylabel("val")
now = datetime.now().timestamp()
while True:
    if len(y) == 0:
        y.append(1)
    else:
        y.append(y[-1]*2)
    x.append(datetime.now().timestamp() - now)
    # plt.plot(x, y)
    # plt.show()
    os.system('clear')
    print(plt.plot(x, y, height=40, width=80, interp="linear"))
    time.sleep(1)