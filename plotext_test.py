import plotext as plt
import time
import os
from datetime import datetime

x = []
y = []
plt.xlabel("datetime")
plt.ylabel("val")
now = datetime.now().timestamp()
while True:
    if len(y) == 0:
        y.append(1)
    else:
        y.append(y[-1]*2)
    x.append(datetime.now().timestamp() - now)
    # os.system("clear")
    plt.plot(x, y)
    plt.show()
    time.sleep(1)