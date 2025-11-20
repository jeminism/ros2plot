from asciimatics.screen import Screen
import math
import time


def time_series(screen: Screen):
    plot_data = []
    max_points = screen.width - 6    # Leave space for Y-axis labels

    t = 0.0

    while True:
        screen.clear()

        # Generate new sample
        y = math.sin(t)
        t += 0.15

        plot_data.append(y)
        if len(plot_data) > max_points:
            plot_data.pop(0)

        # Screen geometry
        mid = screen.height // 2
        amp = (screen.height // 2) - 2   # 1-line padding top/bottom

        # --- Draw Y-axis (vertical) ---
        for ypix in range(screen.height):
            screen.print_at("|", 5, ypix)

        # --- Draw X-axis (horizontal) ---
        for xpix in range(screen.width):
            screen.print_at("-", xpix, mid)

        # --- Draw Y-axis labels ---
        screen.print_at(f"{1.0:>4}", 0, 1)                    # max
        screen.print_at(f"{0.0:>4}", 0, mid)                  # zero
        screen.print_at(f"{-1.0:>4}", 0, screen.height - 2)   # min

        # --- Plot the time series ---
        for i, val in enumerate(plot_data):
            x = i + 6             # offset to the right of the Y-axis
            y_screen = mid - int(val * amp)
            if 0 <= y_screen < screen.height:
                screen.print_at("*", x, y_screen)

        # --- X-axis time labels (every 20 samples) ---
        for i in range(0, len(plot_data), 20):
            label = str(i)
            ypos = mid + 1
            xpos = i + 6
            if xpos + len(label) < screen.width:
                screen.print_at(label, xpos, ypos)

        screen.refresh()
        time.sleep(0.03)

        # Quit on 'q'
        event = screen.get_key()
        if event in (ord('q'), ord('Q')):
            return


Screen.wrapper(time_series)