
from asciimatics.screen import ManagedScreen
from asciimatics.scene import Scene
from asciimatics.exceptions import NextScene
from effects.legend import PopUpLegend

import threading
import time

def toggle_legend(screen, base_scene):
    try:
        legend = PopUpLegend(screen, screen.width//2, screen.height//2, ["test labe1l ", "another", "this is a little bit longer"])
        while True:
            time.sleep(1.0)
            if len(base_scene.effects) > 0:
                legend.close()
                base_scene.remove_effect(legend)
            else:
                base_scene.add_effect(legend)
            screen.force_update()
            screen.draw_next_frame()
    except (KeyboardInterrupt, ExternalShutdownException):
        return
        

with ManagedScreen() as screen:
    base_scene = Scene([], -1, name="empty")
    legend = PopUpLegend(screen, screen.width//2, screen.height//2, ["test labe1l ", "another", "this is a little bit longer"])

    scenes = [base_scene]
    def on_key(event):
        try:
            key = event.key_code
            if key in (ord('p'), ord('P')):
                if legend in base_scene.effects:
                    print("remove")
                    legend.close()
                    base_scene.remove_effect(legend)
                else:
                    print("add")
                    base_scene.add_effect(legend)
                # screen.force_update()
                
            if key in (ord('q'), ord('Q')):
                raise StopApplication("Quit")
        except:
            return

    t = threading.Thread(target=toggle_legend, args=(screen, base_scene), daemon=True,)
    t.start()
    screen.play(scenes)
    t.join()

#######################################################

# from asciimatics.screen import Screen
# from asciimatics.scene import Scene
# from asciimatics.effects import Effect
# from asciimatics.widgets import Frame, Layout, Text, Button
# from asciimatics.exceptions import StopApplication

# class MovingText(Effect):
#     """Background animation."""
#     def __init__(self, screen):
#         super().__init__(screen)
#         self.x = 0

#     def _update(self, frame_no):
#         self._screen.print_at("BACKGROUND RUNNING", self.x, 2)
#         self.x = (self.x + 1) % (self._screen.width - 20)

#     def reset(self):
#         pass

#     @property
#     def stop_frame(self):
#         return 0

# class Popup(Frame):
#     def __init__(self, screen, on_close):
#         super().__init__(screen, 7, 40,
#                          x=(screen.width - 40)//2,
#                          y=(screen.height - 7)//2,
#                          has_border=True,
#                          title="Input")
#         self.on_close = on_close

#         layout = Layout([1])
#         self.add_layout(layout)
#         self.textbox = Text("Value:")
#         layout.add_widget(self.textbox)

#         buttons = Layout([1, 1, 1])
#         self.add_layout(buttons)
#         buttons.add_widget(Button("OK", self.close), 1)

#         self.fix()

#     def close(self):
#         self.on_close(self.textbox.value)


# def main(screen):
#     popup = None

#     def show_popup():
#         nonlocal popup
#         popup = Popup(screen, on_popup_close)
#         scene.effects.append(popup)

#     def on_popup_close(value):
#         nonlocal popup
#         # Remove popup from screen
#         scene.effects.remove(popup)
#         popup = None

#     # Our base effects
#     effects = [
#         MovingText(screen)
#     ]

#     scene = Scene(effects, -1)

#     # Run one frame first, then trigger popup
#     def run(screen):
#         screen.play([scene], stop_on_resize=True, unhandled_input=on_key)

#     def on_key(event):
#         try:
#             key = event.key_code
#             if key in (ord('p'), ord('P')):
#                 show_popup()
#             if key in (ord('q'), ord('Q')):
#                 raise StopApplication("Quit")
#         except:
#             return

#     screen.play([scene], stop_on_resize=True, unhandled_input=on_key)

# Screen.wrapper(main)