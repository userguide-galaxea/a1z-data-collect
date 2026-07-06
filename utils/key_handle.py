"""Keyboard event handler for recording control (S/E/R/Q)."""

from pynput import keyboard


def init_keyboard_listener() -> tuple[keyboard.Listener, dict]:
    events = {
        "start_recording": False,
        "finish_recording": False,
        "rerecord": False,
        "stop": False,
    }

    def on_press(key):
        try:
            k = key.char.lower()
        except AttributeError:
            return
        if k == "s":
            events["start_recording"] = True
        elif k == "e":
            events["finish_recording"] = True
        elif k == "r":
            events["rerecord"] = True
            events["finish_recording"] = True
        elif k == "q":
            events["stop"] = True
            events["finish_recording"] = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    return listener, events
