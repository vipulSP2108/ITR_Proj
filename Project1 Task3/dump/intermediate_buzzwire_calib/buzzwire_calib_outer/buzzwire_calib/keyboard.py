"""Game-style key input for a terminal.

Two backends:
  * "terminal" (default, works over SSH, only reacts while THIS terminal has focus - the safe choice).
      A terminal cannot see key-release, so a key counts as 'held' for `hold_s` after its last
      auto-repeat character.  A quick tap = a short deterministic nudge (speed*hold_s);
      holding = continuous motion (after the OS auto-repeat delay).  Tip: `xset r rate 200 40` shortens it.
  * "pynput" (true key-down/key-up like a game; needs a desktop session; reacts GLOBALLY - even if another
      window has focus - so use it only if you are careful).
Keys are lower-case chars plus: 'space', 'tab', 'enter', 'esc'.
"""
from __future__ import annotations
import select, sys, threading, time, queue


class KeyInput:
    TOGGLE_DEBOUNCE_S = 0.35

    def __init__(self, backend="terminal", hold_s=0.12):
        self.backend, self.hold_s = backend, hold_s
        self._expiry: dict[str, float] = {}
        self._held_hard: set[str] = set()
        self._events: "queue.Queue[str]" = queue.Queue()
        self._last_event: dict[str, float] = {}
        self._stop = threading.Event()
        self._old = None
        self._tty = sys.stdin.isatty()
        self._listener = None

    # ---- lifecycle
    def __enter__(self):
        if self._tty:
            import termios, tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        if self.backend == "pynput":
            try:
                from pynput import keyboard as kb
                self._kb = kb
                self._listener = kb.Listener(on_press=self._pn_press, on_release=self._pn_release)
                self._listener.start()
                threading.Thread(target=self._drain_stdin, daemon=True).start()
                return self
            except Exception as e:
                print(f"[keys] pynput unavailable ({e}); falling back to terminal keys")
                self.backend = "terminal"
        threading.Thread(target=self._run_terminal, daemon=True).start()
        return self

    def __exit__(self, *a):
        self._stop.set()
        if self._listener:
            self._listener.stop()
        if self._old is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    # ---- terminal backend
    def _run_terminal(self):
        while not self._stop.is_set():
            try:
                if not self._tty:
                    line = sys.stdin.readline()
                    if not line:
                        return
                    for ch in line.strip():
                        self._on_char(ch)
                    continue
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    ch = sys.stdin.read(1)
                    if ch == "\x1b":                       # swallow arrow-key escape sequences
                        r2, _, _ = select.select([sys.stdin], [], [], 0.02)
                        if r2:
                            while select.select([sys.stdin], [], [], 0.005)[0]:
                                sys.stdin.read(1)
                            continue
                    self._on_char(ch)
            except Exception:
                return

    def _on_char(self, ch):
        name = {" ": "space", "\t": "tab", "\r": "enter", "\n": "enter", "\x1b": "esc"}.get(ch, ch.lower())
        now = time.time()
        fresh = self._expiry.get(name, 0.0) < now
        self._expiry[name] = now + self.hold_s
        if fresh and now - self._last_event.get(name, 0.0) > self.TOGGLE_DEBOUNCE_S:
            self._last_event[name] = now
            self._events.put(name)

    def _drain_stdin(self):
        while not self._stop.is_set():
            if self._tty and select.select([sys.stdin], [], [], 0.1)[0]:
                sys.stdin.read(1)

    # ---- pynput backend
    def _name(self, key):
        kb = self._kb
        if hasattr(key, "char") and key.char:
            return key.char.lower()
        return {kb.Key.space: "space", kb.Key.tab: "tab", kb.Key.enter: "enter", kb.Key.esc: "esc"}.get(key)

    def _pn_press(self, key):
        n = self._name(key)
        if n and n not in self._held_hard:
            self._held_hard.add(n)
            self._events.put(n)

    def _pn_release(self, key):
        self._held_hard.discard(self._name(key))

    # ---- public
    def held(self, key: str) -> bool:
        if self.backend == "pynput":
            return key in self._held_hard
        return self._expiry.get(key, 0.0) > time.time()

    def events(self):
        """Newly pressed keys since the last call (edge-triggered; use for toggles/commands)."""
        out = []
        while True:
            try:
                out.append(self._events.get_nowait())
            except queue.Empty:
                return out
