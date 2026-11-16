"""Crash-safe, resumable capture session.

Everything is an append-only event log (events.jsonl, fsync'd after every event).
State (current matrix, capture counter, finished flag) is *re-derived by replaying the log*,
so a freeze / Ctrl-C / power-cut / SSH drop can never corrupt it. A capture directory
is written to a temp name and atomically renamed, so half-written captures never exist.
"""
from __future__ import annotations
import json, os, shutil, time
from pathlib import Path


class SessionStore:
    def __init__(self, root):
        self.root = Path(root)
        self.cap_dir = self.root / "captures"
        self.trash_dir = self.root / "trash"
        self.events_path = self.root / "events.jsonl"
        self.root.mkdir(parents=True, exist_ok=True)
        self.cap_dir.mkdir(exist_ok=True)
        self._cleanup_partial()
        self.events = self._load_events()

    def _cleanup_partial(self):
        for p in self.cap_dir.glob(".tmp_*"):
            shutil.rmtree(p, ignore_errors=True)

    def _load_events(self):
        ev = []
        if self.events_path.exists():
            with open(self.events_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        ev.append(json.loads(line))
                    except json.JSONDecodeError:
                        break  # truncated last line from a crash -> ignore
        return ev

    def append_event(self, ev: dict):
        ev = dict(ev, wall_time=time.time())
        with open(self.events_path, "a") as f:
            f.write(json.dumps(ev) + "\n")
            f.flush()
            os.fsync(f.fileno())
        self.events.append(ev)

    def state(self) -> dict:
        matrix, next_id, finished = 0, 0, False
        live = {}
        undone = set()
        for e in self.events:
            t = e["type"]
            if t == "capture":
                live[e["id"]] = e
                next_id = max(next_id, e["id"] + 1)
                matrix = e["matrix"]
            elif t == "next_matrix":
                matrix = e["matrix"]
            elif t == "undo":
                live.pop(e["id"], None)
                undone.add(e["id"])
            elif t == "finish":
                finished = True
            elif t == "reopen":
                finished = False
        # adopt capture dirs that were renamed but whose event line was lost in a crash
        for d in self.cap_dir.glob("cap_*"):
            try:
                cid = int(d.name.split("_")[1])
            except ValueError:
                continue
            if cid not in live and cid not in undone and (d / "meta.json").exists():
                meta = json.loads((d / "meta.json").read_text())
                live[cid] = {"type": "capture", "id": cid, "matrix": meta.get("matrix", matrix)}
                next_id = max(next_id, cid + 1)
        per_matrix = {}
        for e in live.values():
            per_matrix[e["matrix"]] = per_matrix.get(e["matrix"], 0) + 1
        return {"matrix": matrix, "next_id": next_id, "finished": finished,
                "live_ids": sorted(live), "per_matrix": per_matrix,
                "n_captures": len(live), "last_id": max(live) if live else None}

    def begin_capture(self):
        cid = self.state()["next_id"]
        tmp = self.cap_dir / f".tmp_cap_{cid:04d}"
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir()
        return cid, tmp

    def commit_capture(self, cid: int, tmp: Path, meta: dict):
        (tmp / "meta.json").write_text(json.dumps(meta, indent=1))
        final = self.cap_dir / f"cap_{cid:04d}"
        if final.exists():
            shutil.rmtree(final)
        os.rename(tmp, final)
        self.append_event({"type": "capture", "id": cid, "matrix": meta["matrix"]})

    def next_matrix(self) -> int:
        m = self.state()["matrix"] + 1
        self.append_event({"type": "next_matrix", "matrix": m})
        return m

    def undo_last(self):
        st = self.state()
        if st["last_id"] is None:
            return None
        cid = st["last_id"]
        self.trash_dir.mkdir(exist_ok=True)
        src = self.cap_dir / f"cap_{cid:04d}"
        if src.exists():
            shutil.move(str(src), str(self.trash_dir / f"cap_{cid:04d}_{int(time.time())}"))
        self.append_event({"type": "undo", "id": cid})
        return cid

    def finish(self):
        self.append_event({"type": "finish"})

    def reopen(self):
        self.append_event({"type": "reopen"})

    def load_capture_meta(self, cid: int) -> dict:
        return json.loads((self.cap_dir / f"cap_{cid:04d}" / "meta.json").read_text())

    def capture_path(self, cid: int) -> Path:
        return self.cap_dir / f"cap_{cid:04d}"


def find_latest_session(data_root):
    root = Path(data_root)
    cands = sorted([p for p in root.glob("session_*") if p.is_dir()], key=lambda p: p.stat().st_mtime)
    return cands[-1] if cands else None


def new_session_dir(data_root, name=None) -> Path:
    name = name or time.strftime("session_%Y%m%d_%H%M%S")
    return Path(data_root) / name


# ------------------------------------------------------------------ teaching sessions (segments/*.csv)
def latest_teach_session(data_root):
    root = Path(data_root)
    c = sorted([p for p in root.glob("session_*") if (p / "segments").is_dir()], key=lambda p: p.stat().st_mtime)
    return c[-1] if c else None


def next_segment_id(seg_dir) -> int:
    ids = []
    for p in Path(seg_dir).glob("seg_????.csv"):
        try:
            ids.append(int(p.stem.split("_")[1]))
        except ValueError:
            pass
    return max(ids) + 1 if ids else 0


def choose_teach_session(data_root, session_arg=None, new=False, ask=input) -> Path:
    """--session X  -> that one;  --new -> fresh;  otherwise offer to resume the latest teaching session."""
    root = Path(data_root)
    root.mkdir(parents=True, exist_ok=True)
    if session_arg:
        p = Path(session_arg)
        p = p if p.exists() or p.is_absolute() else root / session_arg
    elif new:
        p = new_session_dir(root)
    else:
        last = latest_teach_session(root)
        if last is not None:
            n = len(list((last / "segments").glob("seg_????.csv")))
            ans = ask(f"Found session {last.name} with {n} segment(s). Resume it? [Y/n] (n = new session): ").strip().lower()
            p = last if ans in ("", "y", "yes") else new_session_dir(root)
        else:
            p = new_session_dir(root)
    (p / "segments").mkdir(parents=True, exist_ok=True)
    return p
