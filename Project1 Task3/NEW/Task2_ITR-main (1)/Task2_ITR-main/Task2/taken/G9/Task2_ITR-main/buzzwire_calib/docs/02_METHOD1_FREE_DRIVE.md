# 02 · Method 1 – hand-guided teaching (free_drive)

Robot: `./free_drive`. Laptop: `./buzzwire_teach.sh record`.

| key | action |
|---|---|
| **SPACE** | gripper open ↔ close (stops the stick from falling out – close it once the stick is seated) |
| p | pause / resume recording |
| x | mark CONTACT (you heard the buzz) – merge removes that region |
| q / Ctrl-C | stop, save |

* Starts recording immediately after ENTER. 20 Hz, samples that moved < 1 mm are skipped. Each sample stores position, orientation, 6 joints, gripper state.
* One run = one file `seg_NNNN.csv`. Flushed every sample → a crash loses ≤ 50 ms.
* **If you touch the wire or the arm freezes:** press `q` (or just kill it), free-drive back to a safe point *before* the problem, run `record` again. Same session, new segment. `merge` joins them (trims the contact, bridges the small gap).
* Joint states stale (free_drive stopped)? The recorder shows a warning and waits; nothing is lost.
* The arm drifting? Gravity compensation holds position; pause with `p` if you must let go.

## Tips
* Practise the move first with recording paused (`p`).
* Go slow through the tight U-bends: slow motion = clean data and an accurate replay.
* Keep the stick roughly the same orientation relative to the wire as you want in replay – orientation is recorded and replayed.
* Record the wire in 3–6 segments (e.g. one per valley) rather than one long run; overlaps of 1–3 cm at the joints help.
* Mixed teaching: free-drive the long legs, teleop the tight bends – merge treats both the same.
