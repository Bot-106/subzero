"""HRI CLI: keys -> /subzero/task/request (C.3); prints every /subzero/task/state. `uv run python hri_cli.py --server 127.0.0.1`"""
import argparse, json, sys, threading, time
import ntcore

KEYS = {
    "a": ("alignToTag", {"tagId": 3, "offsetX_m": 0.45, "offsetY_m": 0.0, "yaw_deg": 180}),
    "e": ("elevatorTo", {"height_m": 0.8}),
    "r": ("armTo", {"extension_m": 0.3}),
    "l": ("toolOp", {"tool": 1, "op": "latch", "arg": 0}),
    "u": ("toolOp", {"tool": 1, "op": "release", "arg": 0}),
    "s": ("stow", {}),
    "x": ("abort", {}),
}
HELP = "a alignToTag | e elevatorTo 0.8 | r armTo 0.3 | l latch | u release | s stow | x abort | q quit"


def state_printer(sub, stop):
    while not stop.is_set():
        for ts in sub.readQueue():
            print(f"\r[state] {ts.value}\n> ", end="", flush=True)
        time.sleep(0.02)


def main(argv=None):
    p = argparse.ArgumentParser(description=HELP)
    p.add_argument("--server", default="127.0.0.1", help="127.0.0.1 (sim) or 10.13.60.2 (RoboRIO)")
    p.add_argument("--port", type=int, default=0)
    a = p.parse_args(argv)
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("subzero-hri-cli")
    inst.setServer(a.server, a.port)
    opts = ntcore.PubSubOptions(keepDuplicates=True, periodic=0.02, sendAll=True, pollStorage=64)
    req = inst.getStringTopic("/subzero/task/request").publish(opts)
    state = inst.getStringTopic("/subzero/task/state").subscribe("", opts)
    stop = threading.Event()
    threading.Thread(target=state_printer, args=(state, stop), daemon=True).start()
    print(HELP)
    seq = 0
    try:
        while True:
            key = input("> ").strip().lower()[:1]
            if key == "q":
                break
            if key not in KEYS:
                print(HELP if key else f"connected={inst.isConnected()}")
                continue
            seq += 1
            prim, args = KEYS[key]
            msg = json.dumps({"seq": seq, "primitive": prim, "args": args})
            req.set(msg)
            print(f"[request] {msg}" + ("" if inst.isConnected() else "  (NT not connected yet; queued)"))
    except (KeyboardInterrupt, EOFError):
        pass
    stop.set()
    inst.stopClient()
    return 0


if __name__ == "__main__":
    sys.exit(main())
