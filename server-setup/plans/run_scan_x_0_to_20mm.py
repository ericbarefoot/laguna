"""
Run a line scan on X: current position ("0") to current + 20 mm, at 1 mm/s.

This is Step 8 from server-setup/plans/mqtt_od2000_resume.ipynb, as a
standalone script. NOT executed automatically — run this yourself:

    python3 server-setup/plans/run_scan_x_0_to_20mm.py

What it does:
  1. Connects to the gantry agent (pi_agent transport, safe_mode=False —
     required since scanning is gated the same as any motion command)
  2. Reads the current X position and uses it as the "0" reference —
     the actual absolute target sent to the agent is current + 20mm
  3. Runs the scan via TopographicProfiler.scan() ON A BACKGROUND THREAD,
     while the main thread waits for you to type "stop" + Enter. This is
     NOT "open a second terminal" — a second terminal would launch a
     second gantry_agent.py process that fails to open the already-held
     serial port, and wouldn't have access to this script's `profiler`
     object anyway. Instead, the SAME connection stays responsive via its
     background reader thread (see pi_bridge.py), so calling
     profiler.stop() from this process's main thread while the scan
     thread is blocked in scan() works correctly — this is exactly the
     mechanism the STOP path was built for.
     Meanwhile, the agent handles everything Pi-side: SPD/BMT/MIF on X,
     turning the OD2000 laser on before the move and off after (automatic
     — nothing to do here), and polling pdin/getdata on a background
     thread as fast as the persistent HTTP connection allows (~380 Hz
     confirmed on hardware 2026-07-28 — already "as frequently as
     possible", no rate parameter to set).
  4. Retrieves the resulting CSV + prints a summary (whether the scan
     finished normally or was stopped early — both end at the same
     scan_done completion path, just with fewer samples if stopped)
  5. Plots distance vs. position, separating the slew phase (used for
     the actual profile) from the accel/decel ramp phase (kept in the
     CSV but excluded from the plot's main trace)

Expect ~20s for the move itself (20mm at 1mm/s), plus a couple seconds of
SSH/agent overhead on either side, if you don't stop it early.
"""

import queue
import sys
import threading

sys.path.insert(0, "/home/eric/mysoftware/laguna/src")

from laguna.robot.macron.pi_bridge import PiGantryConnection
from laguna.robot.macron.controller import GantryController
from laguna.robot.macron.profiler import TopographicProfiler

# --- Settings — adjust if your setup differs ---
PI_HOST = "red.lab"
PI_USER = "oak"
PI_KEY = "/home/eric/.ssh/id_ed25519"
REMOTE_SERIAL_DEVICE = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9L0C-if00-port0"
AL1342_HOST = "192.168.1.251"
PDIN_PORT = 2  # OD2000 confirmed on this IO-Link port, 2026-07-28

SCAN_AXIS = "A1"        # X
SCAN_DISTANCE_MM = 20.0  # relative distance from wherever X currently is
SCAN_RATE_MM_S = 1.0

OUTPUT_DIR = "/home/eric/mysoftware/laguna/server-setup/plans/scan_output"


def main():
    conn = PiGantryConnection(
        host=PI_HOST,
        ssh_user=PI_USER,
        ssh_key=PI_KEY,
        remote_serial_device=REMOTE_SERIAL_DEVICE,
        remote_baud=9600,
        safe_mode=False,  # required: scan_start is gated like any motion command
    )

    print(f"Connecting to gantry agent on {PI_HOST}...")
    conn.connect()

    try:
        start_x = float(conn.send(f"{SCAN_AXIS} ACP"))
        target_x = start_x + SCAN_DISTANCE_MM
        print(f"Current X position: {start_x:.3f} mm")
        print(f"Scan target (X + {SCAN_DISTANCE_MM} mm): {target_x:.3f} mm")

        gantry = GantryController(connection=conn)

        profiler = TopographicProfiler(
            gantry=gantry,
            pi_host=PI_HOST,
            pi_user=PI_USER,
            pi_key=PI_KEY,
            pdin_port=PDIN_PORT,
            al1342_host=AL1342_HOST,
            output_dir=OUTPUT_DIR,
        )

        print()
        print(f"Starting scan: {SCAN_AXIS} -> {target_x:.3f} mm at {SCAN_RATE_MM_S} mm/s "
              f"(~{SCAN_DISTANCE_MM / SCAN_RATE_MM_S:.0f}s)...")
        print("(Laser turns on automatically for the scan and off again when it's done.)")
        print()
        print(">>> Type 'stop' and press Enter at any time to cancel the scan early. <<<")
        print(">>> Otherwise this just waits for the scan to finish on its own.       <<<")
        print()

        scan_result = {}

        def run_scan():
            try:
                scan_result["value"] = profiler.scan(
                    axis=SCAN_AXIS, end_mm=target_x, feed_rate_mm_s=SCAN_RATE_MM_S
                )
            except Exception as exc:
                scan_result["error"] = exc

        scan_thread = threading.Thread(target=run_scan, daemon=True)
        scan_thread.start()

        # A dedicated stdin-reading thread, since input() can't be interrupted
        # from another thread — this lets the main loop poll both "did the
        # scan finish on its own?" and "did the user type stop?" without
        # ever blocking indefinitely on either one.
        stdin_q: "queue.Queue[str]" = queue.Queue()

        def read_stdin():
            while True:
                try:
                    line = input()
                except EOFError:
                    break
                stdin_q.put(line)

        input_thread = threading.Thread(target=read_stdin, daemon=True)
        input_thread.start()

        stop_sent = False
        while scan_thread.is_alive():
            try:
                line = stdin_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if line.strip().lower() == "stop" and not stop_sent:
                print("Stop requested — sending stop_scan()...")
                profiler.stop()
                stop_sent = True

        scan_thread.join()

        if "error" in scan_result:
            raise scan_result["error"]
        result = scan_result["value"]

        print()
        print("=== Scan complete ===")
        print(f"Samples collected : {result.metadata['samples']}")
        print(f"Achieved rate     : {result.metadata.get('achieved_rate_hz', 0):.1f} Hz")
        print(f"Start position    : {result.metadata['actual_start_mm']:.3f} mm")
        print(f"End position      : {result.metadata['actual_end_mm']:.3f} mm")
        print(f"Actual distance   : {result.metadata['actual_distance_mm']:.3f} mm")
        print(f"CSV path          : {result.path}")

        try:
            import matplotlib.pyplot as plt

            df = result.df
            slew = df[df["in_ramp"] == 0]
            ramp = df[df["in_ramp"] == 1]

            fig, ax = plt.subplots(figsize=(12, 4))
            ax.plot(slew["pos_mm"], slew["distance_mm"], linewidth=0.8, label="slew phase")
            ax.plot(ramp["pos_mm"], ramp["distance_mm"], ".", alpha=0.3, markersize=3,
                    label="ramp phase (excluded)")
            ax.set_xlabel("Gantry X position (mm)")
            ax.set_ylabel("OD2000 distance (mm)")
            ax.set_title(f"Line scan — X axis, {start_x:.1f} to {target_x:.1f} mm")
            ax.legend()
            plt.tight_layout()
            plot_path = str(result.path).replace(".csv", ".png")
            plt.savefig(plot_path)
            print(f"Plot saved        : {plot_path}")
            plt.show()
        except ImportError:
            print("(matplotlib not installed — skipping plot; CSV is still available above)")

    finally:
        conn.disconnect()
        print()
        print("Disconnected.")


if __name__ == "__main__":
    main()
