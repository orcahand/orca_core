import os
import time
import yaml
import threading
import tkinter as tk
from tkinter import ttk
import argparse
from datetime import datetime
from orca_core import OrcaHand


def select_joints(joint_ids):
    """Interactive joint selection. Returns list of indices."""
    print("\n  Available joints:")
    per_row = 3
    for i in range(0, len(joint_ids), per_row):
        row = "    "
        for j in range(i, min(i + per_row, len(joint_ids))):
            row += f"[{j:2d}] {joint_ids[j]:16s}"
        print(row)

    fingers = sorted(set(j.split("_")[0] for j in joint_ids))
    print(f"\n  Shortcuts: {', '.join(fingers)}, fingers, all")

    while True:
        sel = input("  Select joints: ").strip().lower()
        if not sel or sel == "all":
            return list(range(len(joint_ids)))

        indices = set()
        for token in sel.replace(",", " ").split():
            try:
                idx = int(token)
                if 0 <= idx < len(joint_ids):
                    indices.add(idx)
                continue
            except ValueError:
                pass

            if token == "fingers":
                for i, jid in enumerate(joint_ids):
                    if not jid.startswith("wrist") and not jid.startswith("thumb"):
                        indices.add(i)
                continue

            for i, jid in enumerate(joint_ids):
                if jid.startswith(token + "_") or jid == token:
                    indices.add(i)

        if indices:
            sorted_indices = sorted(indices)
            names = [joint_ids[i] for i in sorted_indices]
            print(f"  Selected: {', '.join(names)}")
            return sorted_indices

        print("  No joints matched. Try again.")


class SliderRecorder:
    """Tkinter window with joint sliders and recording controls."""

    def __init__(self, hand, segments, segment_count_var):
        self.hand = hand
        self.segments = segments
        self.segment_count = segment_count_var
        self.joint_ids = list(hand.joint_ids)
        self.joint_roms = hand.joint_roms_dict

        self.root = tk.Toplevel() if hasattr(tk, '_default_root') and tk._default_root else tk.Tk()
        self.root.title("Record - Joint Sliders")
        self.root.geometry("550x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.joint_vars = {}
        self.closed = False

        self._build_ui()
        self._sync_from_hand()

    def _build_ui(self):
        # Sliders
        sliders_frame = ttk.Frame(self.root)
        sliders_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        for joint in self.joint_ids:
            rom_min, rom_max = self.joint_roms[joint]
            var = tk.DoubleVar(value=0.0)
            self.joint_vars[joint] = var

            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=2, padx=10)

            label = ttk.Label(frame, text=joint, width=14)
            label.pack(side=tk.LEFT)

            slider = ttk.Scale(
                frame, from_=rom_min, to=rom_max, orient=tk.HORIZONTAL,
                variable=var,
                command=lambda val, j=joint: self._on_slider(j, val),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            val_label = ttk.Label(frame, text="0.0", width=8)
            val_label.pack(side=tk.RIGHT)
            var.trace_add("write", lambda *a, j=joint, lbl=val_label: lbl.config(
                text=f"{self.joint_vars[j].get():.1f}"))

        # Buttons
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=10)

        ttk.Button(btn_frame, text="Capture Waypoint", command=self._capture).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Read Hand Position", command=self._sync_from_hand).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Done", command=self.on_close).pack(side=tk.LEFT, padx=5)

        # Status
        self.status_var = tk.StringVar(value="Adjust sliders, then capture.")
        ttk.Label(self.root, textvariable=self.status_var).pack(pady=5)

    def _sync_from_hand(self):
        """Read current hand positions into sliders."""
        current = self.hand.get_joint_pos(as_list=False)
        for joint in self.joint_ids:
            if joint in current and current[joint] is not None:
                self.joint_vars[joint].set(current[joint])

    def _on_slider(self, joint, value):
        try:
            self.hand.set_joint_pos({joint: float(value)})
        except Exception as e:
            print(f"Error updating {joint}: {e}")

    def _capture(self):
        angles = [float(self.joint_vars[j].get()) for j in self.joint_ids]
        self.segments.append({"type": "waypoint", "angles": angles})
        self.segment_count[0] += 1
        count = self.segment_count[0]
        self.status_var.set(f"Captured waypoint {count}")
        print(f"  Captured waypoint {count} (from sliders)")

    def on_close(self):
        self.closed = True
        self.root.destroy()

    def run(self):
        """Run the slider window. Blocks until closed."""
        self.hand.enable_torque()
        self._sync_from_hand()
        self.root.mainloop()
        self.hand.disable_torque()


def main():
    parser = argparse.ArgumentParser(description="Record hand movements (waypoints, continuous, and groups).")
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the orcahand model folder")
    parser.add_argument("--output_dir", type=str, default=None,
                        help="Directory to save output. Defaults to 'replay_sequences/' at project root.")
    parser.add_argument("--frequency", type=float, default=50.0,
                        help="Sampling frequency in Hz for continuous recording (default: 50.0)")
    args = parser.parse_args()

    user_prefix = input("Enter a prefix for the filename (optional, press Enter to skip): ").strip()
    if user_prefix.endswith((".yaml", ".yml")):
        user_prefix = user_prefix.rsplit(".", 1)[0]

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.enable_torque()
    hand.set_control_mode(hand.control_mode)
    hand.set_max_current(hand.max_current)
    if not hand.calibrated:
        hand.calibrate()
    hand._compute_wrap_offsets_dict()
    hand.set_neutral_position(num_steps=150, step_size=0.02)

    hand.disable_torque()
    print("Torque disabled. Ready to record.\n")

    joint_ids = list(hand.joint_ids)
    segments = []
    interval = 1.0 / args.frequency
    segment_count = [0]  # mutable so SliderRecorder can update it

    def record_continuous():
        stop_flag = []

        def wait_for_enter():
            input()
            stop_flag.append(True)

        thread = threading.Thread(target=wait_for_enter, daemon=True)
        thread.start()

        frames = []
        print("  Recording continuous... Press Enter to stop.")
        while not stop_flag:
            angles = hand.get_joint_pos(as_list=True)
            frames.append([float(a) for a in angles])
            time.sleep(interval)

        return frames

    def record_waypoint():
        angles = hand.get_joint_pos(as_list=True)
        return [float(a) for a in angles]

    def create_group():
        print("\n  ╔══ Creating group ══╗")
        motions = []
        motion_count = 0

        while True:
            print(f"\n  ── Group ({motion_count} motion(s)) ──")
            print(f"  [w] Add waypoint motion (select joints, then capture waypoints)")
            print(f"  [c] Add continuous motion (select joints, then record)")
            print(f"  [d] Done with group")
            choice = input("  > ").strip().lower()

            if choice == "d":
                break

            elif choice in ("w", "c"):
                joints = select_joints(joint_ids)

                if choice == "w":
                    waypoints = []
                    print(f"  Recording waypoints for motion {motion_count + 1}.")
                    while True:
                        cmd = input(f"  [Enter=capture wp{len(waypoints)+1}, d=done] > ").strip().lower()
                        if cmd == "d":
                            if waypoints:
                                break
                            print("  Need at least 1 waypoint.")
                        else:
                            wp = record_waypoint()
                            waypoints.append(wp)
                            print(f"  Captured waypoint {len(waypoints)}")

                    motions.append({
                        "type": "waypoint",
                        "joints": joints,
                        "angles": waypoints
                    })

                else:
                    frames = record_continuous()
                    if not frames:
                        print("  No frames captured, skipping.")
                        continue
                    motions.append({
                        "type": "continuous",
                        "joints": joints,
                        "angles": frames
                    })

                motion_count += 1
                print(f"  Motion {motion_count} added to group.")

            else:
                print("  Invalid choice.")

        if not motions:
            print("  Empty group, skipping.")
            return None

        n = len(motions)
        for i, m in enumerate(motions):
            m["phase_shift"] = round(i / n, 4)

        print(f"\n  Default phase shifts (equally spaced):")
        for i, m in enumerate(motions):
            joint_names = [joint_ids[j] for j in m["joints"]]
            print(f"    Motion {i+1} ({', '.join(joint_names[:3])}{'...' if len(joint_names) > 3 else ''}): {m['phase_shift']}")
        print(f"  You can edit phase_shift values in the YAML file later.")

        return {"type": "group", "motions": motions}

    try:
        while True:
            print("─" * 50)
            print(f"  Segments recorded: {segment_count[0]}")
            print(f"  [w] Capture waypoint (from hand position)")
            print(f"  [s] Open sliders (position joints with sliders, then capture)")
            print(f"  [c] Start continuous recording")
            print(f"  [g] Create group (phased motions for joint subsets)")
            print(f"  [Ctrl+C] Stop & save")
            print("─" * 50)
            choice = input("  > ").strip().lower()

            if choice == "w":
                angles = record_waypoint()
                segments.append({"type": "waypoint", "angles": angles})
                segment_count[0] += 1
                print(f"  Captured waypoint {segment_count[0]}")

            elif choice == "s":
                slider = SliderRecorder(hand, segments, segment_count)
                slider.run()
                print(f"  Sliders closed. Total segments: {segment_count[0]}")

            elif choice == "c":
                frames = record_continuous()
                if frames:
                    segments.append({"type": "continuous", "angles": frames})
                    segment_count[0] += 1
                    print(f"  Captured continuous segment {segment_count[0]} ({len(frames)} frames)")
                else:
                    print("  No frames captured, skipping.")

            elif choice == "g":
                group = create_group()
                if group:
                    segments.append(group)
                    segment_count[0] += 1
                    print(f"  Group saved as segment {segment_count[0]} ({len(group['motions'])} motions)")

            else:
                print("  Invalid choice. Press 'w', 's', 'c', or 'g'.")

    except KeyboardInterrupt:
        print("\n\nRecording finished.")

    if not segments:
        print("No segments recorded. Nothing to save.")
        return

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{user_prefix + '_' if user_prefix else ''}recording_{timestamp}.yaml"

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, '..'))
    output_directory = os.path.abspath(args.output_dir) if args.output_dir else os.path.join(project_root, 'replay_sequences')
    os.makedirs(output_directory, exist_ok=True)

    output_filepath = os.path.join(output_directory, filename)

    data = {
        "metadata": {
            "type": "mixed",
            "hand_type": hand.type,
            "created_at": timestamp,
            "sampling_frequency_hz": args.frequency,
            "joint_ids": joint_ids
        },
        "segments": segments
    }

    with open(output_filepath, "w") as f:
        yaml.dump(data, f)

    print(f"Recording saved to {output_filepath}")


if __name__ == "__main__":
    main()
