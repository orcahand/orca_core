import tkinter as tk
from tkinter import ttk
import argparse
from orca_core import OrcaHand

class HandControlUI:
    def __init__(self, root, hand):
        self.hand = hand
        self.joint_values = {joint: tk.DoubleVar() for joint in hand.joint_ids}
        self.create_ui(root)

    def create_ui(self, root):
        root.title("Orca Hand Joint Control")
        root.geometry("400x800")

        # Torque control buttons
        torque_frame = ttk.Frame(root)
        torque_frame.pack(pady=10)

        enable_button = ttk.Button(torque_frame, text="Enable Torque", command=self.enable_torque)
        enable_button.pack(side=tk.LEFT, padx=5)

        disable_button = ttk.Button(torque_frame, text="Disable Torque", command=self.disable_torque)
        disable_button.pack(side=tk.LEFT, padx=5)

        # Sliders for each joint
        sliders_frame = ttk.Frame(root)
        sliders_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # Get current joint positions for initialization
        current_joint_pos = self.hand.get_joint_pos(as_list=False)
        for joint in self.hand.joint_ids:
            self.joint_values[joint].set(current_joint_pos[joint])

            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5)

            # Get ROM limits for the joint
            rom_min, rom_max = self.hand.joint_roms[joint]
            
            label = ttk.Label(frame, text=f"{joint}", width=15)
            label.pack(side=tk.LEFT)

            slider = ttk.Scale(
                frame,
                from_=rom_min,
                to=rom_max,
                orient=tk.HORIZONTAL,
                variable=self.joint_values[joint],
                command=lambda value, j=joint: self.update_joint_position(j, value),
                length=200
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            value_label = ttk.Label(frame, text=f"{current_joint_pos[joint]:.2f}", width=8)
            value_label.pack(side=tk.RIGHT)

            self.joint_values[joint].trace_add("write", lambda *args, j=joint, label=value_label: self.update_value_label(j, label))

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")

    def update_joint_position(self, joint, value):
        try:
            # Get all current joint values and send as a dictionary
            joint_positions = {j: v.get() for j, v in self.joint_values.items()}
            self.hand.set_joint_pos(joint_positions)
            print(f"Updated joint {joint} to position: {value}")
        except Exception as e:
            print(f"Error updating joint {joint}: {e}")

    def update_value_label(self, joint, label):
        value = self.joint_values[joint].get()
        label.config(text=f"{value:.2f}")

def main():
    parser = argparse.ArgumentParser(description='Orca Hand Joint Control')
    parser.add_argument('hand_path', type=str, help='Path to the hand model directory')
    args = parser.parse_args()

    hand = OrcaHand(args.hand_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    root = tk.Tk()
    app = HandControlUI(root, hand)
    root.mainloop()

if __name__ == "__main__":
    main()