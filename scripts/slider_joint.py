import tkinter as tk
from tkinter import ttk
from orca_core import OrcaHand
import argparse

class HandControlUI:
    def __init__(self, root, hand):
        self.hand = hand
        self.joint_roms = hand.joint_roms_dict
        self.joint_ids = hand.joint_ids
        self.joint_values = {joint: tk.DoubleVar() for joint in self.joint_ids}
        
        current_joint_positions = self.hand.get_joint_pos(as_list=False)
        for joint, pos in current_joint_positions.items():
            if joint in self.joint_values:
                self.joint_values[joint].set(pos)
        # Create UI elements
        self.create_ui(root)

    def create_ui(self, root):
        root.title("Orca Hand Control")
        root.geometry("400x600")

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

        for joint in self.joint_ids:
            rom_min, rom_max = self.joint_roms[joint]
            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5)

            label = ttk.Label(frame, text=joint, width=15)
            label.pack(side=tk.LEFT)

            slider = ttk.Scale(
                frame,
                from_=rom_min,
                to=rom_max,
                orient=tk.HORIZONTAL,
                variable=self.joint_values[joint],
                command=lambda value, joint=joint: self.update_joint_position(joint, value),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            value_label = ttk.Label(frame, text=f"{rom_min:.1f}", width=8)
            value_label.pack(side=tk.RIGHT)

            self.joint_values[joint].trace_add("write", lambda *args, joint=joint, label=value_label: self.update_value_label(joint, label))

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")
        current_joint_positions = self.hand.get_joint_pos(as_list=False)
        for joint, pos in current_joint_positions.items():
            if joint in self.joint_values:
                self.joint_values[joint].set(pos)

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")

    def update_joint_position(self, joint, value):
        """
        Update the position of a single joint based on the slider value.
        """
        try:
            joint_positions = {joint: float(value)}  # Only update the specific joint
            self.hand.set_joint_pos(joint_positions)
            # print(f"Updated joint {joint} to position: {value}")
        except Exception as e:
            print(f"Error updating joint {joint}: {e}")

    def update_value_label(self, joint, label):
        """
        Update the value label to display the slider value with one decimal place.
        """
        value = self.joint_values[joint].get()
        label.config(text=f"{value:.1f}")

def main():
    parser = argparse.ArgumentParser(description="Control the ORCA Hand with UI sliders.")
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    root = tk.Tk()
    HandControlUI(root, hand)
    root.mainloop()

if __name__ == "__main__":
    main()