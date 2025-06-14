import tkinter as tk
from tkinter import ttk
import argparse
from orca_core import OrcaHand

class MotorControlUI:
    def __init__(self, root, hand):
        self.hand = hand
        self.motor_values = {motor: tk.DoubleVar() for motor in hand.motor_ids}
        self.create_ui(root)

    def create_ui(self, root):
        root.title("Orca Hand Motor Control")
        root.geometry("400x800")

        # Torque control buttons
        torque_frame = ttk.Frame(root)
        torque_frame.pack(pady=10)

        enable_button = ttk.Button(torque_frame, text="Enable Torque", command=self.enable_torque)
        enable_button.pack(side=tk.LEFT, padx=5)

        disable_button = ttk.Button(torque_frame, text="Disable Torque", command=self.disable_torque)
        disable_button.pack(side=tk.LEFT, padx=5)

        # Sliders for each motor
        sliders_frame = ttk.Frame(root)
        sliders_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # Get current motor positions for initialization
        current_motor_pos = self.hand.get_motor_pos(as_dict=True)
        for motor in self.hand.motor_ids:
            self.motor_values[motor].set(current_motor_pos[motor])

            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5)

            # Define slider range to be very small for precise control
            rom_min = current_motor_pos[motor] - 1.0  # 1.0 encoder units below current position
            rom_max = current_motor_pos[motor] + 1.0  # 1.0 encoder units above current position
            
            label = ttk.Label(frame, text=f"Motor {motor}", width=15)
            label.pack(side=tk.LEFT)

            slider = tk.Scale(
                frame,
                from_=rom_min,
                to=rom_max,
                orient=tk.HORIZONTAL,
                variable=self.motor_values[motor],
                command=lambda value, m=motor: self.update_motor_position(m, value),
                length=200,
                resolution=0.1,  # Set resolution to 0.1 for finer control
                showvalue=False  # Hide the default value display since we have our own
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            value_label = ttk.Label(frame, text=f"{current_motor_pos[motor]:.1f}", width=8)
            value_label.pack(side=tk.RIGHT)

            self.motor_values[motor].trace_add("write", lambda *args, m=motor, label=value_label: self.update_value_label(m, label))

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")

    def update_motor_position(self, motor, value):
        try:
            # Get all current motor values and send as a dictionary
            motor_positions = {m: v.get() for m, v in self.motor_values.items()}
            self.hand._set_motor_pos(motor_positions)
            print(f"Updated motor {motor} to position: {value:.1f}")
        except Exception as e:
            print(f"Error updating motor {motor}: {e}")

    def update_value_label(self, motor, label):
        value = self.motor_values[motor].get()
        label.config(text=f"{value:.1f}")

def main():
    parser = argparse.ArgumentParser(description='Orca Hand Motor Control')
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    root = tk.Tk()
    app = MotorControlUI(root, hand)
    root.mainloop()

if __name__ == "__main__":
    main()