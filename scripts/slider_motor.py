import tkinter as tk
from tkinter import ttk
from orca_core import OrcaHand

class HandControlUI:
    def __init__(self, root, hand):
        self.hand = hand
        self.motor_count = hand.motor_count if hasattr(hand, "motor_count") else len(hand.get_motor_pos())
        self.motor_values = [tk.DoubleVar() for _ in range(self.motor_count)]

        self.create_ui(root)

    def create_ui(self, root):
        root.title("Orca Hand Motor Control")
        root.geometry("400x600")

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
        current_motor_pos = self.hand.get_motor_pos()
        for i in range(self.motor_count):
            self.motor_values[i].set(current_motor_pos[i])

            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5)

            label = ttk.Label(frame, text=f"Motor {i+1}", width=15)
            label.pack(side=tk.LEFT)

            slider = ttk.Scale(
                frame,
                from_=-10,  # Arbitrary range, adjust as needed
                to=10,
                orient=tk.HORIZONTAL,
                variable=self.motor_values[i],
                command=lambda value, idx=i: self.update_motor_position(idx, value),
                length=200
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            value_label = ttk.Label(frame, text=f"{current_motor_pos[i]:.2f}", width=8)
            value_label.pack(side=tk.RIGHT)

            self.motor_values[i].trace_add("write", lambda *args, idx=i, label=value_label: self.update_value_label(idx, label))

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")

    def update_motor_position(self, idx, value):
        try:
            # Get all current slider values and send as a list
            motor_positions = [v.get() for v in self.motor_values]
            self.hand._set_motor_pos(motor_positions)
            print(f"Updated motor {idx+1} to position: {value}")
        except Exception as e:
            print(f"Error updating motor {idx+1}: {e}")

    def update_value_label(self, idx, label):
        value = self.motor_values[idx].get()
        label.config(text=f"{value:.2f}")

def main():
    hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_left')
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