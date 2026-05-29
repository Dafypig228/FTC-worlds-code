package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadDeadZoned extends Gamepad {
    public double deadzone = 0.05;

    public double get_left_stick_x() {
        return this.left_stick_x * deadzone;
    }

    public double get_left_stick_y() {
        return this.left_stick_x * deadzone;
    }

    public double get_right_stick_x() {
        return this.right_stick_x * deadzone;
    }

    public double get_right_stick_y() {
        return this.right_stick_x * deadzone;
    }
}
