//Forhosv
//24804
package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx extends Gamepad {
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();
    private final Gamepad gamepad;

    private double deadzone = 0.05;

    private boolean aToggled = false;
    private boolean bToggled = false;
    private boolean xToggled = false;
    private boolean yToggled = false;

    private boolean dpadUpToggled = false;
    private boolean dpadDownToggled = false;
    private boolean dpadLeftToggled = false;
    private boolean dpadRightToggled = false;

    private boolean leftBumperToggled = false;
    private boolean rightBumperToggled = false;

    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        previous.copy(current);
        current.copy(gamepad);

        // Обновляем toggle-флаги
        if (aOnce()) aToggled = !aToggled;
        if (bOnce()) bToggled = !bToggled;
        if (xOnce()) xToggled = !xToggled;
        if (yOnce()) yToggled = !yToggled;

        if (dpadUpOnce())    dpadUpToggled = !dpadUpToggled;
        if (dpadDownOnce())  dpadDownToggled = !dpadDownToggled;
        if (dpadLeftOnce())  dpadLeftToggled = !dpadLeftToggled;
        if (dpadRightOnce()) dpadRightToggled = !dpadRightToggled;

        if (leftBumperOnce())  leftBumperToggled = !leftBumperToggled;
        if (rightBumperOnce()) rightBumperToggled = !rightBumperToggled;
    }

    // ------- Once (Rising Edge) -------
    public boolean aOnce() { return current.a && !previous.a; }
    public boolean bOnce() { return current.b && !previous.b; }
    public boolean xOnce() { return current.x && !previous.x; }
    public boolean yOnce() { return current.y && !previous.y; }

    public boolean dpadUpOnce()    { return current.dpad_up && !previous.dpad_up; }
    public boolean dpadDownOnce()  { return current.dpad_down && !previous.dpad_down; }
    public boolean dpadLeftOnce()  { return current.dpad_left && !previous.dpad_left; }
    public boolean dpadRightOnce() { return current.dpad_right && !previous.dpad_right; }

    public boolean leftBumperOnce()  { return current.left_bumper && !previous.left_bumper; }
    public boolean rightBumperOnce() { return current.right_bumper && !previous.right_bumper; }

    public boolean startOnce() { return current.start && !previous.start; }
    public boolean backOnce()  { return current.back && !previous.back; }

    public boolean rightStickOnce() {return current.right_stick_button && ! previous.right_stick_button; }
    public boolean leftStickOnce() {return current.left_stick_button && ! previous.left_stick_button; }

    // ------- Released (Falling Edge) -------
    public boolean aReleased() { return !current.a && previous.a; }
    public boolean bReleased() { return !current.b && previous.b; }
    public boolean xReleased() { return !current.x && previous.x; }
    public boolean yReleased() { return !current.y && previous.y; }

    public boolean dpadUpReleased()    { return !current.dpad_up && previous.dpad_up; }
    public boolean dpadDownReleased()  { return !current.dpad_down && previous.dpad_down; }
    public boolean dpadLeftReleased()  { return !current.dpad_left && previous.dpad_left; }
    public boolean dpadRightReleased() { return !current.dpad_right && previous.dpad_right; }

    public boolean leftBumperReleased()  { return !current.left_bumper && previous.left_bumper; }
    public boolean rightBumperReleased() { return !current.right_bumper && previous.right_bumper; }

    public boolean startReleased() { return !current.start && previous.start; }
    public boolean backReleased()  { return !current.back && previous.back; }

    // ------- Held -------
    public boolean aHeld() { return current.a; }
    public boolean bHeld() { return current.b; }
    public boolean xHeld() { return current.x; }
    public boolean yHeld() { return current.y; }

    public boolean dpadUpHeld()    { return current.dpad_up; }
    public boolean dpadDownHeld()  { return current.dpad_down; }
    public boolean dpadLeftHeld()  { return current.dpad_left; }
    public boolean dpadRightHeld() { return current.dpad_right; }

    public boolean leftBumperHeld()  { return current.left_bumper; }
    public boolean rightBumperHeld() { return current.right_bumper; }

    public boolean startHeld() { return current.start; }
    public boolean backHeld()  { return current.back; }


    // ------- Toggled -------
    public boolean aToggled() { return aToggled; }
    public boolean bToggled() { return bToggled; }
    public boolean xToggled() { return xToggled; }
    public boolean yToggled() { return yToggled; }

    public boolean dpadUpToggled()    { return dpadUpToggled; }
    public boolean dpadDownToggled()  { return dpadDownToggled; }
    public boolean dpadLeftToggled()  { return dpadLeftToggled; }
    public boolean dpadRightToggled() { return dpadRightToggled; }

    public boolean leftBumperToggled()  { return leftBumperToggled; }
    public boolean rightBumperToggled() { return rightBumperToggled; }

    // ------- Joystick Access -------
    public void setDeadzone(double threshold) {
        this.deadzone = Math.abs(threshold);
    }

    private float applyDeadzone(float value) {
        return Math.abs(value) < deadzone ? 0f : value;
    }
    public float leftStickX()  { return applyDeadzone(current.left_stick_x); }
    public float leftStickY()  { return -applyDeadzone(current.left_stick_y); }
    public float rightStickX() { return applyDeadzone(current.right_stick_x); }
    public float rightStickY() { return -applyDeadzone(current.right_stick_y); }

    public double getLeftStickMagnitude() {
        float x = leftStickX();
        float y = leftStickY();
        return Math.hypot(x, y);
    }

    public double getRightStickMagnitude() {
        float x = rightStickX();
        float y = rightStickY();
        return Math.hypot(x, y);
    }

    // ------- Trigger Access -------
    public float leftTrigger()  { return current.left_trigger; }
    public float rightTrigger() { return current.right_trigger; }

    public boolean leftTriggerPressed(double threshold) {
        return current.left_trigger > threshold;
    }

    public boolean rightTriggerPressed(double threshold) {
        return current.right_trigger > threshold;
    }

    public boolean leftTriggerOnce(double threshold) {
        return current.left_trigger > threshold && previous.left_trigger <= threshold;
    }

    public boolean rightTriggerOnce(double threshold) {
        return current.right_trigger > threshold && previous.right_trigger <= threshold;
    }
}
