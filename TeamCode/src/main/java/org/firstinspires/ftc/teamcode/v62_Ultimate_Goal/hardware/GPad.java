package org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// class to handle all gamepad interactions
public class GPad
{
    private static OpMode om;           // anchor for opmode

    /**
     * Constructor is private to prevent multiple instances.
     * Everything is static, so no need to instantiate at all.
     */
    private GPad()
    {
    }

    // method to initialize the gamepad
    public static void init(OpMode opMode)
    {
        om = opMode;
        // deadzone be dead
        // om.gamepad1.setJoystickDeadzone(0.05f);
    }

    // public "get" functions that are used for controlling the robot

    // gamepad1

    // boolean functions

    // left bumper
    public static boolean GetLeftBumper()
    {
        return om.gamepad1.left_bumper;
    }

    // right bumper
    public static boolean GetRightBumper()
    {
        return om.gamepad1.right_bumper;
    }

    // speed scaling
    public static double GetDriveScaling()
    {
        double scale;
        if (om.gamepad1.left_bumper && om.gamepad1.right_bumper)
            scale = 1.0;
        else if (om.gamepad1.left_bumper || om.gamepad1.right_bumper)
            scale = 0.666;
        else
            scale = 0.333;
        return scale;
    }

    // "back" button
    public static boolean GetDriveModeToggle()
    {
        return om.gamepad1.back;
    }

    // "a" button
    public static boolean GetCancelAutopilot()
    {
        return om.gamepad1.a;
    }

    // "b" button
    public static boolean GetB()
    {
        return om.gamepad1.b;
    }

    // "x" button
    public static boolean GetX()
    {
        return om.gamepad1.x;
    }

    // "y" button
    public static boolean GetY()
    {
        return om.gamepad1.y;
    }

    // left stick button
    public static boolean GetLeftStickButton()
    {
        return om.gamepad1.left_stick_button;
    }

    // right stick button
    public static boolean GetRightStickButton()
    {
        return om.gamepad1.right_stick_button;
    }

    // dpad up
    public static boolean GetDpadUp()
    {
        return om.gamepad1.dpad_up;
    }

    // dpad left
    public static boolean GetDpadLeft()
    {
        return om.gamepad1.dpad_left;
    }

    // dpad right
    public static boolean GetDpadRight()
    {
        return om.gamepad1.dpad_right;
    }

    // dpad down
    public static boolean GetDpadDown()
    {
        return om.gamepad1.dpad_down;
    }

    // floating point functions
    public static double GetDrive()
    {
        return -om.gamepad1.left_stick_y;
    }

    public static double GetTurn()
    {
        return om.gamepad1.left_stick_x;
    }

    public static double GetArmMovement()
    {
        return -om.gamepad1.right_stick_y;
    }

    public static double GetSideServo()
    {
        return om.gamepad1.left_trigger;
    }

    public static double GetGripServo()
    {
        return om.gamepad1.right_trigger;
    }

}
