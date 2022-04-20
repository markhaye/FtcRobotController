package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware.HardwareGort;

public class DrivingGort extends LinearOpMode
{
    private LinearOpMode myOpMode = this;

    enum ControlMode
    {ALL_LEFT_STICK, ALL_RIGHT_STICK, TRIG_AND_LEFTX, TRIG_AND_RIGHTX, LEFTX_AND_RIGHTY, LEFTY_AND_RIGHTX}

    private double SquareInputWithSign(double input)
    {
        double result = input * input;
        if (input < 0)
            result *= -1;
        return result;
    }

    @Override
    public void runOpMode() //throws InterruptedException
    {
        double drive = 0.0, turn = 0.0, tgtPowerLeft, tgtPowerRight;
        boolean seenModeSwitchButton = false, seenSquareInputButton = false, squareInput = false;
        ControlMode controlMode = ControlMode.ALL_LEFT_STICK;
        String modeSwitchButton = "X", squareInputToggle = "Y", controlModeName = "", driveName = "", turnName = "";

        // set up our hardware, passing the opmode so we have access to all the goodies
        HardwareGort.init(myOpMode);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        // this loop needs to be fast so that the robot is responsive, i.e. no waiting
        while (opModeIsActive())
        {
            // select which control method to use
            if (gamepad1.x)
            {
                if (!seenModeSwitchButton)
                {
                    seenModeSwitchButton = true;

                    // advance to next control mode
                    switch (controlMode)
                    {
                        case ALL_LEFT_STICK:
                            controlMode = ControlMode.ALL_RIGHT_STICK;
                            break;
                        case ALL_RIGHT_STICK:
                            controlMode = ControlMode.TRIG_AND_LEFTX;
                            break;
                        case TRIG_AND_LEFTX:
                            controlMode = ControlMode.TRIG_AND_RIGHTX;
                            break;
                        case TRIG_AND_RIGHTX:
                            controlMode = ControlMode.LEFTX_AND_RIGHTY;
                            break;
                        case LEFTX_AND_RIGHTY:
                            controlMode = ControlMode.LEFTY_AND_RIGHTX;
                            break;
                        case LEFTY_AND_RIGHTX:
                        default:
                            controlMode = ControlMode.ALL_LEFT_STICK;
                            break;
                    }
                }
            }
            else
                seenModeSwitchButton = false;

            // toggle whether or not to use square input with sign

            if (gamepad1.y)
            {
                if (!seenSquareInputButton)
                {
                    seenSquareInputButton = true;
                    squareInput = !squareInput;
                }
                else
                    seenSquareInputButton = false;
            }

            // read controller based on control mode
            switch (controlMode)
            {
                case ALL_LEFT_STICK:
                {
                    drive = gamepad1.left_stick_y;
                    turn = gamepad1.left_stick_x;
                    controlModeName = "All left stick";
                    driveName = "Left stick Y";
                    turnName = "Left stick X";
                    break;
                }
                case ALL_RIGHT_STICK:
                {
                    drive = gamepad1.right_stick_y;
                    turn = gamepad1.right_stick_x;
                    controlModeName = "All right stick";
                    driveName = "Right stick Y";
                    turnName = "Right stick X";
                    break;
                }
                case TRIG_AND_LEFTX:
                {
                    drive = gamepad1.left_trigger - gamepad1.right_trigger;
                    turn = gamepad1.left_stick_x;
                    controlModeName = "Trigger and left X";
                    driveName = "Left/Right trigger";
                    turnName = "Left stick X";
                    break;
                }
                case TRIG_AND_RIGHTX:
                {
                    drive = gamepad1.left_trigger - gamepad1.right_trigger;
                    turn = gamepad1.right_stick_x;
                    controlModeName = "Trigger and right X";
                    driveName = "Left/Right trigger";
                    turnName = "Right stick X";
                    break;
                }
                case LEFTX_AND_RIGHTY:
                {
                    drive = gamepad1.right_stick_y;
                    turn = gamepad1.left_stick_x;
                    controlModeName = "Left X and right Y";
                    driveName = "Right stick Y";
                    turnName = "Left stick X";
                    break;
                }
                case LEFTY_AND_RIGHTX:
                {
                    drive = gamepad1.left_stick_y;
                    turn = gamepad1.right_stick_x;
                    controlModeName = "Left Y and right X";
                    driveName = "Right stick X";
                    turnName = "Left stick Y";
                    break;
                }
                default:
                    break;
            }

            // if using the square input method, do it
            if (squareInput)
            {
                drive = SquareInputWithSign(drive);
                turn = SquareInputWithSign(turn);
            }
            tgtPowerLeft = turn;
            tgtPowerRight = -turn;
            tgtPowerLeft -= drive;
            tgtPowerRight -= drive;

            HardwareGort.SetDrivePowerGort(Range.clip(tgtPowerLeft, -1.0, 1.0), Range.clip(tgtPowerRight, -1.0, 1.0));

            telemetry.addData("Mode", controlModeName);
            telemetry.addData("Fwd/Rev", driveName);
            telemetry.addData("Right/Left", turnName);
            telemetry.addData("Mode switch button", modeSwitchButton);
            telemetry.addData("Squaring inputs", squareInput);
            telemetry.addData("Square input toggle", squareInputToggle);
            telemetry.update();
        }
    }
}
