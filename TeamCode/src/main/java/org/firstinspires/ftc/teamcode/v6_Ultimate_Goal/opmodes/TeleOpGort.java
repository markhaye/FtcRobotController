package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware.HardwareGort;
import org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware.KernelGort;

public class TeleOpGort extends LinearOpMode
{
    private static final double DRIVE_SCALING = 0.5;

    private LinearOpMode myOpMode = this;

    @Override
    public void runOpMode()
    {
        double absIMUheadingRad;
        double leftStickX, leftStickY, drive, turn, tgtPowerLeft, tgtPowerRight;
        double tgtPowerArm, tgtPosServoGrip, tgtPosServoSide;
        long lastOpTime = 0;
        int iterations = 1;
        boolean fwdProgressBlocked = false, seenGamepadBack = false, cardinalDriveMode = false,
                autoPilot = false;
        KernelGort.ArmGrabState grabState = KernelGort.ArmGrabState.IDLE;

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
            // toggle current drive mode
            if (gamepad1.back)
            {
                if (!seenGamepadBack)
                {
                    seenGamepadBack = true;
                    cardinalDriveMode = !cardinalDriveMode;
                }
            }
            else
                seenGamepadBack = false;

            // manage the autopilot
            if (autoPilot)
            {
                // if we encountered something, stop driving and grab it
                if (HardwareGort.sensorRange.getDistance(DistanceUnit.CM) <= 3.0)
                {
                    HardwareGort.SetDrivePowerGort(0.0, 0.0 );
                    if (grabState == KernelGort.ArmGrabState.IDLE)
                        grabState = KernelGort.ArmGrabState.START;
                }

                // manage the grabber
                grabState = KernelGort.ArmGrab(grabState);

                // if done or cancelling autopilot, turn it off and switch arm grabber to idle state
                // (doesn't matter where the arm and servo are)
                if (grabState == KernelGort.ArmGrabState.DONE || gamepad1.a)
                {
                    autoPilot = false;
                    grabState = KernelGort.ArmGrabState.IDLE;
                }
            }
            // otherwise, do whatever the driver says
            else
            {
                // move the drive wheels based on left joystick
                leftStickX = gamepad1.left_stick_x;
                leftStickY = gamepad1.left_stick_y;

                if (cardinalDriveMode)
                {
                    absIMUheadingRad = Math.toRadians(KernelGort.getAbsoluteIMUHeading());
                    drive = leftStickY * Math.cos(absIMUheadingRad) + leftStickX * Math.sin(absIMUheadingRad);
                    turn = leftStickX * Math.cos(absIMUheadingRad) + leftStickY * Math.sin(absIMUheadingRad);
                }
                else
                {
                    drive = leftStickY;
                    turn = leftStickX;
                }

                // prevent forward motion when range sensor detects wall
                if (HardwareGort.sensorRange.getDistance(DistanceUnit.CM) < 3.0)
                {
                    // stick forward is negative, so make sure it's >= 0
                    drive = Math.max(drive, 0.0);
                    fwdProgressBlocked = true;
                }
                else
                    fwdProgressBlocked = false;
                tgtPowerLeft = DRIVE_SCALING * turn;
                tgtPowerRight = -DRIVE_SCALING * turn;
                tgtPowerLeft -= DRIVE_SCALING * drive;
                tgtPowerRight -= DRIVE_SCALING * drive;

                HardwareGort.SetDrivePowerGort(Range.clip(tgtPowerLeft, -1.0, 1.0), Range.clip(tgtPowerRight, -1.0, 1.0));

                // move the arm motor based on right joystick
                // need to limit movement based on encoder
                tgtPowerArm = HardwareGort.ARM_POWER * gamepad1.right_stick_y;
                if (HardwareGort.motorArm.getCurrentPosition() > HardwareGort.ARM_MAX_POS)
                    tgtPowerArm = Math.min(tgtPowerArm, 0.0);
                if (HardwareGort.motorArm.getCurrentPosition() < HardwareGort.ARM_MIN_POS)
                    tgtPowerArm = Math.max(tgtPowerArm, 0.0);
                HardwareGort.motorArm.setPower(tgtPowerArm);

                // move the side servo based on left trigger (0.0 released - 1.0 pressed)
                tgtPosServoSide = Range.clip(gamepad1.left_trigger, 0.0, 1.0);
                HardwareGort.servoSide.setPosition(tgtPosServoSide);

                // move the grip servo based on right trigger (0.0 released - 1.0 pressed)
                tgtPosServoGrip = 1.0 - Range.clip(gamepad1.right_trigger, 0.0, 1.0);
                HardwareGort.servoGrip.setPosition(tgtPosServoGrip);
            }

            // when the left bumper is pressed, do a debugging thing
            if (gamepad1.left_bumper)
            {
                double degrees = 90.0, power = 0.5;
                if (gamepad1.dpad_down)
                    power = 0.2;
                else if (gamepad1.dpad_up)
                    power = 0.9;
                if (gamepad1.dpad_left)
                    degrees = 45.0;
                else if (gamepad1.dpad_right)
                    degrees = 135.0;
                lastOpTime = System.nanoTime();
                KernelGort.TurnGyroBasic(-degrees, power);
                lastOpTime = System.nanoTime() - lastOpTime;
            }

            // when the right bumper is pressed, do a different debugging thing
            if (gamepad1.right_bumper)
            {
                double degrees = 90.0, power = 0.5;
                if (gamepad1.dpad_down)
                    power = 0.2;
                else if (gamepad1.dpad_up)
                    power = 0.9;
                if (gamepad1.dpad_left)
                    degrees = 45.0;
                else if (gamepad1.dpad_right)
                    degrees = 135.0;
                lastOpTime = System.nanoTime();
                KernelGort.TurnGyroBasic(degrees, power);
                lastOpTime = System.nanoTime() - lastOpTime;
            }

            // when the x button is pressed, do a debugging thing
            if (gamepad1.x)
            {
                double degrees = 90.0, power = 0.5;
                if (gamepad1.dpad_down)
                    power = 0.2;
                else if (gamepad1.dpad_up)
                    power = 0.9;
                if (gamepad1.dpad_left)
                    degrees = 45.0;
                else if (gamepad1.dpad_right)
                    degrees = 135.0;
                lastOpTime = System.nanoTime();
                iterations = KernelGort.TurnGyroPID(-degrees, power);
                lastOpTime = System.nanoTime() - lastOpTime;
            }

            // when the b button is pressed, do a different debugging thing
            if (gamepad1.b)
            {
                double degrees = 90.0, power = 0.5;
                if (gamepad1.dpad_down)
                    power = 0.2;
                else if (gamepad1.dpad_up)
                    power = 0.9;
                if (gamepad1.dpad_left)
                    degrees = 45.0;
                else if (gamepad1.dpad_right)
                    degrees = 135.0;
                lastOpTime = System.nanoTime();
                iterations = KernelGort.TurnGyroPID(degrees, power);
                lastOpTime = System.nanoTime() - lastOpTime;
            }

            // when the y button is pressed, do a different debugging thing
            if (gamepad1.y)
            {
                double centimeters = 10.0, power = 0.5;
                if (gamepad1.dpad_down)
                    power = 0.2;
                else if (gamepad1.dpad_up)
                    power = 0.9;
                if (gamepad1.dpad_left)
                    centimeters = -10.0;
                else if (gamepad1.dpad_right)
                    centimeters = 15.0;
                lastOpTime = System.nanoTime();
                KernelGort.DriveCm(centimeters, power);
                lastOpTime = System.nanoTime() - lastOpTime;
            }

            // when the left stick (driving) button is pressed, re-orient the robot
            if (gamepad1.left_stick_button)
            {
                lastOpTime = System.nanoTime();
                // uses TurnGyroPID ... might not be fast, but gets pretty accurate
                KernelGort.ReOrient();
                lastOpTime = System.nanoTime() - lastOpTime;
            }

            // when the left stick button is pressed, use autopilot to grab whatever's in front of the robot ...
            if (gamepad1.right_stick_button && !autoPilot)
            {
                autoPilot = true;

                // drive straight forward slowly ...
                HardwareGort.SetDrivePowerGort(KernelGort.DRIVE_POWER / 2, KernelGort.DRIVE_POWER / 2);
            }

            telemetry.addData("Status", "Running " + (cardinalDriveMode ? "cardinal" : "normal")
                    + ((autoPilot ? " (auto pilot)" : (fwdProgressBlocked ? " (fwd progress blocked)" : " ..."))));
            telemetry.addData("abs heading", "compass %5.1f, imu %5.1f",
                    KernelGort.getAbsoluteCompassHeading(), KernelGort.getAbsoluteIMUHeading());
            telemetry.addData("rel heading", "compass %5.1f", KernelGort.getRelativeCompassHeading());
            telemetry.addData("Distance (cm)", "%.3f, color (rgb) %d %d %d",
                    HardwareGort.sensorRange.getDistance(DistanceUnit.CM), HardwareGort.sensorColor.red(),
                    HardwareGort.sensorColor.green(), HardwareGort.sensorColor.blue());
            //HardwareGort.sensorColor.alpha
            //telemetry.addData("touch is", HardwareGort.sensorTouch.getState());
            telemetry.addData("Last Op Time", "%5.2fs %5.2fms/iteration",
                    (lastOpTime / 1.0E9), (lastOpTime / iterations / 1.0E6));
            telemetry.addData("grip servo pos", HardwareGort.servoGrip.getPosition());
            String grabStateString;
            switch (grabState)
            {
                case IDLE:
                    grabStateString = "IDLE";
                    break;
                case START:
                    grabStateString = "START";
                    break;
                case OPENING:
                    grabStateString = "OPENING";
                    break;
                case LOWERING:
                    grabStateString = "LOWERING";
                    break;
                case CLOSING:
                    grabStateString = "CLOSING";
                    break;
                case RAISING:
                    grabStateString = "RAISING";
                    break;
                case DONE:
                    grabStateString = "DONE";
                    break;
                default:
                    grabStateString = "DEFAULT";
                    break;
            }
            telemetry.addData("grabState", grabStateString);
            telemetry.update();
        }
    }
}
