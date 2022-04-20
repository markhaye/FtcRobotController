package org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.Robot;
import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.ServoTimed;

/*
 Autonomous LinearOpmode that performs operations using while loops (waiting)
 */
public class TestAutoLOM extends LinearOpMode
{
    private static ServoTimed gripper;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // start the init
        Robot.init(this);
        // continue the init
        while (!isStarted() && !isStopRequested())
        {
            Robot.init_loop();
            telemetry.update();
        }
        // don't need waitForStart because the above loop accomplishes the same thing

        // if we're not stopping, we must be starting
        if (!isStopRequested())
        {
            Robot.start();
            gripper = Robot.GetGripServo();
        }

        // do operations sequentially

        // start to raise arm and open gripper while simultaneously driving towards wall
        if (opModeIsActive())
        {
            // start the arm and servo movements, we'll wait for them later
            Robot.SetArmPos(55);
            Robot.SetServoPos(gripper, Robot.SERVO_MAX_POS);

            // start the driving and wait for it now
            Robot.DriveToRangeCM( 3.0, 0.3, true);
            while (!Robot.IsDriveToRangeDone() && opModeIsActive())
            {
                Robot.loop();
                telemetry.update();
            }
            Robot.SetDrivePower(0.0, 0.0);
        }

        // drive backwards a bit
        if (opModeIsActive())
        {
            Robot.DriveToPositionCM(-10.0, 0.3, true);
            while (!Robot.IsDriveToPositionDone() && opModeIsActive())
            {
                Robot.loop();
                telemetry.update();
            }
            Robot.CleanupDriveToPosition();
        }

        // wait for arm raise to be done (might already be done)
        while (!Robot.IsArmDone() && opModeIsActive())
        {
            Robot.loop();
            telemetry.update();
        }

        // wait for gripper to be open (might already be done)
        while (!Robot.IsServoDone(gripper) && opModeIsActive())
        {
            Robot.loop();
            telemetry.update();
        }

        // turn a bit
        if (opModeIsActive())
        {
            Robot.TurnAbsoluteDeg(128.0, 5.0,0.3, true);
            while (!Robot.IsTurnDone() && opModeIsActive())
            {
                Robot.loop();
                telemetry.update();
            }
            Robot.CleanupTurn();
        }

    }
}
