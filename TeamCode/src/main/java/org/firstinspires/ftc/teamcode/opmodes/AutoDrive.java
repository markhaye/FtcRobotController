package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/*
Autonomous opmode for driving the team robot
 */
//@Autonomous
public class AutoDrive extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // first, initialize the robot utility class and wait for start button
        Robot.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // drive forward a bit
        if (opModeIsActive())
        {
            Robot.DriveToPositionCM(20, 0.5);
        }
        while (opModeIsActive() && !Robot.IsDriveToPositionDone())
        {
            DoTelemetry();
        }
        Robot.CleanupDriveToPosition();

        // drive backward a bit
        if (opModeIsActive())
        {
            Robot.DriveToPositionCM(-10, 0.5);
        }
        while (opModeIsActive() && !Robot.IsDriveToPositionDone())
        {
            DoTelemetry();
        }
        Robot.CleanupDriveToPosition();
    }

    private void DoTelemetry()
    {
        telemetry.addData("Status", "Running");
        telemetry.addData("Heading", "%3.1f (%s)",
                Robot.GetAbsoluteCompassHeading(), Robot.GetCompassDir());
        telemetry.update();
    }
}
