package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.TeamRobot;

/*
Autonomous opmode for driving the team robot
 */
//@Autonomous
public class TeamAutoDrive extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // first, initialize the robot utility class and wait for start button
        TeamRobot.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // drive forward a bit
        if (opModeIsActive())
        {
            TeamRobot.DriveToPositionCM(20, 0.5);
        }
        while (opModeIsActive() && !TeamRobot.IsDriveToPositionDone())
        {
            DoTelemetry();
        }
        TeamRobot.CleanupDriveToPosition();

        // drive backward a bit
        if (opModeIsActive())
        {
            TeamRobot.DriveToPositionCM(-10, 0.5);
        }
        while (opModeIsActive() && !TeamRobot.IsDriveToPositionDone())
        {
            DoTelemetry();
        }
        TeamRobot.CleanupDriveToPosition();
    }

    private void DoTelemetry()
    {
        telemetry.addData("Status", "Running");
        telemetry.addData("Heading", "%3.1f (%s)",
                TeamRobot.GetAbsoluteCompassHeading(), TeamRobot.GetCompassDir());
        telemetry.update();
    }
}
