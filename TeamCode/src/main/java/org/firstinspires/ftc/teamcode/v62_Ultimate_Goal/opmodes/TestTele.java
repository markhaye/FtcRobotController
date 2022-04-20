package org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.Robot;
import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.GPad;
import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.utilities.AutoPilot;

/*
 TeleOp Opmode that drives and operates the arm manually and does various autopilot operations
 */
public class TestTele extends OpMode
{

    @Override
    public void init()          // start setting up the hardware
    {
        Robot.init(this);
    }

    @Override
    public void init_loop()     // continue setting up the hardware
    {
        super.init_loop();
        Robot.init_loop();
    }

    @Override
    public void start()         // start running the opmode
    {
        super.start();
        Robot.start();
    }

    @Override
    public void loop()          // continue running the opmode
    {
        if (!AutoPilot.IsRunning())
        {
            // we're not in autopilot mode, so do whatever the user requests

            // get drive info and use it to drive the robot
            double driveScale = GPad.GetDriveScaling();
            double leftPow = driveScale * (GPad.GetDrive() + GPad.GetTurn());
            double rightPow = driveScale * (GPad.GetDrive() - GPad.GetTurn());
            Robot.SetDrivePower(leftPow, rightPow);

            // move the arm
            Robot.SetArmPower(GPad.GetArmMovement());

            // do various things, mostly for unit testing/debugging
            if (GPad.GetDpadUp())
            {
                AutoPilot.DriveCM(10.0, 0.3, true);
            }
            if (GPad.GetDpadDown())
            {
                AutoPilot.DriveCM(-10.0, 0.3, true);
            }
            if (GPad.GetDpadLeft())
            {
                AutoPilot.TurnRelativeDeg(-90.0, 5.0, 0.3, true);
            }
            if (GPad.GetDpadRight())
            {
                AutoPilot.TurnRelativeDeg(90.0, 5.0, 0.3, true);
            }

            if (GPad.GetX())
            {
                AutoPilot.DriveToRange(3.0, 0.3, true);
            }

            if (GPad.GetY())
            {
                AutoPilot.DriveAndGrab(3.0, 0.3, true);
            }

            if (GPad.GetLeftBumper())
            {
                AutoPilot.TurnAbsoluteDeg(128.0, 5.0, 0.3, true);
            }
        }
        else
        {
            if (GPad.GetCancelAutopilot())
            {
                AutoPilot.Reset();
            }
        }

        // always last thing in the loop (not sure it really has to be, just a convention)
        Robot.loop();
    }
}