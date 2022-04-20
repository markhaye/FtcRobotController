package org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.Robot;
import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.ServoTimed;

/*
 Autonomous Opmode that performs operations using finite state machine
 */
public class TestAutoFSM extends OpMode
{
    private enum AutoState
    {START, DRIVE2RANGE, DRIVEBACK, WAIT4ARM, WAIT4GRIP, TURNING, DONE}

    private AutoState autoState = AutoState.START;
    private ServoTimed gripper;

    @Override
    public void init()          // start setting up the hardware
    {
        Robot.init(this);

        // we will be using the grip servo, so get the object for it
        gripper = Robot.GetGripServo();
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
    public void loop()
    {
        switch (autoState)
        {
            case START:
                // start the arm and servo movements, we'll wait for them in later states
                Robot.SetArmPos(55);
                Robot.SetServoPos(gripper, Robot.SERVO_MAX_POS);

                // start the driving and wait for it in the next state
                Robot.DriveToRangeCM( 3.0, 0.3, true);
                autoState = AutoState.DRIVE2RANGE;
                break;
            case DRIVE2RANGE:
                if (Robot.IsDriveToRangeDone())
                {
                    // at the wall, so stop
                    Robot.CleanupDriveToRange();
                    // drive backwards a bit
                    Robot.DriveToPositionCM(-10.0, 0.3, true);
                    autoState = AutoState.DRIVEBACK;
                }
                break;
            case DRIVEBACK:
                if (Robot.IsDriveToPositionDone())
                {
                    Robot.CleanupDriveToPosition();
                    autoState = AutoState.WAIT4ARM;
                }
                break;
            case WAIT4ARM:
                if (Robot.IsArmDone())
                {
                    autoState = AutoState.WAIT4GRIP;
                }
                break;
            case WAIT4GRIP:
                if (Robot.IsServoDone(gripper))
                {
                    Robot.TurnAbsoluteDeg(128.0, 5.0, 0.3, true);
                    autoState = AutoState.TURNING;
                }
                break;
            case TURNING:
                if (Robot.IsTurnDone())
                {
                    Robot.CleanupTurn();
                    autoState = AutoState.DONE;
                }
                break;
            case DONE:
                break;
        }

        // always last thing in the loop (not sure it really has to be, just a convention)
        Robot.loop();
        telemetry.addData("autonomous state", autoState);
    }
}
