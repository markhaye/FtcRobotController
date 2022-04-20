package org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.utilities;

// Class to perform autopilot activities on the robot, like driving, turning, grabbing, etc.
// Uses finite state machine (no waiting/sleeping)

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.Robot;
import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware.ServoTimed;

public class AutoPilot
{
    private static OpMode om = null;
    private static double targetDistCM, targetPower, targetDeg, targetTolDeg;
    private static int targetArmPos, targetWaitMS;
    private static boolean targetBrake;

    private static ServoTimed gripper = Robot.GetGripServo();

    // these are the functions that autopilot can perform
    private enum AutoPilotFunc
    {DRIVE, DRIVE_TO_RANGE, SET_ARM_POS, DRIVE_AND_GRAB, TURNABS, TURNREL, WAIT, NONE}

    private static AutoPilotFunc autoPilotFunc = AutoPilotFunc.NONE;

    private enum AutoPilotState
    {IDLE, START, RUNNING, CLEANUP, DONE}

    private static AutoPilotState autoPilotState = AutoPilotState.IDLE;

    private enum DriveAndGrabState
    {IDLE, START, ARM_LOWERING_AND_GRIP_OPENING, ARM_LOWERING, GRIP_OPENING, DRIVING, GRIP_CLOSING, ARM_RAISING, RESET, DONE}

    private static DriveAndGrabState driveAndGrabState = DriveAndGrabState.IDLE;

    /**
     * Constructor is private to prevent multiple instances.
     * Everything is static, so no need to instantiate at all.
     */
    private AutoPilot()
    {
        AutoPilot.Reset();
    }


    /**
     * initialize the class
     *
     * @param opMode the current opmode object
     */
    public static void init(OpMode opMode)
    {
        om = opMode;
        AutoPilot.Reset();
    }

    /**
     * Report the important state variables for this class.
     *
     * @param opModeState which state the opmode is in (init/run)
     */
    public static void DoTelemetry(Robot.OpModeState opModeState)
    {
        om.telemetry.addData("Auto", "func=%s, state=%s",
                autoPilotFunc, autoPilotState);
        om.telemetry.addData("Drive and Grab", driveAndGrabState);
    }

    // following are a few methods to drive the robot straight

    /**
     * Does common setup for driving.
     *
     * @param distanceCM the desired distance to drive or range
     * @param power      the desired motor power magnitude (min 0.0, max 1.0)
     * @param brake      whether or not to brake the motors when done
     */
    private static void DriveSetupCM(double distanceCM, double power, boolean brake)
    {
        autoPilotState = AutoPilotState.START;
        targetDistCM = distanceCM;
        targetPower = Range.clip(Math.abs(power), 0.0, 1.0);
        targetBrake = brake;
    }

    /**
     * Start driving the robot a given distance.
     * Can be cancelled with AutoPilot.Reset
     *
     * @param distanceCM the desired distance to travel
     *                   can be positive (forward) or negative (backward)
     * @param power      the desired motor power magnitude (min 0.0, max 1.0)
     * @param brake      whether or not to brake the motors when done
     */
    public static void DriveCM(double distanceCM, double power, boolean brake)
    {
        autoPilotFunc = AutoPilotFunc.DRIVE;
        DriveSetupCM(distanceCM, power, brake);
    }

    /**
     * Start driving the robot until the distance sensor detects an obstacle.
     * Can be cancelled with AutoPilot.Reset
     *
     * @param distanceCM the desired maximum distance to obstacle before stopping
     *                   always treated as positive (sign is ignored)
     * @param power      the desired motor power magnitude (min 0.0, max 1.0)
     *                   always positive since range sensor is on the front
     * @param brake      whether or not to brake the motors when done
     */
    public static void DriveToRange(double distanceCM, double power, boolean brake)
    {
        autoPilotFunc = AutoPilotFunc.DRIVE_TO_RANGE;
        DriveSetupCM(distanceCM, power, brake);
    }

    /**
     * Start driving the robot until the distance sensor detects an obstacle,
     * then close gripper, then raise arm, then done.
     * Can be cancelled with AutoPilot.Reset
     *
     * @param distanceCM the desired maximum distance to obstacle before stopping
     *                   always treated as positive (sign is ignored)
     * @param power      the desired motor power magnitude (min 0.0, max 1.0)
     *                   always positive since range sensor is on the front
     * @param brake      whether or not to brake the motors when done
     */
    public static void DriveAndGrab(double distanceCM, double power, boolean brake)
    {
        autoPilotFunc = AutoPilotFunc.DRIVE_AND_GRAB;
        driveAndGrabState = DriveAndGrabState.START;
        DriveSetupCM(distanceCM, power, brake);
    }

    // Following are a few methods to turn the robot

    /**
     * Does common setup for starting/making turns.
     *
     * @param degrees      the desired heading in degrees (min 0.0, max 360.0)
     *                     0 is north/forward, 90 is east/starboard/right, etc.
     * @param toleranceDeg the desired tolerance in degrees
     *                     always positive
     * @param power        the desired motor power (min 0.0, max 1.0)
     * @param brake        whether or not to brake the motors when done
     */
    private static void TurnSetupDeg(double degrees, double toleranceDeg, double power, boolean brake)
    {
        autoPilotState = AutoPilotState.START;
        targetDeg = degrees;
        targetTolDeg = toleranceDeg;
        targetPower = Range.clip(Math.abs(power), 0.0, 1.0);
        targetBrake = brake;
    }

    /**
     * Start a turn to an absolute heading.
     * In this context, "absolute" means relative to the position of the robot
     * when the opmode originally initialized.  If the robot always starts in
     * the same position on the field, then this turn can be thought of as being
     * relative to the field.
     * The reference heading never changes during an entire opmode.
     * Can be cancelled with AutoPilot.Reset
     *
     * @param degrees      the desired heading in degrees (min 0.0, max 360.0)
     *                     0 is north/forward, 90 is east/starboard/right, etc.
     * @param toleranceDeg the desired tolerance in degrees
     *                     always positive
     * @param power        the desired motor power (min 0.0, max 1.0)
     * @param brake        whether or not to brake the motors when done
     */
    public static void TurnAbsoluteDeg(double degrees, double toleranceDeg, double power, boolean brake)
    {
        autoPilotFunc = AutoPilotFunc.TURNABS;
        TurnSetupDeg(degrees, toleranceDeg, power, brake);
    }

    /**
     * Start a turn to a heading relative to the current robot position.
     * Can be cancelled with AutoPilot.Reset
     *
     * @param degrees      the desired turn in degrees (min -180.0, max +180.0)
     *                     0 is north/forward, 90 is east/starboard/right, etc.
     * @param toleranceDeg the desired tolerance in degrees
     *                     always positive
     * @param power        the desired motor power (min 0.0, max 1.0)
     * @param brake        whether or not to brake the motors when done
     */
    public static void TurnRelativeDeg(double degrees, double toleranceDeg, double power, boolean brake)
    {
        autoPilotFunc = AutoPilotFunc.TURNREL;
        TurnSetupDeg(degrees, toleranceDeg, power, brake);
    }

    /**
     * Move the arm to a given position.
     * Can be cancelled with AutoPilot.Reset
     *
     * @param pos the desired position of the arm (ARM_POS_MIN to ARM_POS_MAX)
     */
    public static void SetArmPos(int pos)
    {
        autoPilotFunc = AutoPilotFunc.SET_ARM_POS;
        autoPilotState = AutoPilotState.START;
        targetArmPos = pos;
    }

    /**
     * Start a wait for a given number of milliseconds.
     * Can be cancelled with AutoPilot.Reset
     *
     * @param timeMS the desired number of milliseconds to wait
     */
    public static void Wait(int timeMS)
    {
        autoPilotFunc = AutoPilotFunc.WAIT;
        autoPilotState = AutoPilotState.START;
        targetWaitMS = timeMS;
    }

    /**
     * Determine if any autopilot function is currently running.
     *
     * @return boolean true if autopilot is running
     */
    public static boolean IsRunning()
    {
        return (autoPilotState == AutoPilotState.RUNNING);
    }

    /**
     * Cancel any in-progress autopilot function and reset to idle state.
     */
    public static void Reset()
    {
        targetDistCM = 0.0;
        targetWaitMS = 0;
        autoPilotFunc = AutoPilotFunc.NONE;
        autoPilotState = AutoPilotState.IDLE;
        driveAndGrabState = DriveAndGrabState.IDLE;
    }

    /**
     * Method to perform whatever autopilot function might be in progress.
     * Should be called once for every iteration of the opmode run loop.
     */
    public static void DoAutopilot()
    {
        switch (autoPilotState)
        {
            case IDLE:
                break;
            case START:
                DoAutoPilotStart();
                break;
            case RUNNING:
                DoAutoPilotRunning();
                break;
            case CLEANUP:
                DoAutoPilotCleanup();
                break;
            case DONE:
                break;
        }
    }

    /**
     * Method to perform actions for autopilot START state.
     */
    private static void DoAutoPilotStart()
    {
        switch (autoPilotFunc)
        {
            case DRIVE:
                Robot.DriveToPositionCM(targetDistCM, targetPower, targetBrake);
                break;
            case DRIVE_TO_RANGE:
                Robot.DriveToRangeCM(targetDistCM, targetPower, targetBrake);
                break;
            case SET_ARM_POS:
                Robot.SetArmPos(targetArmPos);
                break;
            case DRIVE_AND_GRAB:
                // start the arm lowering and the grip opening simultaneously
                Robot.SetArmPos(Robot.ARM_MIN_POS);
                Robot.SetServoPos(gripper, Robot.SERVO_MAX_POS);
                driveAndGrabState = DriveAndGrabState.ARM_LOWERING_AND_GRIP_OPENING;
                break;
            case TURNABS:
                Robot.TurnAbsoluteDeg(targetDeg, targetTolDeg, targetPower, targetBrake);
                break;
            case TURNREL:
                Robot.TurnRelativeDeg(targetDeg, targetTolDeg, targetPower, targetBrake);
                break;
            case WAIT:
                Robot.Wait(targetWaitMS);
                break;
        }
        autoPilotState = AutoPilotState.RUNNING;
    }

    /**
     * Method to perform actions for autopilot RUNNING state.
     */
    private static void DoAutoPilotRunning()
    {
        boolean done = false;
        switch (autoPilotFunc)
        {
            case DRIVE:
                done = Robot.IsDriveToPositionDone();
                break;
            case DRIVE_TO_RANGE:
                done = Robot.IsDriveToRangeDone();
                break;
            case SET_ARM_POS:
                done = Robot.IsArmDone();
                break;
            case DRIVE_AND_GRAB:
                done = DoAutoPilotRunningDriveAndGrab();
                break;
            case TURNABS:
            case TURNREL:
                done = Robot.IsTurnDone();
                break;
            case WAIT:
                done = Robot.IsWaitDone();
                break;
            default:
                break;
        }
        if (done)
        {
            autoPilotState = AutoPilotState.CLEANUP;
        }
    }

    /**
     * Method to perform actions for autopilot RUNNING state for DRIVE_AND_GRAB function.
     */
    private static boolean DoAutoPilotRunningDriveAndGrab()
    {
        boolean done = false;
        switch (driveAndGrabState)
        {
            case IDLE:
                break;
            case START:
                break;
            case ARM_LOWERING_AND_GRIP_OPENING:
                if (Robot.IsArmDone())
                {
                    driveAndGrabState = DriveAndGrabState.GRIP_OPENING;
                }
                if (Robot.IsServoDone(gripper))
                {
                    driveAndGrabState = DriveAndGrabState.ARM_LOWERING;
                }
                break;
            case GRIP_OPENING:
                if (Robot.IsServoDone(gripper))
                {
                    Robot.SetDriveBrake(targetBrake);
                    Robot.SetDrivePower(targetPower, targetPower);
                    driveAndGrabState = DriveAndGrabState.DRIVING;
                }
                break;
            case ARM_LOWERING:
                if (Robot.IsArmDone())
                {
                    Robot.DriveToRangeCM(targetDistCM, targetPower, targetBrake);
                    driveAndGrabState = DriveAndGrabState.DRIVING;
                }
                break;
            case DRIVING:
                if (Robot.IsDriveToRangeDone())
                {
                    Robot.CleanupDriveToRange();
                    Robot.SetServoPos(gripper, Robot.SERVO_MIN_POS);
                    driveAndGrabState = DriveAndGrabState.GRIP_CLOSING;
                }
                break;
            case GRIP_CLOSING:
                if (Robot.IsServoDone(gripper))
                {
                    Robot.SetArmPos(Robot.ARM_MAX_POS);
                    driveAndGrabState = DriveAndGrabState.ARM_RAISING;
                }
                break;
            case ARM_RAISING:
                if (Robot.IsArmDone())
                {
                    driveAndGrabState = DriveAndGrabState.RESET;
                }
                break;
            case RESET:
                done = true;
                driveAndGrabState = DriveAndGrabState.DONE;
                break;
            case DONE:
                break;
        }
        return done;
    }

    /**
     * Method to perform actions for autopilot CLEANUP state.
     */
    private static void DoAutoPilotCleanup()
    {
        switch (autoPilotFunc)
        {
            case DRIVE:
                Robot.CleanupDriveToPosition();
                break;
            case DRIVE_TO_RANGE:
                Robot.CleanupDriveToRange();
                break;
            case TURNABS:
            case TURNREL:
                Robot.CleanupTurn();
                break;
            case WAIT:
                break;
        }
        autoPilotState = AutoPilotState.DONE;
    }
}