package org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware;

// class for all the hardware on the robot and methods to interact with the hardware

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.utilities.AutoPilot;

public class Robot
{
    private static OpMode om = null;

    // private constants
    private static final int DRIVE_MOTOR_CPR = 1120; // standard for 31-1301
    // drive gear ratio is ( motor shaft rotations / wheel shaft rotations )
    // This is < 1.0 if geared UP or > 1.0 if geared DOWN
    private static final double DRIVE_GEAR_RATIO = 72.0 / 60.0;
    private static final double WHEEL_DIAMETER_CM = 9.0;
    private static final double COUNTS_PER_CM = (DRIVE_MOTOR_CPR * DRIVE_GEAR_RATIO) /
            (WHEEL_DIAMETER_CM * Math.PI);
    // Track is distance between drive wheels
    private static final double WHEEL_TRACK_CM = 20.0;
    public static final double SERVO_MIN_POS = 0.0;
    public static final double SERVO_MID_POS = 0.5;
    public static final double SERVO_MAX_POS = 1.0;
    private static final int SERVO_MAX_TIME_MS = 1200; // about how long it takes to move the whole range
    private static final int ARM_MOTOR_CPR = 288;   // standard for 31-1300
    private static final double ARM_MAX_POWER = 0.25;
    public static final int ARM_MAX_POS = 55;
    public static final int ARM_MIN_POS = 5;

    // declarations for hardware
    private static BNO055IMU imu;                   // REV 31-1595
    private static TouchSensor sensorTouch;         // REV 31-1425
    private static DistanceSensor sensorRange;      // REV 31-1557
    private static ColorSensor sensorColor;         // REV 31-1557
    private static DcMotor motorDriveLeft;          // REV 31-1301
    private static DcMotor motorDriveRight;         // REV 31-1301
    private static DcMotor motorArm;                // REV 31-1300
    private static Servo servoGrip;                 // REV 41-1097
    private static Servo servoSide;                 // REV 41-1097

    private static ServoTimed servoGripTimed, servoSideTimed;

    // various state variables
    public enum OpModeState
    {INITIALIZING, RUNNING}

    public static OpModeState opModeState;

    private enum ImuInitState
    {START, CALIBRATINGGYRO, CALIBRATINGACCEL, CALIBRATINGMAG, DONE}

    private static ImuInitState imuState;

    private enum ArmInitState
    {START, MOVE2TOUCH, MOVE2MIN, DONE}

    private static ArmInitState armState;
    private static int armPos;

    private static boolean headingStale = true;
    private static double headingDeg, lastHeadingDeg, relativeHeadingDeg;
    private static double startAngleDeg, targetAngleDeg, deltaAngleDeg, toleranceAngleDeg;
    private static double targetRangeCM;
    private static int targetCounts, deltaCounts;
    private static double leftPow, rightPow;
    private static int currentMS, startWaitMS, targetWaitMS, deltaMS;

    /**
     * Constructor is private to prevent multiple instances.
     * Everything is static, so no need to instantiate at all.
     */
    private Robot()
    {
    }

    /**
     * begin the initialization of the class
     *
     * @param opMode the current opmode object
     */
    public static void init(OpMode opMode)
    {
        // save reference to opMode
        om = opMode;

        opModeState = OpModeState.INITIALIZING;

        // set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        // no logging = faster response?
        parameters.loggingEnabled = false;

        // set up internal gyro sensor
        imu = om.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imuState = ImuInitState.START;

        // set up external sensors
        sensorTouch = om.hardwareMap.get(TouchSensor.class, "sensorTouch");
        // the range sensor claims to work for distances of 1-10cm,
        // but in practice it never seems to report more than 4cm
        sensorRange = om.hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        // grab the same sensor again as color
        sensorColor = om.hardwareMap.get(ColorSensor.class, "sensorColorRange");
        sensorColor.enableLed(false); // turn off the light

        // Define and initialize all installed servos.
        servoGrip = om.hardwareMap.get(Servo.class, "servoGrip");
        servoSide = om.hardwareMap.get(Servo.class, "servoSide");
        servoGrip.setPosition(SERVO_MID_POS);
        servoSide.setPosition(SERVO_MID_POS);
        servoGripTimed = new ServoTimed(servoGrip, SERVO_MAX_TIME_MS);
        servoSideTimed = new ServoTimed(servoSide, SERVO_MAX_TIME_MS);

        // Define and initialize drive motors
        motorDriveLeft = om.hardwareMap.get(DcMotor.class, "motorDriveLeft");
        motorDriveRight = om.hardwareMap.get(DcMotor.class, "motorDriveRight");
        motorDriveLeft.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRight.setDirection(DcMotor.Direction.REVERSE);
        motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);

        // set up arm motor
        motorArm = om.hardwareMap.get(DcMotor.class, "motorArm");
        motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setPower(0);
        armState = ArmInitState.START;

        // set up gamepad(s)
        GPad.init(om);

        // set up autopilot
        AutoPilot.init(om);
    }

    /**
     * continue the initialization of the class
     */
    public static void init_loop()
    {
        // make sure the imu gyro is calibrated before continuing.
        switch (imuState)
        {
            case START:
                imuState = ImuInitState.CALIBRATINGGYRO;
                break;
            case CALIBRATINGGYRO:
                if (imu.isGyroCalibrated())
                {
                    imuState = ImuInitState.CALIBRATINGACCEL;
                }
                break;
            case CALIBRATINGACCEL:
                if (imu.isAccelerometerCalibrated())
                {
                    // !!! can't get magnetometer calibrated, so skip it
                    imuState = ImuInitState.DONE;
                }
                break;
            case CALIBRATINGMAG:
                if (imu.isMagnetometerCalibrated())
                {
                    imuState = ImuInitState.DONE;
                }
                break;
            case DONE:
                break;
        }

        // drive arm down until it hits the touch sensor
        // seems to stay in MOVING state for a while after hitting the touch sensor ... ???
        switch (armState)
        {
            case START:
                motorArm.setPower(-ARM_MAX_POWER);
                armState = ArmInitState.MOVE2TOUCH;
                break;
            case MOVE2TOUCH:
                if (sensorTouch.isPressed())
                {
                    motorArm.setPower(0.0);
                    motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorArm.setTargetPosition(ARM_MIN_POS);
                    armPos = ARM_MIN_POS;
                    motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorArm.setPower(ARM_MAX_POWER);
                    armState = ArmInitState.MOVE2MIN;
                }
                break;
            case MOVE2MIN:
                if (!motorArm.isBusy())
                {
                    armState = ArmInitState.DONE;
                }
                break;
            case DONE:
                break;
        }
        Robot.DoTelemetry();
    }

    /**
     * begin running the opmode
     */
    public static void start()
    {
        opModeState = OpModeState.RUNNING;
    }

    /**
     * continue running the opmode
     */
    public static void loop()
    {
        currentMS = (int) (om.getRuntime() * 1000 + 0.5);
        AutoPilot.DoAutopilot();
        DoTelemetry();
        headingStale = true;
    }

    /**
     * Report the important state variables for this class.
     */
    public static void DoTelemetry()
    {
        om.telemetry.addData("robot state", opModeState);
        switch (opModeState)
        {
            case INITIALIZING:
                om.telemetry.addData("arm init state", armState);
                om.telemetry.addData("IMU init state", "%s, calibration %s",
                        imuState, imu.getCalibrationStatus().toString());
                break;
            case RUNNING:
                /* om.telemetry.addData("motor power", "left=%.3f, right=%.3f",
                        leftPow, rightPow); */
                om.telemetry.addData("ArmPos", armPos);
                om.telemetry.addData("Heading", "%3.1f (%s), Range=%1.1f",
                        GetAbsoluteCompassHeading(), GetCompassDir(), GetRangeCM());
                AutoPilot.DoTelemetry(opModeState);
                om.telemetry.addData("Target", "dist=%.1f, angle=%.1f, time=%d",
                        GetCMForCount(targetCounts), targetAngleDeg, targetWaitMS);
                om.telemetry.addData("Current", "dist=%.1f, angle=%.1f, time=%d",
                        GetCMForCount(deltaCounts), deltaAngleDeg, deltaMS);
                break;
        }
    }

    // following are some methods that work with the robot's heading

    /**
     * Check if we have a recent heading from the imu, if not get one.
     */
    private static void UpdateHeading()
    {
        // only get imu at most once per cycle
        if (headingStale)
        {
            headingDeg = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;
            headingStale = false;
        }
    }

    /**
     * Gets current absolute IMU heading.
     * Value is "absolute" or relative to to last IMU calibration (opmode start).
     *
     * @return heading in degrees ... -180 to +180, 0=forward/north, -90=right/east/starboard, etc.
     */
    public static double GetAbsoluteIMUHeading()
    {
        UpdateHeading();
        return headingDeg;
    }

    /**
     * Get current absolute compass heading.
     * Value is "absolute" or relative to to last IMU calibration (opmode start).
     *
     * @return heading in degrees ... 0 to 360, 0=forward/north, 90=right/east/starboard, etc.
     */
    public static double GetAbsoluteCompassHeading()
    {
        UpdateHeading();
        double heading = -headingDeg;
        if (heading < 0.0)
            heading += 360.0;
        return heading;
    }

    /**
     * Computes simple modulo for angles, returns value in 0-360 range.
     *
     * @param heading input heading
     * @return output heading
     */
    public static double ModuloHeading(double heading)
    {
        while (heading < 0.0)
            heading += 360.0;
        while (heading >= 360.0)
            heading -= 360.0;
        return heading;
    }

    /**
     * Resets the current cumulative angle tracking to straight forward.
     */
    private static void ResetRelativeHeading()
    {
        // we want compass heading (0 to 360, 0=forward/north, 90=right/east/starboard, etc.)
        lastHeadingDeg = GetAbsoluteCompassHeading();
        relativeHeadingDeg = 0.0;
    }

    /**
     * Get current relative heading.
     * Value is relative to the last reset or turn.
     *
     * @return heading in degrees ... 0 to 360, 0=forward/north, 90=right/east/starboard, etc.
     */
    private static double GetRelativeHeading()
    {
        // we want compass heading (0 to 360, 0=forward/north, 90=right/starboard/east, etc.)
        double thisHeading = GetAbsoluteCompassHeading();
        double deltaHeading = ModuloHeading(thisHeading - lastHeadingDeg);
        lastHeadingDeg = thisHeading;
        relativeHeadingDeg = ModuloHeading(relativeHeadingDeg + deltaHeading);

        return relativeHeadingDeg;
    }

    /**
     * Get a string abbreviation for the absolute compass heading (N,NE,E,SE, etc.)
     * In this context, "absolute" means relative to the position of the robot
     * when the opmode originally initialized.
     *
     * @return heading in degrees ... 0 to 360, 0=forward/north, 90=right/east/starboard, etc.
     */
    public static String GetCompassDir()
    {
        final String[] headings = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        double heading = GetAbsoluteCompassHeading() + 22.5;
        if (heading >= 360.0)
        {
            heading -= 360.0;
        }
        int index = (int) (heading / 45.0);
        return headings[index];
    }

    // following are some methods to read sensors

    /**
     * Determine if the touch sensor is currently pressed.
     *
     * @return whether or not the touch sensor is currently pressed
     */
    public static boolean IsTouchPressed()
    {
        return sensorTouch.isPressed();
    }

    /**
     * Get the current value of the range sensor.
     *
     * @return current range in centimeters
     */
    public static double GetRangeCM()
    {
        return sensorRange.getDistance(DistanceUnit.CM);
    }

    /**
     * Get the average encoder value for the drive motors.
     *
     * @return current average encoder count
     */
    public static int GetCount()
    {
        return (motorDriveLeft.getCurrentPosition() + motorDriveRight.getCurrentPosition()) / 2;
    }

    // following are some methods for driving straight

    /**
     * Get the encoder count needed to travel a given distance.
     *
     * @param distanceCM desired travel distance in centimeters
     * @return number of encoder counts required
     */
    public static int GetCountForCM(double distanceCM)
    {
        return (int) (distanceCM * COUNTS_PER_CM + 0.5);
    }

    /**
     * Get the distance that will be travelled for a given encoder count.
     *
     * @param count desired number of encoder counts
     * @return equivalent distance in centimeters
     */
    public static double GetCMForCount(int count)
    {
        return count / COUNTS_PER_CM;
    }

    /**
     * Set the braking (zero power behavior) for the drive motors.
     *
     * @param brake whether or not braking is desired
     */
    public static void SetDriveBrake(boolean brake)
    {
        if (brake)
        {
            motorDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            motorDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * Set the power for the drive motors.
     *
     * @param leftPower  power for the left motor
     * @param rightPower power for the right motor
     */
    public static void SetDrivePower(double leftPower, double rightPower)
    {
        leftPow = Range.clip(leftPower, -1.0, 1.0);
        rightPow = Range.clip(rightPower, -1.0, 1.0);
        motorDriveLeft.setPower(leftPow);
        motorDriveRight.setPower(rightPow);
    }

    /**
     * Set the drive motors to drive to a given distance.
     * This method does not wait for the move to be complete.
     * Use IsDriveToPositionDone to determine if this operation is complete.
     * Use CleanupDriveToPosition to cancel or cleanup the operation.
     *
     * @param distanceCM distance to move in centimeters
     * @param power      power magnitude for both motors (min 0.0, max 1.0)
     * @param brake      whether or not to brake
     */
    public static void DriveToPositionCM(double distanceCM, double power, boolean brake)
    {
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        SetDriveBrake(brake);
        motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetCounts = GetCountForCM(distanceCM);
        motorDriveLeft.setTargetPosition(targetCounts);
        motorDriveRight.setTargetPosition(targetCounts);
        motorDriveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetDrivePower(power, power);
    }

    /**
     * Determine if a previously started drive to position is complete.
     * Use SetDriveToPositionCM to start an operation.
     * Use CleanupDriveToPosition to cancel or cleanup the operation.
     *
     * @return whether or not the operation is complete
     */
    public static boolean IsDriveToPositionDone()
    {
        deltaCounts = Robot.GetCount();
        return !(motorDriveLeft.isBusy() || motorDriveRight.isBusy());
    }

    /**
     * Cancel (if running) and cleanup a drive to position operation.
     * Use SetDriveToPositionCM to start an operation.
     * Use IsDriveToPositionDone to determine if this operation is complete.
     */
    public static void CleanupDriveToPosition()
    {
        SetDrivePower(0.0, 0.0);
        motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Start driving the robot until the distance sensor detects an obstacle.
     * Use IsDriveToRangeDone to determine if this operation is complete.
     * Use CleanupDriveToRange() to cancel this operation.
     * @param distanceCM the desired maximum distance to obstacle before stopping
     *                   always treated as positive (sign is ignored)
     * @param power      the desired motor power magnitude (min 0.0, max 1.0)
     *                   always positive since range sensor is on the front
     * @param brake      whether or not to brake the motors when done
     */
    public static void DriveToRangeCM(double distanceCM, double power, boolean brake)
    {
        targetRangeCM = Math.abs(distanceCM);
        SetDriveBrake(brake);
        SetDrivePower(Math.abs(power), Math.abs(power));
    }

    /**
     * Determine if a previously started drive to range is complete.
     * Use DriveToRangeCM to start an operation.
     * Use CleanupDriveToRange to cancel or cleanup the operation.
     *
     * @return whether or not the operation is complete
     */
    public static boolean IsDriveToRangeDone()
    {
        return (Robot.GetRangeCM() < targetRangeCM);
    }

    /**
     * Cancel (if running) and cleanup a drive to range operation.
     * Use DriveToRangeCM to start an operation.
     * Use IsDriveToRangeDone to determine if this operation is complete.
     */
    public static void CleanupDriveToRange()
    {
        SetDrivePower(0.0, 0.0);
    }

    /**
     * Does common setup for starting/making turns.
     *
     * @param toleranceDeg the desired tolerance in degrees
     *                     always positive
     */
    private static void SetupTurnCommonDeg(double toleranceDeg)
    {
        startAngleDeg = org.firstinspires.ftc.teamcode.hardware.Robot.GetAbsoluteCompassHeading();
        toleranceAngleDeg = Math.abs(toleranceDeg);
    }

    /**
     * Set the drive motors to start turning a given angle.
     * Use IsTurnDone to determine if this operation is complete.
     * Use CleanupTurn to cancel or cleanup the operation.
     *
     * @param power power magnitude for both motors (min 0.0, max 1.0)
     * @param brake whether or not to brake
     */
    private static void StartTurn(double power, boolean brake)
    {
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        SetDriveBrake(brake);
        if (targetAngleDeg > 0.0)
        {
            SetDrivePower(power, -power);
        }
        else
        {
            SetDrivePower(-power, power);
        }
    }

    /**
     * Turn to an absolute heading.
     * In this context, "absolute" means relative to the position of the robot
     * when the opmode originally initialized.  If the robot always starts in
     * the same position on the field, then this turn can be thought of as being
     * relative to the field.
     * The reference heading never changes during an entire opmode.
     *
     * @param degrees      the desired heading in degrees (min 0.0, max 360.0)
     *                     0 is north/forward, 90 is east/starboard/right, etc.
     * @param toleranceDeg the desired tolerance in degrees
     *                     always positive
     * @param power        power magnitude for both motors (min 0.0, max 1.0)
     * @param brake        whether or not to brake
     */
    public static void TurnAbsoluteDeg(double degrees, double toleranceDeg, double power, boolean brake)
    {
        SetupTurnCommonDeg(toleranceDeg);
        targetAngleDeg = Robot.ModuloHeading(degrees - startAngleDeg);
        // determine shortest path to desired heading
        if (targetAngleDeg > 180.0)
        {
            targetAngleDeg -= 360.0;
        }
        StartTurn(power, brake);
    }

    /**
     * Turn to a heading relative to the current robot position.
     *
     * @param degrees      the desired turn in degrees (min -180.0, max +180.0)
     *                     positive is clockwise, negative is counter-clockwise
     * @param toleranceDeg the desired tolerance in degrees
     *                     always positive
     * @param power        power magnitude for both motors (min 0.0, max 1.0)
     * @param brake        whether or not to brake
     */
    public static void TurnRelativeDeg(double degrees, double toleranceDeg, double power, boolean brake)
    {
        SetupTurnCommonDeg(toleranceDeg);
        // ensure angle is reasonable
        targetAngleDeg = Range.clip(degrees, -180.0, 180.0);
        StartTurn(power, brake);
    }

    public static boolean IsTurnDone()
    {
        deltaAngleDeg = GetAbsoluteCompassHeading() - startAngleDeg;
        if ((targetAngleDeg > 0.0) && (deltaAngleDeg < 0.0))
        {
            deltaAngleDeg += 360.0;
        }
        if ((targetAngleDeg < 0.0) && (deltaAngleDeg > 0.0))
        {
            deltaAngleDeg -= 360.0;
        }
        return ((deltaAngleDeg > targetAngleDeg - toleranceAngleDeg) &&
                (deltaAngleDeg < targetAngleDeg + toleranceAngleDeg));
    }

    /**
     * Cancel (if running) and cleanup a turn operation.
     * Use SetupTurnRelative or SetupTurnAbsolute to compute target angle.
     * Use StartTurn to start an operation.
     * Use IsTurnDone to determine if this operation is complete.
     */
    public static void CleanupTurn()
    {
        targetAngleDeg = 0.0;
        SetDrivePower(0.0, 0.0);
    }

    // following methods are for operating the arm

    /**
     * Sets the position for the arm motor.
     * Used during autonomous or in teleop via autopilot
     *
     * @param pos desired position for the arm (min ARM_MIN_POS, max ARM_MAX_POS)
     */
    public static void SetArmPos(int pos)
    {
        armPos = Range.clip(pos, ARM_MIN_POS, ARM_MAX_POS);
        motorArm.setTargetPosition(armPos);
        if (motorArm.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setPower(ARM_MAX_POWER);
        }
    }

    /**
     * Check if arm is done with latest position request.
     *
     * @return true if done (i.e. desired position was reached)
     */
    public static boolean IsArmDone()
    {
        return !motorArm.isBusy();
    }

    /**
     * Sets the power for the arm motor.
     * Used during teleop via joystick.
     * If a RUN_TO_POSITION operation is still busy, nothing happens.
     *
     * @param power desired power level for arm (min -1.0, max 1.0)
     */
    public static void SetArmPower(double power)
    {
        power = Range.clip(power, -1.0, 1.0);
        if (!motorArm.isBusy())
        {
            if (motorArm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
            {
                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            armPos = motorArm.getCurrentPosition();
            if ((armPos <= ARM_MIN_POS && power < 0.0) || (armPos >= ARM_MAX_POS && power > 0.0))
            {
                power = 0.0;
            }
            motorArm.setPower(ARM_MAX_POWER * power);
        }
    }

    /**
     * Start a wait for a given number of milliseconds.
     * Check for done with Robot.IsWaitDone()
     * Can be cancelled with ???
     *
     * @param timeMS the desired number of milliseconds to wait
     */
    public static void Wait(int timeMS)
    {
        startWaitMS = currentMS;
        targetWaitMS = timeMS;
    }

    /**
     * Check if wait time has elapsed.
     * Start a wait with Robot.Wait()
     * Can be cancelled with ???
     *
     * @return true if wait is elapsed
     */
    public static boolean IsWaitDone()
    {
        deltaMS = currentMS - startWaitMS;
        return (deltaMS > targetWaitMS);
    }

    // following methods are for operating servos

    public static ServoTimed GetGripServo()
    {
        return servoGripTimed;
    }

    public static ServoTimed GetSideServo()
    {
        return servoSideTimed;
    }

    /**
     * Get servo last requested position.
     * This will NOT return current position!
     *
     * @param whichServo specify which servo to check
     * @return last requested position (min 0.0, max 1.0)
     */
    public static double GetServoPos(ServoTimed whichServo)
    {
        return whichServo.GetServoPos();
    }

    /**
     * Set servo to a specific position.
     *
     * @param whichServo specify which servo to set
     * @param pos        desired position for the servo (min 0.0, max 1.0)
     */
    public static void SetServoPos(ServoTimed whichServo, double pos)
    {
        whichServo.SetServoPos(pos);
    }

    /**
     * Check if servo is done with latest position request.
     * This is determined solely based on time.
     *
     * @param whichServo specify which servo to check
     * @return true if done (i.e. enough time has elapsed)
     */
    public static boolean IsServoDone(ServoTimed whichServo)
    {
        return whichServo.IsServoDone();
    }

}
