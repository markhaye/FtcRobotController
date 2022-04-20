package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class KernelGort
{
    private static LinearOpMode om;

    // imu/gyro stuff
    private static double robotHeading, lastHeading;
    private static PIDController pidRotate = null;

    // set some constants
    public static final double DRIVE_POWER = 0.6;
    static final double TURN_POWER = 0.5;

    // Public variables

    // constructor is private to ensure there is only one instance
    // (everything is static, so there's no need to instantiate anyway)
    private KernelGort()
    {
    }

    // =============================================================================================
    // private functions
    // =============================================================================================

    /**
     * computes simple modulo for angles, returns value in 0-360 range
     *
     * @param heading input heading
     * @return output heading
     */
    private static double moduloHeading(double heading)
    {
        if (heading < 0.0)
            heading += 360.0;
        else if (heading > 360.0)
            heading -= 360.0;
        return heading;
    }

    /*
    Resets the current mulative angle tracking to straight forward
    */
    private static void resetHeading()
    {
        // we want compass heading (0 to 360, 0=forward/north, 90=right/starboard/east, etc.)
        lastHeading = getAbsoluteCompassHeading();
        robotHeading = 0.0;
    }

    /**
     * Get current relative heading
     *
     * @return heading in degrees ... 0 to 360, 0=forward/north, 90=right/starboard/east, etc.
     */
    private static double getHeading()
    {
        // we want compass heading (0 to 360, 0=forward/north, 90=right/starboard/east, etc.)
        double thisHeading = getAbsoluteCompassHeading();
        double deltaHeading = moduloHeading(thisHeading - lastHeading);
        lastHeading = thisHeading;
        robotHeading = moduloHeading(robotHeading + deltaHeading);

        return robotHeading;
    }

    // =============================================================================================
    // public functions
    // =============================================================================================

    /**
     * Initializes the kernel objects
     *
     * @param opMode current opmode reference
     */
    public static void init(LinearOpMode opMode)
    {
        // save reference to opmode
        om = opMode;
        // create pid controller for rotation
        if (pidRotate == null)
        {
            // make a new one ... PID values are irrelevant here and will be reset later
            pidRotate = new PIDController(0.0, 0.0, 0.0);
            // getHeading will return values from 0 to 360
            pidRotate.setInputRange(0.0, 360.0);
            // heading is continuous, i.e. 0 and 360 are the same thing
            pidRotate.setContinuous();
            // motor power will be 0.0 to 1.0
            pidRotate.setOutputRange(0.0, 1.0);
            // set tolerance as a percentage of the input range, e.g. 0.28% of 360 is about 1.0
            pidRotate.setTolerance(0.277777);
            // must settle for a bit before done
            pidRotate.setSettleIterations(16);
        }
        else
            pidRotate.reset();
    }

    /**
     * Get current relative compass heading
     * value is relative to last reset, which occurs during every gyro turn
     *
     * @return heading in degrees ... 0 to 360, 0=forward/north, 90=right/starboard/east, etc.
     */
    public static double getRelativeCompassHeading()
    {
        return getHeading();
    }

    /**
     * Get current absolute compass heading
     * value is "absolute" or relative to to last IMU calibration (opmode start)
     *
     * @return heading in degrees ... 0 to 360, 0=forward/north, 90=right/starboard/east, etc.
     */
    public static double getAbsoluteCompassHeading()
    {
        double heading = -HardwareGort.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (heading < 0.0)
            heading += 360.0;
        return heading;
    }

    /**
     * gets current absolute IMU heading
     * value is "absolute" or relative to to last IMU calibration (opmode start)
     *
     * @return heading in degrees ... -180 to +180, 0=forward/north, -90=right/starboard/east, etc.
     */
    public static double getAbsoluteIMUHeading()
    {
        return HardwareGort.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /*
    "macro" that will open the gripper, lower the arm, close the gripper, and raise the arm
    doesn't matter where the arm or servo are on entry
    implemented as a finite state machine to prevent waiting and keep the robot responsive
    */
    public enum ArmGrabState
    {
        IDLE, START, OPENING, LOWERING, CLOSING, RAISING, DONE
    }

    public static ArmGrabState ArmGrab(ArmGrabState grabState)
    {
        switch (grabState)
        {
            case IDLE:
                break;
            case START:
                grabState = ArmGrabState.OPENING;
                break;
            case OPENING:
                HardwareGort.servoGrip.setPosition(HardwareGort.SERVO_MIN_POS);
                if (HardwareGort.servoGrip.getPosition() <= HardwareGort.SERVO_MIN_POS)
                    grabState = ArmGrabState.LOWERING;
                break;
            case LOWERING:
                HardwareGort.motorArm.setPower(-HardwareGort.ARM_POWER); // lower arm
                if (HardwareGort.motorArm.getCurrentPosition() <= HardwareGort.ARM_MIN_POS)
                    grabState = ArmGrabState.CLOSING;
                break;
            case CLOSING:
                HardwareGort.motorArm.setPower(0.0); // stop arm
                HardwareGort.servoGrip.setPosition(HardwareGort.SERVO_MAX_POS); // close grip
                if (HardwareGort.servoGrip.getPosition() >= HardwareGort.SERVO_MAX_POS)
                    grabState = ArmGrabState.RAISING;
                break;
            case RAISING:
                HardwareGort.motorArm.setPower(HardwareGort.ARM_POWER); // raise arm
                if (HardwareGort.motorArm.getCurrentPosition() >= HardwareGort.ARM_MAX_POS)
                    grabState = ArmGrabState.DONE;
                break;
            case DONE:
                HardwareGort.motorArm.setPower(0.0); // stop arm
            default:
                grabState = ArmGrabState.IDLE;
                break;
        }
        return grabState;
    }

    /**
     * Move in a "straight" line a given number of centimeters
     * notes: most useful during autonomous, not so much during teleop
     * "straight" depends on relative power/friction of drive motors
     *
     * @param centimeters distance to move (positive=forward or negative=reverse)
     * @param power       motor power (0.0 to 1.0)
     */
    public static void DriveCm(double centimeters, double power)
    {
        int newLeftTarget, newRightTarget;
        boolean running = true;

        // ensure distance and power are reasonable
        centimeters = Range.clip(centimeters, -100.0, 100.0);
        power = Math.min(Math.abs(power), 1.0);

        // Ensure that the opmode is still active
        if (om.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newLeftTarget = HardwareGort.motorDriveLeft.getCurrentPosition() + (int) (centimeters * HardwareGort.COUNTS_PER_CM + 0.5);
            newRightTarget = HardwareGort.motorDriveRight.getCurrentPosition() + (int) (centimeters * HardwareGort.COUNTS_PER_CM + 0.5);
            HardwareGort.motorDriveLeft.setTargetPosition(newLeftTarget);
            HardwareGort.motorDriveRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            HardwareGort.motorDriveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            HardwareGort.motorDriveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            HardwareGort.motorDriveLeft.setPower(power);
            HardwareGort.motorDriveRight.setPower(power);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: Use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, to stop the motion.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (running)
                running = (om.opModeIsActive() && HardwareGort.motorDriveLeft.isBusy() &&
                        HardwareGort.motorDriveRight.isBusy());

            // Stop all motion;
            HardwareGort.motorDriveLeft.setPower(0);
            HardwareGort.motorDriveRight.setPower(0);

            // Turn off RUN_TO_POSITION
            HardwareGort.motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            HardwareGort.motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(8000 ); // wait for me to look at the telemetry
        }
    }

    /**
     * Move in a "straight" line a given number of inches
     * notes: most useful during autonomous, not so much during teleop
     * "straight" depends on relative power/friction of drive motors
     *
     * @param inches distance to move (positive=forward or negative=reverse)
     * @param power  motor power (0.0 to 1.0)
     */
    static void DriveInch(double inches, double power)
    {
        DriveCm(inches * 2.54, power);
    }

    /**
     * Rotate back to orientation when imu was last reset (e.g. when opmode was started)
     * does not return until rotation is complete
     */
    public static void ReOrient()
    {
        double degrees = getAbsoluteIMUHeading();
        if (Math.abs(degrees) > 2.0)
            TurnGyroPID(degrees, TURN_POWER);
    }

    /**
     * Rotate left or right the number of degrees relative to current heading.
     * notes: most useful during autonomous, not so much during teleop
     * Does not support turning more than 180 degrees
     * does not return until rotation is complete
     * uses PID controller, which might behave unpredictably, depending on degrees and power ...
     *
     * @param degrees Degrees to turn, + is right - is left (min -180, max 180)
     * @param power   motor power to use, 0.0 to 1.0 ... this is initial power, PID controller
     *                may raise power up to max of 1.0 if needed
     */
    public static int TurnGyroPID(double degrees, double power)
    {
        double Kp, Ki, Kd;
        boolean running = true;
        int iterations = 0;

        // ensure angle and power are reasonable
        degrees = Range.clip(degrees, -180.0, 180.0);
        power = Math.min(Math.abs(power), 1.0);

        // reset imu angle tracking.
        resetHeading();

        // Starting value for p is proportional to the max power value (power) divided by
        // the maximum error value, degrees, so should limit max result
        // i and d are proportional to p, based on manual tuning and testing with multiple
        // power levels and angles
        Kp = power / Math.abs(degrees);
        Ki = Kp / 400.0;
        Kd = Kp * 20.0;
        pidRotate.setPID(Kp, Ki, Kd);

        // getHeading returns compass heading (0-360 with 0=forward/north, 90=right/east, etc.),
        // so adjust target degrees to use it as a setpoint
        if (degrees < 0.0)
            degrees += 360.0;

        // Start PID controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to try to prevent
        // the robot's momentum from overshooting the turn too much. The PID controller reports
        // onTarget() = true when the difference between turn angle and target angle is within the
        // specified tolerance AND has been there for at least the specified settling time.
        // If the controller overshoots, it will reverse the sign of the output, turning the robot
        // back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.enable();

        // rotate until turn is completed.
        while (running)
        {
            iterations++;
            power = pidRotate.performPID(getHeading()); // power will be + on right turn and - on left.
            HardwareGort.motorDriveLeft.setPower(power);
            HardwareGort.motorDriveRight.setPower(-power);
            running = om.opModeIsActive() && !pidRotate.onTarget();
        }

        // turn the motors off.
        HardwareGort.motorDriveRight.setPower(0);
        HardwareGort.motorDriveLeft.setPower(0);
        return iterations;
    }

    /**
     * Rotate left or right the number of degrees relative to current heading.
     * notes: most useful during autonomous, not so much during teleop
     * Does not support turning more than 180 degrees
     * does not return until rotation is complete
     * does not use PID control
     *
     * @param degrees Degrees to turn, + is right - is left (min -180, max 180)
     * @param power   motor power to use, 0.0 to 1.0
     */
    public static void TurnGyroBasic(double degrees, double power)
    {
        boolean running = true;

        // ensure angle and power are reasonable
        degrees = Range.clip(degrees, -180.0, 180.0);
        power = Math.min(Math.abs(power), 1.0);

        // adjust degrees down a bit to prevent overshoot ... full power gets 20% drop
        degrees *= (1 - power * 0.2);

        // reset imu movement tracking.
        resetHeading();

        if (degrees < 0.0)     // turn left.
        {
            // getHeading returns compass heading (0 to 360, 0=forward/north, 90=right/east, etc.),
            // so adjust target degrees to match
            degrees += 360.0;
            power = -power;
        }

        // start motion
        HardwareGort.motorDriveLeft.setPower(power);
        HardwareGort.motorDriveRight.setPower(-power);

        // need to get off zero (variable amount based on power)
        om.sleep((long) ((1.05 - Math.abs(power)) * 500));

        // rotate until turn is completed or we have to stop or time limit is reached
        if (degrees > 180.0)    // left (counter-clockwise) turn
            while (running)
                running = (om.opModeIsActive() && (getHeading() > degrees));
        else                    // right (clockwise) turn
            while (running)
                running = (om.opModeIsActive() && (getHeading() < degrees));

        // turn the motors off.
        HardwareGort.motorDriveRight.setPower(0);
        HardwareGort.motorDriveLeft.setPower(0);

        // wait for rotation to stop (variable amount based on power)
        om.sleep((long) (Math.abs(power) * 1000));
    }

    /**
     * Turn a given number of degrees using encoders (odometry)
     * notes: most useful during autonomous, not so much during teleop
     * Does not support turning more than 180 degrees
     * does not return until rotation is complete
     * results are highly dependant on wheel traction/slippage
     *
     * @param degrees Degrees to turn, + is right - is left (min -180, max 180)
     * @param power   motor power to use, 0.0 to 1.0
     */
    static void TurnOdo(double degrees, double power)
    {
        int delta, newLeftTarget, newRightTarget;
        boolean running = true;

        // ensure angle and power are reasonable
        degrees = Range.clip(degrees, -180.0, 180.0);
        power = Math.min(Math.abs(power), 1.0);

        // Ensure that the opmode is still active
        if (om.opModeIsActive())
        {
            double fudge;
            // Determine new target position, and pass to motor controller
            // for a turn, subtract from left and add to right
            // not sure why the fudge factor needs to be there (poor traction?) ... but it kinda works
            if (degrees > 0.0)
                fudge = 1.25;
            else
                fudge = 1.18;
            delta = (int) (fudge * degrees / 360 * HardwareGort.WHEEL_TRACK_CM * Math.PI * HardwareGort.COUNTS_PER_CM + 0.5);
            newLeftTarget = HardwareGort.motorDriveLeft.getCurrentPosition() + delta;
            newRightTarget = HardwareGort.motorDriveRight.getCurrentPosition() - delta;
            HardwareGort.motorDriveLeft.setTargetPosition(newLeftTarget);
            HardwareGort.motorDriveRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            HardwareGort.motorDriveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            HardwareGort.motorDriveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            HardwareGort.motorDriveLeft.setPower(power);
            HardwareGort.motorDriveRight.setPower(power);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: Use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, to stop the motion.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (running)
                running = (om.opModeIsActive() && HardwareGort.motorDriveLeft.isBusy() &&
                        HardwareGort.motorDriveRight.isBusy());

            // Stop all motion;
            HardwareGort.motorDriveLeft.setPower(0);
            HardwareGort.motorDriveRight.setPower(0);

            // Turn off RUN_TO_POSITION
            HardwareGort.motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            HardwareGort.motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
