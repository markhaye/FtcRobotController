package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot
{
    private static final int DRIVE_MOTOR_CPR = 1120; // TODO - figure this out
    // drive gear ratio is ( motor shaft rotations / wheel shaft rotations )
    // This is < 1.0 if geared UP or > 1.0 if geared DOWN
    private static final double DRIVE_GEAR_RATIO = 1.0; // TODO - figure this out
    private static final double WHEEL_DIAMETER_CM = 9.0; // TODO - figure this out
    private static final double COUNTS_PER_CM = (DRIVE_MOTOR_CPR * DRIVE_GEAR_RATIO) /
            (WHEEL_DIAMETER_CM * Math.PI);
    private static LinearOpMode lom;
    private static BNO055IMU imu;
    private static DcMotor LeftTop, LeftBottom, RightTop, RightBottom;

    public static void init(LinearOpMode opMode)
    {
        lom = opMode;

        // set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        // no logging = faster response?
        parameters.loggingEnabled = false;

        // set up internal gyro sensor
        imu = lom.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // wait for gyro sensor calibration
        while (!imu.isGyroCalibrated())
        {
            ;
        }

        LeftTop = lom.hardwareMap.get(DcMotor.class, "LT");
        LeftBottom = lom.hardwareMap.get(DcMotor.class, "LB");
        RightTop = lom.hardwareMap.get(DcMotor.class, "RT");
        RightBottom = lom.hardwareMap.get(DcMotor.class, "RB");
        LeftTop.setDirection(DcMotor.Direction.FORWARD);
        LeftBottom.setDirection(DcMotor.Direction.FORWARD);
        RightTop.setDirection(DcMotor.Direction.REVERSE);
        RightBottom.setDirection(DcMotor.Direction.REVERSE);
        SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Get current absolute compass heading.
     * Value is "absolute" or relative to to last IMU calibration (opmode start).
     *
     * @return heading in degrees ... 0 to 360, 0=forward/north, 90=right/east/starboard, etc.
     */
    public static double GetAbsoluteCompassHeading()
    {
        double heading = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        if (heading < 0.0)
            heading += 360.0;
        return heading;
    }

    /**
     * Get a string abbreviation for the absolute compass heading (N,NE,E,SE, etc.)
     * In this context, "absolute" means relative to the position of the robot
     * when the opmode originally initialized.
     *
     * @return heading as string (N,NE,E,SE, etc.)
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

    /**
     * Sets mode for all drive motors
     *
     * @param mode desired mode
     */
    private static void SetDriveMode(DcMotor.RunMode mode)
    {
        LeftTop.setMode(mode);
        LeftBottom.setMode(mode);
        RightTop.setMode(mode);
        RightBottom.setMode(mode);
    }

    /**
     * Set the power for the drive motors.
     *
     * @param leftPower  power for the left motor
     * @param rightPower power for the right motor
     */
    public static void SetDrivePower(double leftPower, double rightPower)
    {
        LeftTop.setPower(leftPower);
        LeftBottom.setPower(leftPower);
        RightTop.setPower(leftPower);
        RightBottom.setPower(leftPower);
    }

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
     * Set the drive motors to drive to a given distance.
     * This method does not wait for the move to be complete.
     * Use IsDriveToPositionDone to determine if this operation is complete.
     * Use CleanupDriveToPosition to cancel or cleanup the operation.
     *
     * NOTE THIS REQUIRES THE USE OF THE BUILT-IN ENCODERS, NOT ODOMETERS
     *
     * @param distanceCM distance to move in centimeters
     * @param power      power magnitude for motors (min 0.0, max 1.0)
     */
    public static void DriveToPositionCM(double distanceCM, double power)
    {
        int targetCounts;
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetCounts = GetCountForCM(distanceCM);
        LeftTop.setTargetPosition(targetCounts);
        LeftBottom.setTargetPosition(targetCounts);
        RightTop.setTargetPosition(targetCounts);
        RightBottom.setTargetPosition(targetCounts);
        SetDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        return !(LeftTop.isBusy() || RightTop.isBusy());
    }

    /**
     * Cancel (if running) and cleanup a drive to position operation.
     * Use SetDriveToPositionCM to start an operation.
     * Use IsDriveToPositionDone to determine if this operation is complete.
     */
    public static void CleanupDriveToPosition()
    {
        SetDrivePower(0.0, 0.0);
        SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
