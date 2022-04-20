package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class HardwareGort
{
    private static LinearOpMode om;         // anchor for opmode

    // package-private (public within package) constants
    public static final int DRIVE_MOTOR_CPR = 1120; // standard for 31-1301
    // drive gear ratio is ( motor shaft rotations / wheel shaft rotations )
    // This is < 1.0 if geared UP or > 1.0 if geared DOWN
    public static final double DRIVE_GEAR_RATIO = 72.0 / 60.0;
    public static final double WHEEL_DIAMETER_CM = 9.0;
    public static final double COUNTS_PER_CM = (DRIVE_MOTOR_CPR * DRIVE_GEAR_RATIO) /
            (WHEEL_DIAMETER_CM * Math.PI);
    // Track is distance between drive wheels
    public static final double WHEEL_TRACK_CM = 20.0;
    public static final double SERVO_MIN_POS = 0.0;
    public static final double SERVO_MID_POS = 0.5;
    public static final double SERVO_MAX_POS = 1.0;
    public static final int ARM_MOTOR_CPR = 288;   // standard for 31-1300
    public static final double ARM_POWER = 0.25;
    public static final double ARM_MAX_POS = 55.0;
    public static final double ARM_MIN_POS = 5.0;

    // Package-private (public within package) variables

    // declarations for hardware
    public static BNO055IMU imu;                   // REV 31-1595
    private static TouchSensor sensorTouch; // REV 31-1425
    public static DistanceSensor sensorRange;      // REV 31-1557
    public static ColorSensor sensorColor;         // REV 31-1557
    public static DcMotor motorDriveLeft;          // REV 31-1301
    public static DcMotor motorDriveRight;         // REV 31-1301
    public static DcMotor motorArm;                // REV 31-1300
    public static Servo servoGrip;                 // REV 41-1097
    public static Servo servoSide;                 // REV 41-1097

    // constructor is private to ensure there is only one instance
    // (everything is static, so there's no need to instantiate anyway)
    private HardwareGort()
    {
    }

    // method to initialize the hardware
    public static void init(LinearOpMode opMode)
    {
        boolean running = true;

        // save reference to opMode
        om = opMode;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        // no logging = faster response?
        parameters.loggingEnabled = false;

        // set up internal gyro sensor
        imu = om.hardwareMap.get(BNO055IMU.class, "imu"); // was Gyroscope.class
        imu.initialize(parameters);

        // set up external sensors
        sensorTouch = om.hardwareMap.get(TouchSensor.class, "sensorTouch");
        // the range sensor claims to work for distances of 1-10cm, but in practice it never seems to report more than 4cm
        sensorRange = om.hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        // grab the same sensor again as color
        sensorColor = om.hardwareMap.get(ColorSensor.class, "sensorColorRange");
        sensorColor.enableLed(false); // turn off the light

        // Define and initialize motors
        motorDriveLeft = om.hardwareMap.get(DcMotor.class, "motorDriveLeft");
        motorDriveRight = om.hardwareMap.get(DcMotor.class, "motorDriveRight");
        motorArm = om.hardwareMap.get(DcMotor.class, "motorArm");

        // set direction for all motors
        motorDriveLeft.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRight.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to run with encoders.
        motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set zero power behavior for all motors
        motorDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);
        motorArm.setPower(0);

        // Define and initialize all installed servos.
        servoGrip = om.hardwareMap.get(Servo.class, "servoGrip");
        servoSide = om.hardwareMap.get(Servo.class, "servoSide");
        servoGrip.setPosition(SERVO_MAX_POS);
        servoSide.setPosition(SERVO_MIN_POS);

        // drive arm down until it hits the touch sensor
        //om.telemetry.addData("Status", "Initializing arm ...");
        //om.telemetry.update();
        motorArm.setPower(-ARM_POWER);
        // unpressed touch sensor state is true, pressed is false; wait for false
        while (running)
            running = !sensorTouch.isPressed();
        motorArm.setPower(0.0);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // make sure the imu gyro is calibrated before continuing.
        om.telemetry.addData("Status", "Calibrating IMU ...");
        om.telemetry.update();
        while (om.opModeIsActive() && !imu.isGyroCalibrated())
            om.sleep(50);
        om.telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());

        // set up kernel routines
        KernelGort.init(om);
    }

    public static void SetDrivePowerGort(double leftMotorPower, double rightMotorPower)
    {
        motorDriveLeft.setPower(leftMotorPower);
        motorDriveRight.setPower(rightMotorPower);
    }
}
