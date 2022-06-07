package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TurretTest extends LinearOpMode
{
    private static final int TURRET_MOTOR_CPR = 288;   // standard for 31-1300
    private static final int TURRET_MOTOR_PINION_TEETH = 15;
    private static final int TURRET_PLATFORM_GEAR_TEETH = 125;
    private static final double TURRET_COUNT_PER_DEGREE = 1.0 * TURRET_MOTOR_CPR *
            TURRET_PLATFORM_GEAR_TEETH / TURRET_MOTOR_PINION_TEETH / 360.0;
    private static final double TURRET_MOTOR_MIN_POWER = 0.2;
    private static final double TURRET_MOTOR_MAX_POWER = 1.0;
    private static final double SQRT3 = 1.7320508075688772935274463415059;

    private static DcMotor left, right, turretMotor;
    private static TouchSensor touch, turretLimit;
    private static Servo servoGrip;                 // REV 41-1097
    private static CRServo servoArm;               // REV 41-1097
    private static AnalogInput armRot;

    private static double drive, turn, leftPower, rightPower, turretPower, servoPos;
    private static int maxTurretPos, turretPos;

    @Override
    public void runOpMode() throws InterruptedException
    {
        left = hardwareMap.get(DcMotor.class, "motorDriveLeft");
        right = hardwareMap.get(DcMotor.class, "motorDriveRight");
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor = hardwareMap.get(DcMotor.class, "motorArm");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretLimit = hardwareMap.get(TouchSensor.class, "sensorTurretLimit");
        touch = hardwareMap.get(TouchSensor.class, "sensorTouch");
        armRot = hardwareMap.get(AnalogInput.class, "sensorArmRot");

        servoGrip = hardwareMap.get(Servo.class, "servoGrip");
        servoGrip.resetDeviceConfigurationForOpMode();
        servoGrip.setPosition(0.5);
        // continuous rotation servo
        servoArm = hardwareMap.get(CRServo.class, "servoArm");
        servoArm.resetDeviceConfigurationForOpMode();

        // following code assumes that the turret is in the legal/safe range
        // first go towards minimum
        telemetry.addData("finding", "min turret pos");
        telemetry.update();
        turretMotor.setPower(-TURRET_MOTOR_MAX_POWER);
        while (!turretLimit.isPressed() && !isStopRequested()) ;
        turretMotor.setPower(0.0);
        sleep(100); // give the brakes time to stop the turntable
        turretMotor.setPower(TURRET_MOTOR_MIN_POWER);
        while (!turretLimit.isPressed() && !isStopRequested())
            ; // wait for it to be on again (in case of overshoot)
        while (turretLimit.isPressed() && !isStopRequested()) ;
        turretMotor.setPower(0.0);
        sleep(4096);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("finding", "max turret pos");
        telemetry.update();
        turretMotor.setPower(TURRET_MOTOR_MAX_POWER);
        // min has been set, now have to run forward for at least a little bit to get away from limit
        while ((turretMotor.getCurrentPosition() < 40) && !isStopRequested()) ;
        // now continue towards maximum
        while (!turretLimit.isPressed() && !isStopRequested()) ;
        turretMotor.setPower(0.0);
        sleep(100); // give the brakes time to stop the turntable
        turretMotor.setPower(-TURRET_MOTOR_MIN_POWER);
        while (!turretLimit.isPressed() && !isStopRequested())
            ; // wait for it to be on again (in case of overshoot)
        while (turretLimit.isPressed() && !isStopRequested()) ;
        turretMotor.setPower(0.0);
        maxTurretPos = turretMotor.getCurrentPosition() / 2;
        sleep(4096);
        // drive turret to mid position
        turretMotor.setTargetPosition(maxTurretPos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_MOTOR_MAX_POWER);
        while (turretMotor.isBusy() && !isStopRequested()) ;
        sleep(100); // give it a little more time to settle
        turretMotor.setPower(0.0);
        // set midpoint to be 0
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("maxTurretPos", maxTurretPos);
        telemetry.addData("max turret angle", "%.1f", maxTurretPos / TURRET_COUNT_PER_DEGREE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // get gamepad input for driving and turning
            drive = -gamepad1.left_stick_y;
            turn = -gamepad1.left_stick_x;
            leftPower = drive - turn;
            rightPower = drive + turn;

            // get gamepad input for turret ("a" button overrides turret limits)
            turretPos = turretMotor.getCurrentPosition();
            turretPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if ((turretPos < -maxTurretPos) && !gamepad1.a)
                turretPower = Math.max(turretPower, 0.0);
            if ((turretPos > maxTurretPos) && !gamepad1.a)
                turretPower = Math.min(turretPower, 0.0);

            // tell the robot what to do
            left.setPower(leftPower);
            right.setPower(rightPower);
            turretMotor.setPower(turretPower);

            servoPos = 0.5 + gamepad1.right_stick_x * 0.5;
            servoGrip.setPosition(servoPos);
            servoArm.setPower(gamepad1.right_stick_y);

            // display values on the phone
            telemetry.addData("power", "left %.2f right %.2f", leftPower, rightPower);
            telemetry.addData("turretPower", "%.2f", turretPower);
            telemetry.addData("turretPos", turretPos);
            telemetry.addData("turret angle", "%.1f", turretPos / TURRET_COUNT_PER_DEGREE);
            telemetry.addData("turretLimit", turretLimit.isPressed());
            telemetry.addData("touch", touch.isPressed());
            telemetry.addData("armRot", "volts=%.2f, degrees=%.1f", armRot.getVoltage(), GetArmRotDegrees(armRot.getVoltage()));
            telemetry.update();
        }
    }

    private double GetArmRotDegrees(double voltage)
    {
        voltage += 0.0000000001; // make sure it's not zero
        // formula derived from the following formula from potentiometer's data sheet on REV's website
        // voltage = 445.5 * (degrees - 270.0) / (degrees * (degrees - 270.0) - 36450.0)
        return 6.75 * (-Math.sqrt(1089.0 / (voltage * voltage) - (1320.0 / voltage) + 1200.0) + (33.0 / voltage) + 20.0);
    }
}
