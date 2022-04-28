package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.ServoTimed;

@TeleOp
public class TurretTest extends LinearOpMode
{
    private static final int TURRET_MOTOR_CPR = 288;   // standard for 31-1300
    private static final int TURRET_MOTOR_PINION_TEETH = 15;
    private static final int TURRET_PLATFORM_GEAR_TEETH = 125;
    private static final double TURRET_COUNT_PER_DEGREE = 1.0 * TURRET_MOTOR_CPR *
            TURRET_PLATFORM_GEAR_TEETH / TURRET_MOTOR_PINION_TEETH / 360.0;
    private static final double TURRET_MOTOR_INIT_POWER = 0.6;

    private static DcMotor left, right, turretMotor;
    private static double drive, turn, leftPower, rightPower, turretPower, servoPos;
    private static int maxTurretPos, turretPos;
    private static TouchSensor touch, turretLimit;

    private static Servo servoGrip;                 // REV 41-1097
    private static CRServo servoSide;               // REV 41-1097

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
        turretLimit = hardwareMap.get(TouchSensor.class,"sensorTurretLimit");
        touch = hardwareMap.get(TouchSensor.class,"sensorTouch");

        // following code assumes that the magnetic limit sensor is in between the magnets,
        // i.e. that the turret is in the legal/safe range
        // first go towards minimum
        turretMotor.setPower(-TURRET_MOTOR_INIT_POWER);
        while (!turretLimit.isPressed() && !isStopRequested())
        {
            telemetry.addData("finding", "min turret pos");
            telemetry.addData("turretLimit", turretLimit.isPressed());
            telemetry.update();
        }
        turretMotor.setPower(0.0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // min has been set, now have to run forward for at least a little bit to get away from limit
        turretMotor.setPower(TURRET_MOTOR_INIT_POWER);
        while((turretPos = turretMotor.getCurrentPosition()) < 100)
        {
            telemetry.addData("moving", "away from min turret pos");
            telemetry.addData("turretLimit", turretLimit.isPressed());
            telemetry.addData("turretPos", turretPos);
            telemetry.update();
        }
        // now go towards maximum
        while (!turretLimit.isPressed() && !isStopRequested())
        {
            telemetry.addData("finding", "max turret pos");
            telemetry.addData("turretLimit", turretLimit.isPressed());
            telemetry.update();
        }
        turretMotor.setPower(0.0);
        maxTurretPos = turretMotor.getCurrentPosition();
        telemetry.addData("maxTurretPos", maxTurretPos);
        telemetry.addData("max turret angle", maxTurretPos/TURRET_COUNT_PER_DEGREE);
        // drive turret to mid position
        turretMotor.setTargetPosition(maxTurretPos/2);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);
        while (turretMotor.isBusy() && !isStopRequested())
        {}
        turretMotor.setPower(0.0);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        while (!gamepad1.x)
        {
            turretMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("turretLimit", turretLimit.isPressed());
            telemetry.addData("set", "min turret pos with X");
            telemetry.update();
        }
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!gamepad1.b)
        {
            turretMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("turretLimit", turretLimit.isPressed());
            telemetry.addData("set", "max turret pos with B");
            telemetry.update();
        }
        maxTurretPos = turretMotor.getCurrentPosition();
        telemetry.addData("maxTurretPos", maxTurretPos);

         */
        servoGrip = hardwareMap.get(Servo.class, "servoGrip");
        servoGrip.resetDeviceConfigurationForOpMode();
        servoGrip.setPosition(0.5);
        // continuous rotation servo
        servoSide = hardwareMap.get(CRServo.class, "servoSide");
        servoSide.resetDeviceConfigurationForOpMode();

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
            if ((turretPos < 10) && !gamepad1.a) turretPower = Math.max(turretPower, 0.0);
            if ((turretPos > (maxTurretPos - 10)) && !gamepad1.a) turretPower = Math.min(turretPower, 0.0);

            // tell the robot what to do
            left.setPower(leftPower);
            right.setPower(rightPower);
            turretMotor.setPower(turretPower);

            servoPos = 0.5 + gamepad1.right_stick_y * 0.5;
            servoGrip.setPosition(servoPos);
            servoSide.setPower(gamepad1.right_stick_x);

            // display values on the phone
            telemetry.addData("power", "left %.2f right %.2f", leftPower, rightPower);
            telemetry.addData("turretPower", "%.2f", turretPower);
            telemetry.addData("turretPos", turretPos);
            telemetry.addData("turret angle", "%.1f", turretPos/TURRET_COUNT_PER_DEGREE);
            telemetry.addData("turretLimit", turretLimit.isPressed());
            telemetry.addData("touch", touch.isPressed());
            telemetry.update();
        }
    }
}
