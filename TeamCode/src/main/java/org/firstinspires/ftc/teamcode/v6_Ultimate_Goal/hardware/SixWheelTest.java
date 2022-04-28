package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name = "SixWheelTest", group = "Test")
public class SixWheelTest extends LinearOpMode
{
    static final double DRIVE_SCALING = 0.5;
    static DcMotor right_drive_a;
    static DcMotor right_drive_b;
    static DcMotor left_drive_a;
    static DcMotor left_drive_b;
    double drive, turn, tgtPowerLeft, tgtPowerRight, driveScale;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // get motors and configure them
        left_drive_a = hardwareMap.get(DcMotor.class, "left_drive_m18");
        left_drive_b = hardwareMap.get(DcMotor.class, "left_drive_m20");
        right_drive_a = hardwareMap.get(DcMotor.class, "right_drive_m19");
        right_drive_b = hardwareMap.get(DcMotor.class, "right_drive_m14");
        left_drive_a.setDirection(DcMotor.Direction.FORWARD);
        left_drive_b.setDirection(DcMotor.Direction.FORWARD);
        right_drive_a.setDirection(DcMotor.Direction.REVERSE);
        right_drive_b.setDirection(DcMotor.Direction.REVERSE);
        left_drive_a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive_a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive_a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_drive_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive_a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive())
        {
            // use left stick to move robot
            drive = gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x;

            // press both bumpers to get full power
            if (gamepad1.right_bumper && gamepad1.left_bumper)
                driveScale = 1.0;
                // press either bumper to get half power
            else if (gamepad1.left_bumper || gamepad1.right_bumper)
                driveScale = 0.5;
                // press neither bumper to get quarter power
            else
                driveScale = 0.25;

            tgtPowerLeft = driveScale * turn;
            tgtPowerRight = -driveScale * turn;
            tgtPowerLeft -= driveScale * drive;
            tgtPowerRight -= driveScale * drive;

            telemetry.addData("tgtPowerLeft", tgtPowerLeft);
            telemetry.addData("tgtPowerRight", tgtPowerRight);
            telemetry.update();

            left_drive_a.setPower(Range.clip(tgtPowerLeft, -1.0, 1.0));
            left_drive_b.setPower(Range.clip(tgtPowerLeft, -1.0, 1.0));
            right_drive_a.setPower(Range.clip(tgtPowerRight, -1.0, 1.0));
            right_drive_b.setPower(Range.clip(tgtPowerRight, -1.0, 1.0));
        }
    }
}
