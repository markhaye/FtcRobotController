package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/*
Remote control opmode for driving the team robot
 */
@TeleOp
public class TeleDrive extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double drive = 0.0, turn = 0.0, scale = 0.0, leftPower = 0.0, rightPower = 0.0;

        // first, initialize the robot utility class and wait for start button
        Robot.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // as long as the opmode is running, keep driving the robot
        while (opModeIsActive())
        {
            // do telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Power", "Left=%.2f, Right=%.2f", leftPower, rightPower);
            telemetry.addData("Heading", "%3.1f (%s)",
                    Robot.GetAbsoluteCompassHeading(), Robot.GetCompassDir());
            telemetry.update();

            // get the gamepad inputs that will be used for driving and turning
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x;

            // get the gamepad inputs that will be used to determine drive scale
            if (gamepad1.left_bumper && gamepad1.right_bumper)
                scale = 1.0;
            else if (gamepad1.left_bumper || gamepad1.right_bumper)
                scale = 0.666;
            else
                scale = 0.333;

            // and finally, drive
            leftPower = scale * (drive + turn);
            rightPower = scale * (drive - turn);
            Robot.SetDrivePower(leftPower, rightPower);
        }
    }
}
