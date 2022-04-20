package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware.HardwareGort;
import org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware.KernelGort;

public class CalibrationGort extends LinearOpMode
{
    private LinearOpMode myOpMode = this;

    @Override
    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();
        int leftPos, rightPos;
        double leftCoefficient, rightCoefficient, error;

        // set up our hardware, passing the opmode so we have access to all the goodies
        //HardwareGort.getInstance();
        HardwareGort.init(myOpMode);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // reset the timer and encoders for a clean start
        runtime.reset();
        HardwareGort.motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HardwareGort.motorDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to use encoders
        HardwareGort.motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HardwareGort.motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // start motion.
        HardwareGort.motorDriveLeft.setPower(KernelGort.DRIVE_POWER);
        HardwareGort.motorDriveRight.setPower(KernelGort.DRIVE_POWER);

        // run for a fixed amount of time
        while (myOpMode.opModeIsActive() && (runtime.seconds() < 10));

        // Stop all motion;
        HardwareGort.motorDriveLeft.setPower(0);
        HardwareGort.motorDriveRight.setPower(0);

        // calculate relative power
        leftPos = HardwareGort.motorDriveLeft.getCurrentPosition();
        rightPos = HardwareGort.motorDriveRight.getCurrentPosition();
        if (leftPos > rightPos)
        {
            leftCoefficient = 100.0;
            rightCoefficient = 100.0 * rightPos / leftPos;
            error = 100.0 - rightCoefficient;
        }
        else
        {
            rightCoefficient = 100.0;
            leftCoefficient = 100.0 * leftPos / rightPos;
            error = 100.0 - leftCoefficient;
        }

        myOpMode.telemetry.addData("Motor positions", "Left %7d, right %7d",
                leftPos, rightPos);
        myOpMode.telemetry.addData("Relative power", "Left %.3f, right %.3f, error %.3f",
                leftCoefficient, rightCoefficient, error);
        myOpMode.telemetry.update();

        sleep(8000 ); // wait for me to look at the telemetry
    }
}
