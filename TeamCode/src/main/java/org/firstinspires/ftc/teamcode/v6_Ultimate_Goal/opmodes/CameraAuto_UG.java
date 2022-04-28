package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.opmodes;

/*
adapted from external.samples/ConceptTensorFlowObjectDetection
 */

import static java.lang.Double.NaN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class CameraAuto_UG extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AWfOFc3/////AAABmXMsSsa4mkLNs5JY/osSiR9YHaLw9uPlqefxONDXQPuT7/6lGxp2vfjMuePmvdiXXlKXj+OuYhfiiVTaqEiLUUwZZgMLqKgdkEIYxVHjKU8jo6PzKM/smXAsFUy3AMU+Vn74cleRqpX2y7HqWPYcnwyRGxlYdm2kEGSjtjCvmil5LlpgUtLf2hH0IbMQoXjKwRaANPndk6/s/574vNBkoTbFS9nhdNrTmJ3gW0imfmsGhQIydC+9nIGnAQOqOvTDztn69VVwm+yOUY/eCoqtUngSiAsT6uILXgjpRayDb7QZoOlYSi+NjYtydXZXjskrLeNZ6u4JXix6J8/8ZlrkE611G0JWBEMOjdaQf/oplPhb\n";

    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod = null;

    int numberOfRings = -1;
    char targetZone = 'Z';
    double confidence = 0.0;

    @Override
    public void runOpMode()
    {
        // set up object detection stuff
        initVuforia();
        initTfod();

        // continually detect objects until game starts or stops
        while (!isStarted() && !isStopRequested() && tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                numberOfRings = 0;
                targetZone = 'A';
                confidence = NaN;

                // step through the list of recognitions and see what was found
                for (Recognition recognition : updatedRecognitions)
                {
                    if (recognition.getLabel().equals("Single"))
                    {
                        numberOfRings = 1;
                        targetZone = 'B';
                    }
                    // there are only two recognizable objects, if it's not one, it must be the other
                    else
                    {
                        numberOfRings = 4;
                        targetZone = 'C';
                    }
                    confidence = 100.0 * recognition.getConfidence();
                }
            }
            telemetry.addData("numberOfRings", numberOfRings);
            telemetry.addData("targetZone", targetZone);
            telemetry.addData("confidence", "%.1f%%", confidence);
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();
        }

        // game has started, kill object detection stuff
        if (tfod != null)
        {
            tfod.shutdown();
        }

        // don't need waitForStart(); because the above loop accomplishes the same thing

        // start doing normal autonomous stuff
        while (opModeIsActive())
        {
            telemetry.addData("numberOfRings", numberOfRings);
            telemetry.addData("This is the robot going to target zone", targetZone);
            telemetry.update();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        if (tfod != null)
        {
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
            tfod.activate();
            tfod.setZoom(3.0, 16.0 / 9.0);
        }
    }
}
