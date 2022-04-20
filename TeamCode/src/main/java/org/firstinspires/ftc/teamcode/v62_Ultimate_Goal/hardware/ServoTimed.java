package org.firstinspires.ftc.teamcode.v62_Ultimate_Goal.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * class to implement a timer to determine when a servo has completed positioning
 */
public class ServoTimed
{
    private Servo myServo;
    private int maxTimeMS;
    private ElapsedTime timer;
    private double endTimeMS, currentPos;

    /**
     * Constructor for the class
     * @param inServo the servo object
     * @param inMaxTimeMS the maximum time to move the servo in milliseconds
     */
    public ServoTimed(Servo inServo, int inMaxTimeMS)
    {
        myServo = inServo;
        maxTimeMS = inMaxTimeMS;
        endTimeMS = 0.0;
        timer = new ElapsedTime();
        currentPos = myServo.getPosition();
    }

    /**
     * Method to get the current REQUESTED position, which may or may not be the
     * ACTUAL position.
     * @return position min 0.0, max 1.0
     */
    public double GetServoPos()
    {
        currentPos = myServo.getPosition();
        return currentPos;
    }

    /**
     * Method to set a new position for the servo
     * @param pos requested position min 0.0, max 1.0
     */
    public void SetServoPos(double pos)
    {
        pos = Range.clip(pos, 0.0, 1.0);
        timer.reset();
        endTimeMS = Math.abs(currentPos - pos) * maxTimeMS;
        myServo.setPosition(pos);
        currentPos = pos;
    }

    /**
     * Method to determine if the servo has had enough time to get to the last
     * requested position
     * @return true if enough time has elapsed
     */
    public boolean IsServoDone()
    {
        return (timer.milliseconds() >= endTimeMS);
    }
}
