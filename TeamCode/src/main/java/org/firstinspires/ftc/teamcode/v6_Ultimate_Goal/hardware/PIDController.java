package org.firstinspires.ftc.teamcode.v6_Ultimate_Goal.hardware;

import com.qualcomm.robotcore.util.Range;

public class PIDController
{
    private double m_P;                     // factor for "proportional" control
    private double m_I;                     // factor for "integral" control
    private double m_D;                     // factor for "derivative" control
    private double m_input;                 // sensor input for pid controller
    // input and output ranges are absolute value, i.e. magnitude only, no sign
    // both are usually constant for a given controller
    // input range, used to limit input (sensor) value and setpoint (target)
    private double m_maximumInput = 1.0;    // |maximum input|
    private double m_minimumInput = 0.0;    // |minimum input|
    // output range, used to limit result value
    private double m_maximumOutput = 1.0;   // |maximum output|
    private double m_minimumOutput = 0.0;   // |minimum output|
    private double m_setpoint = 0.0;
    private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = false;      // is the pid controller enabled
    private double m_prevError = 0.0;       // the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
    private double m_maximumError = 0.0;    // max error based on tolerance and min/max input           @mah
    private double m_tolerance = 0.05;      // the percentage (of the input range) error that is considered on target
    private double m_error = 0.0;
    private double m_result = 0.0;
    private int m_settleIterations = 0;     // settling iteration count (default none)                  >>> @mah
    private int m_settlingCount;            // elapsed time settling
    private boolean m_settling = false;     // is the controller settling on target?
    //private long m_maxTime = 120000000000L; // max time in nanoseconds (default two minutes)
    // private long m_maxTimer;                // use System.nanoTime()                                 >>> @mah

    /**
     * Allocate a PID object with the given constants for P, I, D
     *
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    PIDController(double Kp, double Ki, double Kd)
    {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    private void calculate()
    {
        // If enabled then proceed into controller calculations
        if (m_enabled)
        {
            // Calculate the error signal
            m_error = m_setpoint - m_input;

            // If continuous is set to true allow wrap around
            if (m_continuous)
            {
                if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2)
                {
                    if (m_error > 0)
                        m_error = m_error - m_maximumInput + m_minimumInput;
                    else
                        m_error = m_error + m_maximumInput - m_minimumInput;
                }
            }

            // Integrate the errors as long as the upcoming integrator does
            // not exceed the minimum and maximum output thresholds.
            if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                    (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                m_totalError += m_error;

            // Perform the primary PID calculation
            m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

            // Set the current error to the previous error for the next cycle.
            m_prevError = m_error;

            // Make sure the final result is within bounds. If we constrain the result, we make
            // sure the sign of the constrained result matches the original result sign.
            enforceLimit(m_result, m_minimumOutput, m_maximumOutput);                                   // @mah
        }
    }

    /**                                                                                                 >>> @mah
     * enforce limits on a value, preserving the sign of the value
     *
     * @param val the value to be limited
     * @param min minimum allowed value
     * @param max maximum allowed value
     */
    private double enforceLimit(double val, double min, double max)
    {
        int sign = 1;

        if (val < 0) sign = -1; // record sign of val

        if (Math.abs(val) > max)
            val = max * sign;
        else if (Math.abs(val) < min)
            val = min * sign;

        return val;
    }

    /**
     * enforce limits on a value, but only if range is non-zero (max > min)
     *
     * @param val the value to be limited
     * @param min minimum allowed value
     * @param max maximum allowed value
     */
    private double checkRangeEnforceLimit(double val, double min, double max)
    {
        if (max > min)
            val = enforceLimit(val, min, max);
        return val;
    }                                                                                                   /* <<< @mah*/

    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    void setPID(double p, double i, double d)
    {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     *
     * @return proportional coefficient
     */
    public double getP()
    {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     *
     * @return integral coefficient
     */
    public double getI()
    {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     *
     * @return differential coefficient
     */
    double getD()
    {
        return m_D;
    }

    /**
     * Return the current PID result for the last input set with setInput().
     * This is always centered on zero and constrained the the max and min outs
     *
     * @return the latest calculated output
     */
    double performPID()
    {
        calculate();
        return m_result;
    }

    /**
     * Return the current PID result for the specified input.
     *
     * @param input The input value to be used to calculate the PID result.
     *              This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    double performPID(double input)
    {
        setInput(input);
        return performPID();
    }

    /**
     * Set the input value to be used by the next call to performPID().
     *
     * @param input Input value to the PID calculation.
     */
    void setInput(double input)
    {
        m_input = checkRangeEnforceLimit(input, m_minimumInput, m_maximumInput);                        // @mah
    }

    /**
     * Set the PID controller to consider the input to be continuous,
     * Rather then using the max and min as constraints, it considers them to
     * be the same point and automatically calculates the shortest route to
     * the setpoint.
     *
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    void setContinuous(boolean continuous)
    {
        m_continuous = continuous;
    }

    void setContinuous()
    {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input, always positive
     * @param maximumInput the maximum value expected from the output, always positive
     */
    void setInputRange(double minimumInput, double maximumInput)
    {
        m_minimumInput = Math.abs(minimumInput);
        m_maximumInput = Math.abs(maximumInput);
        m_maximumError = Math.abs(m_tolerance / 100.0 * (m_maximumInput - m_minimumInput));             // @mah
        setSetpoint(m_setpoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output, always positive
     * @param maximumOutput the maximum value to write to the output, always positive
     */
    void setOutputRange(double minimumOutput, double maximumOutput)
    {
        m_minimumOutput = Math.abs(minimumOutput);
        m_maximumOutput = Math.abs(maximumOutput);
    }

    /**
     * Set the setpoint for the PIDController
     *
     * @param setpoint the desired setpoint
     */
    void setSetpoint(double setpoint)
    {
        m_setpoint = checkRangeEnforceLimit(setpoint, m_minimumInput, m_maximumInput);                  // @mah
    }

    /**
     * Returns the current setpoint of the PIDController
     *
     * @return the current setpoint
     */
    double getSetpoint()
    {
        return m_setpoint;
    }

    /**
     * Returns the current difference of the input from the setpoint
     *
     * @return the current error
     */
    /*synchronized*/ double getError() // why synchronized? when would multiple threads use this?       @mah
    {
        return m_error;
    }


    /**
     * Set the percentage error which is considered tolerable for use with
     * onTarget. (Input of 15.0 = 15 percent)
     *
     * @param percent error which is tolerable
     */
    void setTolerance(double percent)
    {
        m_tolerance = Range.clip(percent, 0.0, 100.0);
        m_maximumError = Math.abs(m_tolerance / 100.0 * (m_maximumInput - m_minimumInput));             // @mah
    }

    /**                                                                                                 >>> @mah
     * Set the number of iterations for which the error must be withing tolerance before
     * onTarget reports true.
     * Using this means the movement will always take AT LEAST this long to complete
     *
     * @param settleIterations time spent settling, in milliseconds
     */
    void setSettleIterations(int settleIterations)
    {
        m_settleIterations = Range.clip(settleIterations, 0, 32768);
    }

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInputRange.
     * Also enforce the settling.
     *
     * @return true if the error is less than the tolerance for at least the settling time
     */
    boolean onTarget()
    {
        boolean on_Target = (Math.abs(m_error) < m_maximumError);

        // if we're on target and there is a settling time, check settling status
        if (on_Target && (m_settleIterations > 0))
        {
            // if not yet settling, start settling and don't report true yet
            if (!m_settling)
            {
                m_settling = true;
                m_settlingCount = 0;
                on_Target = false;
            }
            // if already settling, but not long enough, don't report true
            else if (m_settlingCount++ < m_settleIterations)
                on_Target = false;
        }
        // not on target or no settleTime - can't be settling
        else
            m_settling = false;

        return (on_Target);
    }                                                                                                   /* <<< @mah*/

    /**
     * Begin running the PIDController
     */
    void enable()
    {
        m_enabled = true;
    }

    /**
     * Stop running the PIDController.
     */
    void disable()
    {
        m_enabled = false;
    }

    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    void reset()
    {
        disable();
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
        m_settling = false;                                                                             // @mah
        m_settlingCount = 0;                                                                            // @mah
    }
}
