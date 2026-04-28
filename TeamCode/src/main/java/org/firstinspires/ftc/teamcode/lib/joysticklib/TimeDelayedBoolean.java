package org.firstinspires.ftc.teamcode.lib.joysticklib;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class contains a boolean value and a timer. It can set its boolean value and return whether the timer is within
 * a set timeout. This returns true if the stored value is true and the timeout has expired.
 */
public class TimeDelayedBoolean {

    private ElapsedTime elapsedTime = new ElapsedTime();
    private boolean m_old = false;

    /**
     * Updates the internal state and checks if the given timeout has expired.
     *
     * @param value   The new boolean value to process.
     * @param timeout The timeout duration in seconds.
     * @return True if the stored value is true and the timeout has expired, otherwise false.
     */
    public boolean update(boolean value, double timeout) {
        if (!m_old && value) {
            elapsedTime.reset();
        }
        m_old = value;
        return value && elapsedTime.seconds() >= timeout;
    }
}
