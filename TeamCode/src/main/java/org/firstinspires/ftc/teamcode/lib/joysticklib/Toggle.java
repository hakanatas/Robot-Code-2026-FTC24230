package org.firstinspires.ftc.teamcode.lib.joysticklib;

public class Toggle {
    private boolean state;
    private boolean lastButtonState;

    /**
     * Constructor to initialize the toggle with an initial state.
     *
     * @param initialState The initial state of the toggle.
     */
    public Toggle(boolean initialState) {
        this.state = initialState;
        this.lastButtonState = false;
    }

    /**
     * Updates the toggle state based on the button input.
     *
     * @param buttonPressed The current state of the button (true if pressed, false otherwise).
     */
    public void update(boolean buttonPressed) {
        if (buttonPressed && !lastButtonState) {
            state = !state; // Toggle the state when the button is pressed.
        }
        lastButtonState = buttonPressed; // Save the current button state for the next update.
    }

    /**
     * Gets the current state of the toggle.
     *
     * @return The current state of the toggle (true or false).
     */
    public boolean getState() {
        return state;
    }
}
