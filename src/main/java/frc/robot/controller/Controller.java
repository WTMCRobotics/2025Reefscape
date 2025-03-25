package frc.robot.controller;

import com.google.flatbuffers.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {

    protected final CommandXboxController xboxController;

    protected StickProfile leftProfile = StickProfile.LINEAR;
    protected StickProfile rightProfile = StickProfile.LINEAR;

    protected Deadzone deadzoneType = Deadzone.ROUND;

    protected boolean invertLeftY = false;
    protected boolean invertRightY = false;

    protected boolean invertLeftX = false;
    protected boolean invertRightX = false;

    protected double leftDeadzone = 0.1;
    protected double rightDeadzone = 0.1;

    /**
     * Constructor for the Controller class
     *
     * @param port The port of the controller
     *
     * @return A new Controller object
     */
    public Controller(int port) {
        this.xboxController = new CommandXboxController(port);
    }

    // Setup Methods

    /**
     * Set the deadzone for the left stick
     * Default is 0.1
     *
     * @param deadzone The deadzone value
     *
     * @return The Controller object
     */
    public Controller setLeftDeadzone(double deadzone) {
        this.leftDeadzone = deadzone;
        return this;
    }

    /**
     * Set the deadzone for the right stick
     * Default is 0.1
     *
     * @param deadzone The deadzone value
     *
     * @return The Controller object
     */
    public Controller setRightDeadzone(double deadzone) {
        this.rightDeadzone = deadzone;
        return this;
    }

    /**
     * Set the profile for the left stick
     * Default is LINEAR
     *
     * @param profile The profile to use
     *
     * @return The Controller object
     */
    public Controller setLeftProfile(StickProfile profile) {
        this.leftProfile = profile;
        return this;
    }

    /**
     * Set the profile for the right stick
     * Default is LINEAR
     *
     * @param profile The profile to use
     *
     * @return The Controller object
     */
    public Controller setRightProfile(StickProfile profile) {
        this.rightProfile = profile;
        return this;
    }

    /**
     * Set the deadzone type
     * Default is ROUND
     *
     * @param deadzoneType The deadzone type to use
     *
     * @return The Controller object
     */
    public Controller setDeadzoneType(Deadzone deadzoneType) {
        this.deadzoneType = deadzoneType;
        return this;
    }

    /**
     * Invert the left X axis
     *
     * @return The Controller object
     */
    public Controller invertLeftX() {
        this.invertLeftX = true;
        return this;
    }

    /**
     * Invert the left Y axis
     *
     * @return The Controller object
     */
    public Controller invertLeftY() {
        this.invertLeftY = true;
        return this;
    }

    /**
     * Invert the right X axis
     *
     * @return The Controller object
     */
    public Controller invertRightX() {
        this.invertRightX = true;
        return this;
    }

    /**
     * Invert the right Y axis
     *
     * @return The Controller object
     */
    public Controller invertRightY() {
        this.invertRightY = true;
        return this;
    }

    // Internal Methods

    public double applyDeadzone(double value, double deadzone, boolean isLeftStick) {
        if (deadzoneType == Deadzone.SQUARE) {
            if (Math.abs(value) < deadzone) {
                return 0d;
            }
            return value;
        } else if (deadzoneType == Deadzone.ROUND) {
            double xAxis = isLeftStick ? (xboxController.getLeftX()) : (xboxController.getRightX());
            double yAxis = isLeftStick ? (xboxController.getLeftY()) : (xboxController.getRightY());
            if (Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2)) < deadzone) {
                return 0d;
            }
            return value;
        }
        return value;
    }

    private double sign(double value) {
        return value < 0 ? -1 : 1;
    }

    public double applyProfile(double value, StickProfile profile) {
        switch (profile) {
            case LINEAR:
                return value;
            case SQUARE:
                return Math.pow(value, 2) * sign(value);
            case FLOOR_SQUARE:
                if (sign(value) == 1) {
                    return Math.max(Math.pow(value, 2), 0.03);
                } else {
                    return Math.min(Math.pow(value, 2) * sign(value), -0.03);
                }
            case FLOOR_LINEAR:
                if (sign(value) == 1) {
                    return Math.max(value, 0.03);
                } else {
                    return Math.min(value * sign(value), -0.03);
                }
            case INTERPOLATED:
                return 0.5 * value + Math.pow(value, 2) * sign(value);
            default:
                return value;
        }
    }

    // Public Methods

    /**
     * Get the X of the left stick
     *
     * @return The X of the left stick
     */
    public double getLeftStickX() {
        return (
            applyProfile(applyDeadzone(xboxController.getLeftX(), leftDeadzone, true), leftProfile) *
            (invertLeftX ? -1 : 1)
        );
    }

    /**
     * Get the Y of the left stick
     *
     * @return The Y of the left stick
     */
    public double getLeftStickY() {
        return (
            applyProfile(applyDeadzone(xboxController.getLeftY(), leftDeadzone, true), leftProfile) *
            (invertLeftY ? -1 : 1)
        );
    }

    /**
     * Get the X of the right stick
     *
     * @return The X of the right stick
     */
    public double getRightStickX() {
        return (
            applyProfile(applyDeadzone(xboxController.getRightX(), rightDeadzone, false), rightProfile) *
            (invertRightX ? -1 : 1)
        );
    }

    /**
     * Get the Y of the right stick
     *
     * @return The Y of the right stick
     */
    public double getRightStickY() {
        return (
            applyProfile(applyDeadzone(xboxController.getRightY(), rightDeadzone, false), rightProfile) *
            (invertRightY ? -1 : 1)
        );
    }

    /**
     * Get the left trigger axis
     *
     * @return The left trigger axis
     */
    public double getLeftTrigger() {
        return xboxController.getLeftTriggerAxis();
    }

    /**
     * Get the right trigger axis
     *
     * @return The right trigger axis
     */
    public double getRightTrigger() {
        return xboxController.getRightTriggerAxis();
    }

    // BUTTONS

    /**
     * Get the A button Trigger
     *
     * @return The A button Trigger
     */
    public Trigger buttonA() {
        return xboxController.a();
    }

    /**
     * Get the B button Trigger
     *
     * @return The B button Trigger
     */
    public Trigger buttonB() {
        return xboxController.b();
    }

    /**
     * Get the X button Trigger
     *
     * @return The X button Trigger
     */
    public Trigger buttonX() {
        return xboxController.x();
    }

    /**
     * Get the Y button Trigger
     *
     * @return The Y button Trigger
     */
    public Trigger buttonY() {
        return xboxController.y();
    }

    /**
     * Get the Left Bumper button Trigger
     *
     * @return The Left Bumper button Trigger
     */
    public Trigger leftBumper() {
        return xboxController.leftBumper();
    }

    /**
     * Get the Right Bumper button Trigger
     *
     * @return The Right Bumper button Trigger
     */
    public Trigger rightBumper() {
        return xboxController.rightBumper();
    }

    /**
     * Get the Back button Trigger
     *
     * @return The Back button Trigger
     */
    public Trigger buttonBack() {
        return xboxController.back();
    }

    /**
     * Get the Start button Trigger
     *
     * @return The Start button Trigger
     */
    public Trigger buttonStart() {
        return xboxController.start();
    }

    /**
     * Get the Dpad Up Trigger
     *
     * @return The Dpad Up Trigger
     */
    public Trigger dpadUp() {
        return xboxController.pov(0);
    }

    /**
     * Get the Dpad Up Trigger
     *
     * @return The Dpad Up Trigger
     */
    public Trigger dpadRight() {
        return xboxController.pov(90);
    }

    /**
     * Get the Dpad Up Trigger
     *
     * @return The Dpad Up Trigger
     */
    public Trigger dpadDown() {
        return xboxController.pov(180);
    }

    /**
     * Get the Dpad Up Trigger
     *
     * @return The Dpad Up Trigger
     */
    public Trigger dpadLeft() {
        return xboxController.pov(270);
    }

    /**
     * Get the internal CommandXboxController
     *
     * @return The internal CommandXboxController
     */
    public CommandXboxController getInternalController() {
        return this.xboxController;
    }

    // ENUMS

    /**
     * The profile to use for the sticks
     *
     * LINEAR: Linear profile
     * SQUARE: Square profile
     * FLOOR_SQUARE: Square profile with a floor
     * FLOOR_LINEAR: Linear profile with a floor
     * INTERPOLATED: Interpolated profile between linear and square
     */
    public enum StickProfile {
        LINEAR,
        SQUARE,
        FLOOR_SQUARE,
        FLOOR_LINEAR,
        INTERPOLATED,
    }

    /**
     * The shape of deadzone to use
     *
     * SQUARE: Square deadzone
     * ROUND: Round deadzone
     */
    public enum Deadzone {
        SQUARE,
        ROUND,
    }
}
