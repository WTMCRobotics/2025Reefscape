package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class Controller {
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
        this.xboxController = new CommandXboxController(0);
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
     * Set the left stick to be inverted on the Y axis
     * Default is false
     * 
     * @param invert Whether or not to invert the Y axis
     * 
     * @return The Controller object
     */
    public Controller setInvertLeftY(boolean invert) {
        this.invertLeftY = invert;
        return this;
    }

    /**
     * Set the right stick to be inverted on the Y axis
     * Default is false
     * 
     * @param invert Whether or not to invert the Y axis
     * 
     * @return The Controller object
     */
    public Controller setInvertRightY(boolean invert) {
        this.invertRightY = invert;
        return this;
    }

    /**
     * Set the left stick to be inverted on the X axis
     * Default is false
     * 
     * @param invert Whether or not to invert the X axis
     * 
     * @return The Controller object
     */
    public Controller setInvertLeftX(boolean invert) {
        this.invertLeftX = invert;
        return this;
    }

    /**
     * Set the right stick to be inverted on the X axis
     * Default is false
     * 
     * @param invert Whether or not to invert the X axis
     * 
     * @return The Controller object
     */
    public Controller setInvertRightX(boolean invert) {
        this.invertRightX = invert;
        return this;
    }

    // Internal Methods

    public double applyDeadzone(double value, double deadzone, boolean left) {
        if (deadzoneType == Deadzone.SQUARE) {
            if (Math.abs(value) < deadzone) {
                return 0d;
            }
            return value;
        } else if (deadzoneType == Deadzone.ROUND) {
            if (Math.sqrt(Math.pow(left ? (xboxController.getLeftX()) : (xboxController.getRightX()), 2) + Math.pow(left ? (xboxController.getLeftY()) : (xboxController.getRightY()), 2)) < deadzone) {
                return 0d;
            }
            return value;
        }
        return value;
    }

    private double sign(double value) {
        return value < 0 ? -1 : 1;
    }

    private double applyProfile(double value, StickProfile profile) {
        switch (profile) {
            case LINEAR:
                return value;
            case SQUARE:
                return Math.pow(value, 2) * sign(value);
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
        return applyProfile(applyDeadzone(xboxController.getLeftX(), leftDeadzone, true), leftProfile) * (invertLeftX ? -1 : 1);
    }

    /**
     * Get the Y of the left stick
     * 
     * @return The Y of the left stick
     */
    public double getLeftStickY() {
        return applyProfile(applyDeadzone(xboxController.getLeftY(), leftDeadzone, true), leftProfile) * (invertLeftY ? -1 : 1);
    }

    /**
     * Get the X of the right stick
     * 
     * @return The X of the right stick
     */
    public double getRightStickX() {
        return applyProfile(applyDeadzone(xboxController.getRightX(), rightDeadzone, false), rightProfile) * (invertRightX ? -1 : 1);
    }
    
    /**
     * Get the Y of the right stick
     * 
     * @return The Y of the right stick
     */
    public double getRightStickY() {
        return applyProfile(applyDeadzone(xboxController.getRightY(), rightDeadzone, false), rightProfile) * (invertRightY ? -1 : 1);
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
    public Trigger back() {
        return xboxController.back();
    }

    /**
     * Get the Start button Trigger
     * 
     * @return The Start button Trigger
     */
    public Trigger start() {
        return xboxController.start();
    }


    // ENUMS

    /**
     * The profile to use for the sticks
     * 
     * LINEAR: Linear profile
     * SQUARE: Square profile
     */
    enum StickProfile {
        LINEAR,
        SQUARE
    }

    /**
     * The shape of deadzone to use
     * 
     * SQUARE: Square deadzone
     * ROUND: Round deadzone
     */
    enum Deadzone {
        SQUARE,
        ROUND
    }
}