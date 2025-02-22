package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

class Controller {
    private final CommandXboxController xboxController;

    private StickProfile leftProfile = StickProfile.LINEAR;
    private StickProfile rightProfile = StickProfile.LINEAR;

    private double leftDeadzone = 0.1;
    private double rightDeadzone = 0.1;

    /*
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


    /*
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


    /*
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

    /*
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

    /*
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

    // Internal Methods

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
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

    /*
     * Get the X of the left stick
     * 
     * @return The X of the left stick
     */
    public double getLeftStickX() {
        return applyProfile(applyDeadzone(xboxController.getLeftX(), leftDeadzone), leftProfile);
    }

    /*
     * Get the Y of the left stick
     * 
     * @return The Y of the left stick
     */
    public double getLeftStickY() {
        return applyProfile(applyDeadzone(xboxController.getLeftY(), leftDeadzone), leftProfile);
    }

    /*
     * Get the X of the right stick
     * 
     * @return The X of the right stick
     */
    public double getRightStickX() {
        return applyProfile(applyDeadzone(xboxController.getRightX(), rightDeadzone), rightProfile);
    }
    
    /*
     * Get the Y of the right stick
     * 
     * @return The Y of the right stick
     */
    public double getRightStickY() {
        return applyProfile(applyDeadzone(xboxController.getRightY(), rightDeadzone), rightProfile);
    }

    /*
     * Get the left trigger axis
     * 
     * @return The left trigger axis
     */
    public double getLeftTrigger() {
        return xboxController.getLeftTriggerAxis();
    }

    /*
     * Get the right trigger axis
     * 
     * @return The right trigger axis
     */
    public double getRightTrigger() {
        return xboxController.getRightTriggerAxis();
    }

    //TODO: Add button methods HELP???


    // ENUMS

    enum StickProfile {
        LINEAR,
        SQUARE
    }

    enum Deadzone {
        SQUARE,
        ROUND
    }
}