package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GuitarController extends Controller {

    public GuitarController(int port) {
        super(port);
    }

    // BUTTONS

    /**
     * Get the Trigger for the top green fret
     *
     * @return Top green fret Trigger
     */
    public Trigger fretGreen() {
        return xboxController.a();
    }

    /**
     * Get the Trigger for the top red fret
     *
     * @return Top red fret Trigger
     */
    public Trigger fretRed() {
        return xboxController.b();
    }

    /**
     * Get the Trigger for the top yellow fret
     *
     * @return Top yellow fret Trigger
     */
    public Trigger fretYellow() {
        return xboxController.y();
    }

    /**
     * Get the Trigger for the top blue fret
     *
     * @return Top blue fret Trigger
     */
    public Trigger fretBlue() {
        return xboxController.x();
    }

    /**
     * Get the Trigger for the top orange fret
     *
     * @return Top orange fret Trigger
     */
    public Trigger fretOrange() {
        return xboxController.leftBumper();
    }

    /**
     * Get the Trigger for the lower frets
     *
     * @return Lower fretboard trigger
     */
    public Trigger fretLower() {
        return xboxController.rightBumper();
    }

    /**
     * Get the Trigger for struming up
     *
     * @return Strum up Trigger
     */
    public Trigger strumUp() {
        return xboxController.pov(0);
    }

    /**
     * Get the Trigger for struming down
     * @return Strum down Trigger
     */
    public Trigger strumDown() {
        return xboxController.pov(180);
    }
}
