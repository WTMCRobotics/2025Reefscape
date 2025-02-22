package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GuitarController extends Controller {
    
    public GuitarController(int port) {
        super(port);
    }

    // BUTTONS

    public Trigger fretGreen() {
        return xboxController.a();
    }

    public Trigger fretRed() {
        return xboxController.b();
    }

    public Trigger fretYellow() {
        return xboxController.y();
    }

    public Trigger fretBlue() {
        return xboxController.x();
    }

    public Trigger fretOrange() {
        return xboxController.leftBumper();
    }

    public Trigger strumUp() {
        return xboxController.pov(0);
    }

    public Trigger strumDown() {
        return xboxController.pov(180);
    }

    public Trigger buttonBack() {
        return xboxController.back();
    }

    public Trigger buttonStart() {
        return xboxController.start();
    }
}
