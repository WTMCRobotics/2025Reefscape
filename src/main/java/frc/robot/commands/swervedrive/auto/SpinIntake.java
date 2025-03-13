package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinIntake extends Command {

    private IntakeSubsystem intakeSubsystem;
    private double speed;

    public SpinIntake(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.spinIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.spinIntake(0);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        // return intakeSubsystem.isLoaded() && speed < 0;
        return false;
    }
}
