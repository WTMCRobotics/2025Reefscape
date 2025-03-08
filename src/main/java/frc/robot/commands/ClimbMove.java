package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbMove extends Command {

    private final ClimbSubsystem climbSubsystem;

    private double speed;

    public ClimbMove(ClimbSubsystem climbSubsystem, double speed) {
        this.climbSubsystem = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.move(speed);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }
}
