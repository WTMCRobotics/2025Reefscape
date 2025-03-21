package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DealgaenatorSubsystem;

public class ResetDealgaenator extends Command {

    private final DealgaenatorSubsystem dealgaenatorSubsystem;

    public ResetDealgaenator(DealgaenatorSubsystem dealgaenatorSubsystem) {
        this.dealgaenatorSubsystem = dealgaenatorSubsystem;
        addRequirements(this.dealgaenatorSubsystem);
    }

    @Override
    public void initialize() {
        dealgaenatorSubsystem.movePivot(-0.3);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return dealgaenatorSubsystem.getReverseLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        dealgaenatorSubsystem.stopPivot();
        dealgaenatorSubsystem.resetEncoder();
    }
}
