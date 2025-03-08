package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LidarProxy;
import frc.robot.subsystems.DealgaenatorSubsystem;

public class SpinDealgaenator extends Command {

    private DealgaenatorSubsystem dealgaenatorSubsystem;
    private double speed;

    public SpinDealgaenator(DealgaenatorSubsystem dealgaenatorSubsystem, double speed) {
        this.dealgaenatorSubsystem = dealgaenatorSubsystem;
        this.speed = speed;
        addRequirements(dealgaenatorSubsystem);
    }

    @Override
    public void initialize() {
        // if(speed < 0 && )
        dealgaenatorSubsystem.movePusher(speed);
    }

    @Override
    public void end(boolean interrupted) {
        dealgaenatorSubsystem.stopPusher();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
