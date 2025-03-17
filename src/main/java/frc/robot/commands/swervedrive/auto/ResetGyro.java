package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ResetGyro extends Command {

    private SwerveSubsystem drivebaseSubsystem;

    public ResetGyro(SwerveSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void initialize() {
        drivebaseSubsystem.zeroGyroWithAlliance();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
