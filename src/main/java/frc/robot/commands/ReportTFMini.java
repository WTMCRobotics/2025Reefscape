package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TFMini;

/**
 *
 */
public class ReportTFMini extends Command {

    private TFMini lidar;

    public ReportTFMini(TFMini lidar) {
        addRequirements(lidar);
        this.lidar = lidar;
    }

    // Called just before this Command runs the first time
    public void initialize() {}

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        lidar.report();
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {}

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {}
}
