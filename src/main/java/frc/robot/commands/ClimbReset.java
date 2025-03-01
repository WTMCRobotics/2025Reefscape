package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbReset extends Command {

  private final ClimbSubsystem climbSubsystem;

  private final PIDController controller = new PIDController(Constants.PIVOT_P, Constants.PIVOT_I, Constants.PIVOT_D);

  double targetAngle;

  public ClimbReset(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    
  }

  @Override
  public void initialize() {
    climbSubsystem.move(-0.2);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isLimit();
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
    climbSubsystem.resetEncoder();
  }
}
