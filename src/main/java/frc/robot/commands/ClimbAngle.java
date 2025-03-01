package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbAngle extends Command {

  private final ClimbSubsystem climbSubsystem;

  private final PIDController controller = new PIDController(Constants.PIVOT_P, Constants.PIVOT_I, Constants.PIVOT_D);

  double targetAngle;

  public ClimbAngle(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(this.climbSubsystem);
    this.targetAngle = 4;


  }

  @Override
  public void initialize() {
    controller.setTolerance(1);
    controller.setSetpoint(targetAngle * Constants.AngleMotorConversion);
  }

  @Override
  public void execute() {
    climbSubsystem.move(controller.calculate(climbSubsystem.getAngle()));
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }
}
