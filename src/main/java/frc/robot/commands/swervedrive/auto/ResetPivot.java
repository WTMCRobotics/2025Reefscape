package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class ResetPivot extends Command {

  private final IntakeSubsystem intakeSubsystem;

  private final PIDController controller = new PIDController(Constants.PIVOT_P, Constants.PIVOT_I, Constants.PIVOT_D);

  double targetAngle;

  public ResetPivot(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.movePivot(-0.2);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public boolean isFinished() {
    return intakeSubsystem.getReverseLimit();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopPivot();
    intakeSubsystem.resetEncoder();
  }
}
