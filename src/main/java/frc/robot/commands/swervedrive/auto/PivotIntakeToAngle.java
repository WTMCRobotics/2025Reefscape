package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotIntakeToAngle extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    private final PIDController controller;

    public PivotIntakeToAngle(IntakeSubsystem intakeSubsystem, double targetAngle)
    {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);

        double intakeAngle = intakeSubsystem.getIntakeAngle();

        controller = new PIDController(Constants.PIVOT_P, Constants.PIVOT_I, Constants.PIVOT_D);
        controller.setTolerance(1);
        controller.setSetpoint(targetAngle*Constants.AngleMotorConversion);

        


    }




@Override
public void initialize()
{

}


  @Override
  public void execute()
  {
    intakeSubsystem.movePivot(controller.calculate(intakeSubsystem.getIntakeAngle(),0 ));
  }

@Override
public boolean isFinished()
{
  return controller.atSetpoint();
}


@Override
public void end(boolean interrupted)
{
    intakeSubsystem.stopPivot();
}
}
