package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class PivotIntakeToAngle extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final PIDController controller = new PIDController(
        Constants.INTAKE_PIVOT_DOWN_P,
        Constants.INTAKE_PIVOT_I,
        Constants.INTAKE_PIVOT_D
    );

    double targetAngle;

    public PivotIntakeToAngle(IntakeSubsystem intakeSubsystem, IntakePosition intakePosition) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
        this.targetAngle = intakePosition.getPivotAngleRotations();
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.getPivotAngle() < targetAngle) {
            controller.setP(Constants.INTAKE_PIVOT_DOWN_P);
        } else {
            controller.setP(Constants.INTAKE_PIVOT_UP_P);
        }

        controller.setTolerance(0.25);
        controller.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        double calcValue = controller.calculate(intakeSubsystem.getPivotAngle());
        intakeSubsystem.movePivot(calcValue);
        System.out.println(calcValue);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopPivot();
    }
}
