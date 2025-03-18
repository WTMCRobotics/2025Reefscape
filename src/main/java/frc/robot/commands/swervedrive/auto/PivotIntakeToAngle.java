package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class PivotIntakeToAngle extends Command {

    private final IntakeSubsystem intakeSubsystem;
    boolean finishedDownReset;

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
        finishedDownReset = false;
        if (intakeSubsystem.getPivotAngle() < targetAngle) {
            controller.setP(Constants.INTAKE_PIVOT_DOWN_P);
        } else {
            controller.setP(Constants.INTAKE_PIVOT_UP_P);
            finishedDownReset = true;
        }

        controller.setTolerance(0.25);
        // controller.setSetpoint(targetAngle);
        if (finishedDownReset) {
            controller.setSetpoint(targetAngle);
        } else {
            controller.setSetpoint(IntakePosition.GROUND_INTAKE.getPivotAngleRotations());
        }
    }

    @Override
    public void execute() {
        double calcValue = controller.calculate(intakeSubsystem.getPivotAngle());
        intakeSubsystem.movePivot(calcValue);
        if (controller.atSetpoint() && !finishedDownReset) {
            controller.setSetpoint(targetAngle);
            controller.setP(Constants.INTAKE_PIVOT_UP_P);
            finishedDownReset = true;
        }
        System.out.println(calcValue);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint() && finishedDownReset;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopPivot();
    }
}
