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
            //If going to dealgaenating, try not to slam into robot too hard
            if (Math.abs(targetAngle - IntakePosition.DEALGAENATING.getPivotAngleRotations()) < 0.01) {
                controller.setP(Constants.INTAKE_GOING_UP_TO_DEALGEANATE);
            }
            finishedDownReset = true;
        }

        controller.setTolerance(0.28);
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
        if (controller.atSetpoint() && !finishedDownReset) {
            controller.setP(Constants.INTAKE_PIVOT_UP_P);
            controller.setSetpoint(targetAngle);
            finishedDownReset = true;
            System.out.println("Using " + controller.getP());
        }
        intakeSubsystem.movePivot(calcValue);
        System.out.println("going to " + controller.getSetpoint() + " at speed of " + calcValue);
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
