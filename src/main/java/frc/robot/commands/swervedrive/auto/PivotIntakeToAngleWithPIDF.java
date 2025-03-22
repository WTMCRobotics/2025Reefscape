package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class PivotIntakeToAngleWithPIDF extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final ProfiledPIDController controller = new ProfiledPIDController(
        Constants.INTAKE_PIVOT_DOWN_P,
        Constants.INTAKE_PIVOT_I,
        Constants.INTAKE_PIVOT_D,
        new Constraints(1, 0.5)
    );

    double targetAngle;
    int debounce;

    public PivotIntakeToAngleWithPIDF(IntakeSubsystem intakeSubsystem, IntakePosition intakePosition) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
        this.targetAngle = intakePosition.getPivotAngleRotations();
        SmartDashboard.putData("Intake PID", controller);
    }

    @Override
    public void initialize() {
        debounce = 0;
        if (
            Double.compare(targetAngle, IntakePosition.SCORING.getPivotAngleRotations()) == 0 ||
            Double.compare(targetAngle, IntakePosition.CORAL_SNAG.getPivotAngleRotations()) == 0
        ) {
            targetAngle -= 0.01;
        }

        // else {
        //     controller.setP(Constants.INTAKE_PIVOT_UP_P);
        //If going to dealgaenating, try not to slam into robot too hard
        // if (Double.compare(targetAngle, IntakePosition.DEALGAENATING.getPivotAngleRotations()) == 0) {
        //     controller.setP(Constants.INTAKE_GOING_UP_TO_DEALGEANATE);
        // }
        // }

        // controller.setP(Constants.INTAKE_PIVOT_DOWN_P);

        controller.setTolerance(0.01);

        controller.reset(intakeSubsystem.getPivotAngle());

        controller.setGoal(new State(targetAngle, 0.01));

        System.out.println("START GOING TO " + targetAngle + " with p " + controller.getP());
    }

    @Override
    public void execute() {
        controller.setGoal(targetAngle);
        double calcValue = controller.calculate(intakeSubsystem.getPivotAngle());
        calcValue -= 0.05;
        System.out.println(
            "going to " +
            controller.getGoal().position +
            " at speed of " +
            calcValue +
            " with vel " +
            controller.getSetpoint().velocity
        );
        intakeSubsystem.movePivot(calcValue);
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal() && controller.getSetpoint().velocity <= 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopPivot();
    }
}
