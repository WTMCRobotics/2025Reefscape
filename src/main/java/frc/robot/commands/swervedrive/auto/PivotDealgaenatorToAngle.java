package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DealgaenatorSubsystem;
import frc.robot.subsystems.DealgaenatorSubsystem.DealgaenatorPosition;

public class PivotDealgaenatorToAngle extends Command {

    private final DealgaenatorSubsystem dealgaenatorSubsystem;

    private final PIDController controller = new PIDController(
        Constants.DEALGAENATOR_P,
        Constants.DEALGAENATOR_I,
        Constants.DEALGAENATOR_D
    );

    double targetAngle;

    public PivotDealgaenatorToAngle(
        DealgaenatorSubsystem dealgaenatorSubsystem,
        DealgaenatorPosition dealgaenatorPosition
    ) {
        this.dealgaenatorSubsystem = dealgaenatorSubsystem;
        addRequirements(this.dealgaenatorSubsystem);
        this.targetAngle = dealgaenatorPosition.getDealgaenatorAngleDegrees();
    }

    @Override
    public void initialize() {
        controller.setTolerance(0.025);
        controller.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        System.out.println(
            "executing pivot dealgeanator to angle: " +
            controller.calculate(-dealgaenatorSubsystem.getDealgaenatorAngle())
        );
        dealgaenatorSubsystem.movePivot(-controller.calculate(dealgaenatorSubsystem.getDealgaenatorAngle()));
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ending pivot dealg.");
        dealgaenatorSubsystem.stopPivot();
    }
}
