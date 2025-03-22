package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.swervedrive.auto.PivotDealgaenatorToAngle;
import frc.robot.commands.swervedrive.auto.PivotIntakeToAngle;
import frc.robot.commands.swervedrive.auto.ResetDealgaenator;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class DealgaenatorSubsystem extends SubsystemBase {

    private SparkMax pivotMotor = new SparkMax(5, MotorType.kBrushed);
    private SparkMax pusherMotor = new SparkMax(6, MotorType.kBrushless);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Dealgaenator Encoder", getDealgaenatorAngle());
        SmartDashboard.putNumber("Dealgaenator Absolute Encoder", pivotMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putBoolean("Dealgaenator Reverse Limit Switch", getReverseLimitSwitch());
    }

    public void movePivot(double speed) {
        pivotMotor.set(speed);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    public void movePusher(double speed) {
        pusherMotor.set(speed);
    }

    public void stopPusher() {
        pusherMotor.set(0);
    }

    public void resetEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }

    public double getDealgaenatorAngle() {
        return pivotMotor.getEncoder().getPosition();
    }

    public boolean getReverseLimitSwitch() {
        return pivotMotor.getReverseLimitSwitch().isPressed();
    }

    public Command deployDealgenatorSafely(IntakeSubsystem intake) {
        if (intake.getPivotAngle() < IntakePosition.DEALGAENATING.getPivotAngleRotations() + 6) {
            return Commands.sequence(
                new PivotIntakeToAngle(intake, IntakePosition.GROUND_INTAKE),
                new PivotDealgaenatorToAngle(this, DealgaenatorPosition.DEPLOYED)
            );
        }
        return new PivotDealgaenatorToAngle(this, DealgaenatorPosition.DEPLOYED);
    }

    public Command retractDealgenatorSafely(IntakeSubsystem intake) {
        if (intake.getPivotAngle() < IntakePosition.DEALGAENATING.getPivotAngleRotations() + 6) {
            return Commands.sequence(
                new PivotIntakeToAngle(intake, IntakePosition.GROUND_INTAKE),
                new ResetDealgaenator(this)
            );
        }
        return new ResetDealgaenator(this);
    }

    public enum DealgaenatorPosition {
        DEPLOYED(-1.3923046875);

        double dealgaenatorAngleDegrees;

        DealgaenatorPosition(double angleDegrees) {
            dealgaenatorAngleDegrees = angleDegrees;
        }

        public double getDealgaenatorAngleDegrees() {
            return dealgaenatorAngleDegrees;
        }
    }
}
