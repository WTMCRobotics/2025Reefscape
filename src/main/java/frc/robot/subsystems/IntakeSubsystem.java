package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // TODO: make the IDs good
    private SparkMax pivotMotor = new SparkMax(8, MotorType.kBrushless);
    private SparkMax intakeMotor = new SparkMax(9, MotorType.kBrushless);

    @Override
    public void periodic() {

    }

    public void movePivot(double speed) {
        pivotMotor.set(speed);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    public double getPivotAngle() {
        return pivotMotor.getEncoder().getPosition();
    }

    public void resetEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }

    public enum IntakePosition {
        GROUND_INTAKE(45.2),
        STARTING_POSITION(1),
        DEALGAENATING(2),
        CLIMBING(3),
        SCORING(4),
        CORAL_SNAG(5);

        double pivotAngleDegrees;

        IntakePosition(double angleDegrees) {
            pivotAngleDegrees = angleDegrees;
        }

        public double getPivotAngleDegrees(){
            return pivotAngleDegrees;
        }
    }

    public boolean getReverseLimit(){
        return pivotMotor.getReverseLimitSwitch().isPressed();
    }
}
