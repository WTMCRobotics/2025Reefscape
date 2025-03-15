package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DealgaenatorSubsystem extends SubsystemBase {

    private SparkMax pivotMotor = new SparkMax(5, MotorType.kBrushed);
    private SparkMax pusherMotor = new SparkMax(6, MotorType.kBrushless);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Dealgaenator Encoder", getDealgaenatorAngle());
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

    public enum DealgaenatorPosition {
        DEPLOYED(-1.30);

        double dealgaenatorAngleDegrees;

        DealgaenatorPosition(double angleDegrees) {
            dealgaenatorAngleDegrees = angleDegrees;
        }

        public double getDealgaenatorAngleDegrees() {
            return dealgaenatorAngleDegrees;
        }
    }
}
