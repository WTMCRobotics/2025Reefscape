package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private SparkMax climbMotor = new SparkMax(7, MotorType.kBrushless);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Subsystem Encoder", getAngle());
    }

    public void move(double speed) {
        climbMotor.set(speed);
    }

    public void stop() {
        climbMotor.set(0);
    }

    public double getAngle() {
        return climbMotor.getEncoder().getPosition();
    }

    public void resetEncoder() {
        climbMotor.getEncoder().setPosition(0);
    }

    public boolean isLimit() {
        return climbMotor.getForwardLimitSwitch().isPressed();
    }
}
