package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax climbMotor = new SparkMax(0, MotorType.kBrushless);

    @Override
    public void periodic() {

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
        return climbMotor.getReverseLimitSwitch().isPressed();
    }
}
