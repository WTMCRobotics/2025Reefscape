package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DealgaenatorSubsystem extends SubsystemBase {
    // TODO: correct the motor ids
    private SparkMax pivotMotor = new SparkMax(8, MotorType.kBrushless);
    private SparkMax pusherMotor = new SparkMax(9, MotorType.kBrushless);

    @Override
    public void periodic() {

    }
    public void movePivot(double speed) {
        pivotMotor.set(speed);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    public void movePusher(double speed){
        pusherMotor.set(speed);
    }

    public void stopPusher() {
        pusherMotor.set(0);
    }

    public void resetEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }


    public double getPivotEncoder() {
        return pivotMotor.getEncoder().getPosition();
    }

    public boolean getReverseLimitSwitch() {
        return pivotMotor.getReverseLimitSwitch().isPressed();
    }
}
