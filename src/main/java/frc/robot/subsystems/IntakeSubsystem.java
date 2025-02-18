package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // TODO: make the IDs good
    SparkMax pivotMotor = new SparkMax(8, MotorType.kBrushless);
    SparkMax intakeMotor = new SparkMax(9, MotorType.kBrushless);

    @Override
    public void periodic() {
        
    }
    public void movePivot(double speed) {
        pivotMotor.set(speed);
    }
    
    public void stopPivot(double speed) {
        pivotMotor.set(0);
    }
    public void dealgaenatePivot() {}
}
