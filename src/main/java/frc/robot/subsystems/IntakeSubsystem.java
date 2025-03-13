package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LidarProxy;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax pivotMotor = new SparkMax(8, MotorType.kBrushless);
    private SparkMax intakeMotor = new SparkMax(9, MotorType.kBrushless);

    private Ultrasonic distanceSensor = new Ultrasonic(2, 3);/*ultrasonic line 2 PRTC*/

    // private LidarProxy lidar = new LidarProxy(SerialPort.Port.kMXP);

    // ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    // private SparkMax pivotMotor = null;
    // private SparkMax intakeMotor = null;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Subsystem Pivot Encoder", getPivotAngle());
        // SmartDashboard.putNumber("Intake Color Sensor Proximity", colorSensor.getProximity());
        // SmartDashboard.putString("Intake Color Sensor Color", colorSensor.getColor().toString());
        // SmartDashboard.putNumber("Intake Color Sensor IR", colorSensor.getIR());
        SmartDashboard.putNumber("Intake Ultrasonic", distanceSensor.getRangeMM());
        SmartDashboard.putNumber("check", 5);
    }

    // public boolean isLoaded() {
    //     return 0.5 <= distanceSensor.getRangeInches() && distanceSensor.getRangeInches() <= 7.0;
    // }

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
        STARTING_POSITION(0),
        DEALGAENATING(2),
        CLIMBING(3),
        CORAL_SNAG(22.54),
        SCORING(7.5),
        GROUND_INTAKE(28.76);

        double pivotAngleRotations;

        IntakePosition(double angleRotations) {
            pivotAngleRotations = angleRotations;
        }

        public double getPivotAngleRotations() {
            return pivotAngleRotations;
        }
    }

    public boolean getReverseLimit() {
        return pivotMotor.getReverseLimitSwitch().isPressed();
    }

    public void spinIntake(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
    }
}
