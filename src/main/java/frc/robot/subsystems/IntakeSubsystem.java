package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax pivotMotor = new SparkMax(8, MotorType.kBrushless);
    private SparkMax intakeMotor = new SparkMax(9, MotorType.kBrushless);

    // private Ultrasonic distanceSensor = new Ultrasonic(2, 3);/*ultrasonic line 2 PRTC*/

    // private LidarProxy lidar = new LidarProxy(SerialPort.Port.kMXP);

    // ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    // private SparkMax pivotMotor = null;
    // private SparkMax intakeMotor = null;

    DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(1);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Subsystem Pivot Encoder", getPivotAngle());
        // SmartDashboard.putNumber("Intake Color Sensor Proximity", colorSensor.getProximity());
        // SmartDashboard.putString("Intake Color Sensor Color", colorSensor.getColor().toString());
        // SmartDashboard.putNumber("Intake Color Sensor IR", colorSensor.getIR());
        // SmartDashboard.putNumber("Intake Ultrasonic", distanceSensor.getRangeMM());
    }

    public void movePivot(double speed) {
        pivotMotor.set(speed);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    public double getPivotAngle() {
        // return pivotMotor.getEncoder().getPosition();
        return intakeEncoder.get();
    }

    public void resetEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }

    public enum IntakePosition {
        STARTING_POSITION(0),
        DEALGAENATING(.366),
        CLIMBING(.366),
        SCORING(.591),
        CORAL_SNAG(.567),
        GROUND_INTAKE(.651);

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

    public Command spinPivot(double speed) {
        return Commands.runOnce(
            () -> {
                movePivot(speed);
            },
            this
        );
    }

    public double getMotorSpeed() {
        return pivotMotor.get();
    }

    public Command moveIntakeUptest() {
        return new Command() {
            double power;

            @Override
            public void initialize() {
                power = 0;
            }

            @Override
            public void execute() {
                power -= 0.002;
                System.out.println("Current power: " + power);
                pivotMotor.set(power);
            }

            @Override
            public boolean isFinished() {
                return power >= 0.75;
            }

            @Override
            public void end(boolean interrupted) {
                pivotMotor.set(0);
            }
        };
    }
}
