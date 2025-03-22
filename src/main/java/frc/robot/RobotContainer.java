// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbAngle;
import frc.robot.commands.ClimbAngle.ClimbPosition;
import frc.robot.commands.ClimbMove;
import frc.robot.commands.ClimbReset;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.swervedrive.auto.PivotDealgaenatorToAngle;
import frc.robot.commands.swervedrive.auto.PivotIntakeToAngle;
import frc.robot.commands.swervedrive.auto.PivotIntakeToAngleWithPIDF;
import frc.robot.commands.swervedrive.auto.ResetDealgaenator;
import frc.robot.commands.swervedrive.auto.ResetGyro;
import frc.robot.commands.swervedrive.auto.ResetPivot;
import frc.robot.commands.swervedrive.auto.SpinDealgaenator;
import frc.robot.commands.swervedrive.auto.SpinIntake;
import frc.robot.controller.Controller;
import frc.robot.controller.Controller.StickProfile;
import frc.robot.controller.GuitarController;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DealgaenatorSubsystem;
import frc.robot.subsystems.DealgaenatorSubsystem.DealgaenatorPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final Controller driverController = new Controller(0)
        .invertLeftY()
        .setLeftDeadzone(0d)
        .setLeftProfile(StickProfile.LINEAR);
    public final GuitarController codriverController = new GuitarController(1);

    // The robot's subsystems and commands are defined here...
    public final SwerveSubsystem drivebase;
    private IntakeSubsystem intake;
    private DealgaenatorSubsystem dealgaenator;
    public ClimbSubsystem climb;

    {
        if (Robot.isReal()) {
            drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_non_simulation"));
            // drivebase.getSwerveDrive().setCosineCompensator(true);
            drivebase.getSwerveDrive().setHeadingCorrection(true);
            drivebase.getSwerveDrive().setAngularVelocityCompensation(true, true, -0.1);
            drivebase.getSwerveDrive().setChassisDiscretization(true, true, 0.02);
            drivebase.getSwerveDrive().setAutoCenteringModules(false);
            intake = new IntakeSubsystem();
            climb = new ClimbSubsystem();
            dealgaenator = new DealgaenatorSubsystem();
        } else {
            drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_simulation"));
            drivebase.getSwerveDrive().setHeadingCorrection(false); // Heading correction should only be used while
            // controlling the robot via angle.
            drivebase.getSwerveDrive().setCosineCompensator(false);
            intake = new IntakeSubsystem();
            climb = new ClimbSubsystem();
            dealgaenator = new DealgaenatorSubsystem();
        }
    }

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        //Remember, on the robot, positive X is forward, Postive Y is to the left
        () -> {
            return driverController.getLeftStickY();
        },
        () -> {
            return driverController.getLeftStickX() * -1;
        }
    )
        .withControllerRotationAxis(() -> driverController.getRightStickX() * -1)
        .deadband(OperatorConstants.DEADBAND)
        // .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity
        .copy()
        .withControllerHeadingAxis(driverController::getRightStickX, driverController::getRightStickY)
        .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity
        .copy()
        .robotRelative(true)
        .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> -driverController.getLeftStickY(),
        () -> -driverController.getLeftStickX()
    )
        .withControllerRotationAxis(driverController::getRightStickX)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
        .copy()
        .withControllerHeadingAxis(
            () -> Math.sin(driverController.getLeftTrigger() * Math.PI) * (Math.PI * 2),
            () -> Math.cos(driverController.getLeftTrigger() * Math.PI) * (Math.PI * 2)
        )
        .headingWhile(true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("Reset", resetRobot());
        NamedCommands.registerCommand("SetIntake", resetClimbAndMoveIntakeUp());
        NamedCommands.registerCommand(
            "SetIntake: Coral Snag",
            new PivotIntakeToAngle(intake, IntakePosition.CORAL_SNAG)
        );
        NamedCommands.registerCommand("Intake", new SpinIntake(intake, Constants.INTAKE_SPEED));
        // NamedCommands.registerCommand("Drop Coral", Commands.none());
        NamedCommands.registerCommand("Drop Coral", new ClimbAngle(climb, ClimbPosition.DEPOSIT_CORAL_ZEROED));
        SmartDashboard.putData("Reset Encoders", new ResetEncoders(intake, climb, dealgaenator));
        SmartDashboard.putData("Reset Intake", new ResetPivot(intake));
        SmartDashboard.putData("Reset Climb", new ClimbReset(climb));
        SmartDashboard.putData("Reset Dealgaenator", new ResetDealgaenator(dealgaenator));
        SmartDashboard.putData("Reset robot", resetRobot());
        SmartDashboard.putData("Reset gyro", new ResetGyro(drivebase).ignoringDisable(true));

        SmartDashboard.putData("Test moving intake up slowy", intake.moveIntakeUptest());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

        //We could do the commands .repeatatly
        // codriverController.fretOrange().onTrue(new PivotIntakeToAngle(intake, IntakePosition.GROUND_INTAKE).andThen(new PivotIntakeToAngle(intake, IntakePosition.GROUND_INTAKE)));
        // codriverController.fretYellow().onTrue(new PivotIntakeToAngle(intake, IntakePosition.SCORING).andThen(new PivotIntakeToAngle(intake, IntakePosition.SCORING)));
        // codriverController.fretBlue().onTrue(new PivotIntakeToAngle(intake, IntakePosition.CORAL_SNAG).andThen(new PivotIntakeToAngle(intake, IntakePosition.CORAL_SNAG)));
        // codriverController.fretRed().onTrue(new PivotIntakeToAngle(intake, IntakePosition.CLIMBING).andThen(new PivotIntakeToAngle(intake, IntakePosition.CLIMBING)));
        // codriverController
        //     .fretGreen()
        //     .onTrue(new PivotIntakeToAngle(intake, IntakePosition.DEALGAENATING));

        codriverController.fretOrange().onTrue(new PivotIntakeToAngleWithPIDF(intake, IntakePosition.GROUND_INTAKE));
        codriverController.fretYellow().onTrue(new PivotIntakeToAngleWithPIDF(intake, IntakePosition.SCORING));
        codriverController.fretBlue().onTrue(new PivotIntakeToAngleWithPIDF(intake, IntakePosition.CORAL_SNAG));
        codriverController.fretRed().onTrue(new PivotIntakeToAngleWithPIDF(intake, IntakePosition.CLIMBING));
        codriverController.fretGreen().onTrue(new PivotIntakeToAngleWithPIDF(intake, IntakePosition.DEALGAENATING));

        codriverController.strumUp().whileTrue(new SpinDealgaenator(dealgaenator, -0.5));
        codriverController.strumDown().whileTrue(new SpinDealgaenator(dealgaenator, 0.5));

        codriverController.buttonStart().whileTrue(new ClimbMove(climb, 1.0));
        codriverController.buttonBack().whileTrue(new ClimbMove(climb, -1.0));

        codriverController.dpadLeft().onTrue(new ClimbAngle(climb, ClimbPosition.DEPLOY_CLIMB));

        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        } else {
            // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        if (Robot.isSimulation()) {
            driverController
                .buttonStart()
                .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverController.buttonA().whileTrue(drivebase.sysIdDriveMotorCommand());
        }
        if (DriverStation.isTest()) {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            driverController.buttonX().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverController.buttonY().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
            driverController.buttonStart().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverController.buttonBack().whileTrue(drivebase.centerModulesCommand());
            driverController.leftBumper().onTrue(Commands.none());
            driverController.rightBumper().onTrue(Commands.none());
        } else {
            // driverController.buttonX().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            // driverController.buttonA().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            // driverController.buttonX().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
            // driverController
            //     .buttonB()
            // .whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
            driverController.buttonX().onTrue(intake.spinPivot(-0.2));
            driverController.buttonY().onTrue(intake.spinPivot(0.2));
            driverController.buttonX().onFalse(intake.spinPivot(-0.0));
            driverController.buttonY().onFalse(intake.spinPivot(0.0));
            driverController.buttonStart().whileTrue(Commands.none());
            driverController.buttonBack().whileTrue(Commands.none());
            // in
            driverController.leftBumper().whileTrue(new SpinIntake(intake, Constants.INTAKE_SPEED));
            // out
            driverController.rightBumper().whileTrue(new SpinIntake(intake, -Constants.INTAKE_SPEED));

            // driverController.dpadUp().onTrue(dealgaenator.deployDealgenatorSafely(intake));
            // driverController.dpadDown().onTrue(dealgaenator.retractDealgenatorSafely(intake));
            driverController.dpadUp().onTrue(new PivotDealgaenatorToAngle(dealgaenator, DealgaenatorPosition.DEPLOYED));
            driverController.dpadDown().onTrue(new ResetDealgaenator(dealgaenator));
        }
    }

    public void setOffset(double offset) {
        drivebase.getSwerveDrive().setGyroOffset(new Rotation3d(0, 0, offset));
    }

    public void resetIntakePivot() {
        new ResetPivot(intake).schedule();
    }

    public Command resetRobot() {
        return Commands.sequence(
            new ResetPivot(intake),
            new PivotIntakeToAngle(intake, IntakePosition.SCORING).withTimeout(2),
            Commands.parallel(new ResetDealgaenator(dealgaenator))
            // new ResetDealgaenator(dealgaenator),
            // new PivotDealgaenatorToAngle(dealgaenator, DealgaenatorPosition.DEPLOYED),
        );
    }

    public Command resetClimbAndMoveIntakeUp() {
        return Commands.sequence(
            new ClimbAngle(climb, ClimbPosition.ZERO_POSITION),
            new PivotIntakeToAngle(intake, IntakePosition.DEALGAENATING)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(String autoName) {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand(autoName);
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public void sendTelementary() {
        SmartDashboard.putData("gyro", (Sendable) drivebase.getSwerveDrive().getGyro().getIMU());
        if (Robot.isSimulation()) {
            Logger.recordOutput("wheelStates", drivebase.getSwerveDrive().getStates());
        }
    }

    public void resetSimulation() {
        if (Robot.isReal()) return;

        drivebase
            .getSwerveDrive()
            .getMapleSimDrive()
            .get()
            .setSimulationWorldPose(new Pose2d(9.3, 2, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Robot.isReal()) {
            return;
        }

        Logger.recordOutput(
            "FieldSimulation/RobotPosition",
            drivebase.getSwerveDrive().getMapleSimDrive().get().getSimulatedDriveTrainPose()
        );
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }
}
