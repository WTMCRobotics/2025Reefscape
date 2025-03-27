// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbAngle;
import frc.robot.commands.ClimbAngle.ClimbPosition;
import frc.robot.commands.ClimbMove;
import frc.robot.commands.ClimbReset;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.swervedrive.auto.PivotDealgaenatorToAngle;
import frc.robot.commands.swervedrive.auto.PivotIntakeToAngle;
import frc.robot.commands.swervedrive.auto.PivotIntakeToAngleWithPIDF;
import frc.robot.commands.swervedrive.auto.ResetDealgaenator;
import frc.robot.commands.swervedrive.auto.ResetPivot;
import frc.robot.commands.swervedrive.auto.SpinDealgaenator;
import frc.robot.commands.swervedrive.auto.SpinIntake;
import frc.robot.controller.Controller;
import frc.robot.controller.Controller.StickProfile;
import frc.robot.controller.GuitarController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DealgaenatorSubsystem;
import frc.robot.subsystems.DealgaenatorSubsystem.DealgaenatorPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

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
        .setLeftDeadzone(0d)
        .setLeftProfile(StickProfile.SQUARE);
    public final GuitarController codriverController = new GuitarController(1);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem intake;
    private DealgaenatorSubsystem dealgaenator;
    public ClimbSubsystem climb;

    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        intake = new IntakeSubsystem();
        climb = new ClimbSubsystem();
        dealgaenator = new DealgaenatorSubsystem();
        // Configure the trigger bindings
        configureBindings();

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

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
        // SmartDashboard.putData("Reset gyro", new ResetGyro(drivebase).ignoringDisable(true));

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
        SlewRateLimiter XaxisLimiter = new SlewRateLimiter(5);
        SlewRateLimiter YAxisLimiter = new SlewRateLimiter(5);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-driverController.getLeftStickY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driverController.getLeftStickX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightStickX() * MaxAngularRate)
                // Drive counterclockwise with negative X (left)
            )
        );

        // STOP TIPPING BUT BROKEN SYT
        // drivetrain.setDefaultCommand(
        //         // Drivetrain will execute this command periodically
        //         drivetrain.applyRequest(() -> drive.withVelocityX(YAxisLimiter.calculate(
        //                 -driverController.getLeftStickY() * MaxSpeed) // Drive forward with negative Y (forward)
        //                 .withVelocityY(XaxisLimiter.calculate(
        //                         -driverController.getLeftStickX() * MaxSpeed) // Drive left with negative X (left)
        //                         .withRotationalRate(-driverController.getRightStickX() * MaxAngularRate) // Drive counterclockwise
        //                                                                                     // with negative X (left)
        //     )
        //                 )));

        driverController.buttonA().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController
            .buttonB()
            .whileTrue(
                drivetrain.applyRequest(() ->
                    point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftStickY(), -driverController.getLeftStickX())
                    )
                )
            );

        if (DriverStation.isTest()) {
            driverController
                .buttonBack()
                .and(driverController.buttonY())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            driverController
                .buttonBack()
                .and(driverController.buttonX())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            driverController
                .buttonStart()
                .and(driverController.buttonY())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            driverController
                .buttonStart()
                .and(driverController.buttonX())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

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

        // Pose2d processorSpot = new Pose2d(new Translation2d(6.4, 1.4), Rotation2d.fromDegrees(-90.000));
        // if (DriverStation.getAlliance().get() == Alliance.Red) {
        //     processorSpot = new Pose2d(new Translation2d(11.484, 8.49), Rotation2d.fromDegrees(90.000));
        // }

        // driverController.buttonB().whileTrue(drivebase.driveToPose(processorSpot));

        // driverController.buttonStart().whileTrue(Commands.none());

        driverController
            .buttonStart()
            .onTrue(
                drivetrain.runOnce(() -> {
                    System.out.println("RESSETING GYRO");
                    drivetrain.seedFieldCentric();
                })
            );
        // in
        driverController.leftBumper().whileTrue(new SpinIntake(intake, Constants.INTAKE_SPEED));
        // out
        driverController.rightBumper().whileTrue(new SpinIntake(intake, -Constants.INTAKE_SPEED));

        // driverController.dpadUp().onTrue(dealgaenator.deployDealgenatorSafely(intake));
        // driverController.dpadDown().onTrue(dealgaenator.retractDealgenatorSafely(intake));
        driverController.dpadUp().onTrue(new PivotDealgaenatorToAngle(dealgaenator, DealgaenatorPosition.DEPLOYED));
        driverController.dpadDown().onTrue(new ResetDealgaenator(dealgaenator));
    }

    public void resetIntakePivot() {
        new ResetPivot(intake).schedule();
    }

    public Command resetRobot() {
        return Commands.sequence(
            new ResetPivot(intake),
            new PivotIntakeToAngle(intake, IntakePosition.SCORING).withTimeout(2),
            Commands.parallel(new ResetDealgaenator(dealgaenator).withTimeout(3))
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
        // return autoChooser.getSelected();
        return Commands.none(); // TODO thhis
    }
}
