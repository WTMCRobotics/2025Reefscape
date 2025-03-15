// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ClimbReset;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TFMini;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    private static Robot instance;
    private Command m_autonomousCommand;

    private RobotContainer robotContainer;

    // private Timer disabledTimer;

    SendableChooser<String> autonRouteChooser = new SendableChooser<>();

    public Robot() {
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        autonRouteChooser.addOption("forward1meter", "test");
        autonRouteChooser.addOption("back1meter", "testback");
        autonRouteChooser.addOption("forward0.5meter", "testhalffor");
        autonRouteChooser.addOption("back0.5meter", "testhalfback");
        autonRouteChooser.addOption("forward1meterfast", "for1mfast");
        autonRouteChooser.addOption("back1meterfast", "back1mfast");
        autonRouteChooser.addOption("spin", "spin");
        // autonRouteChooser.addOption("new auto", "New New Auto");
        SmartDashboard.putData("auton routes", autonRouteChooser);

        robotContainer = new RobotContainer();
        // disabledTimer = new Timer();

        robotContainer.doGyroSetup();

        if (Robot.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        NamedCommands.registerCommand("ResetRobot", robotContainer.resetRobot());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        robotContainer.resetSimulation();
        robotContainer.setMotorBrake(true);
        // disabledTimer.reset();
        // disabledTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        // if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
        //   robotContainer.setMotorBrake(false);
        //   disabledTimer.stop();
        // }
    }

    // public static final TFMini tfMini = new TFMini(new SerialPort(115200, SerialPort.Port.kMXP, 8, Parity.kNone, StopBits.kOne));

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        robotContainer.setMotorBrake(true);
        m_autonomousCommand = robotContainer.getAutonomousCommand(autonRouteChooser.getSelected());

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        } else {
            CommandScheduler.getInstance().cancelAll();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        robotContainer.sendTelementary();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.resetIntakePivot();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        robotContainer.displaySimFieldToAdvantageScope();
    }
}
