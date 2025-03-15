package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DealgaenatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ResetEncoders extends Command {

    public final IntakeSubsystem intakeSubsystem;
    public final ClimbSubsystem climbSubsystem;
    public final DealgaenatorSubsystem dealgaenatorSubsystem;

    public ResetEncoders(
        IntakeSubsystem intakeSubsystem,
        ClimbSubsystem climbSubsystem,
        DealgaenatorSubsystem dealgaenatorSubsystem
    ) {
        this.intakeSubsystem = intakeSubsystem;
        this.climbSubsystem = climbSubsystem;
        this.dealgaenatorSubsystem = dealgaenatorSubsystem;
        addRequirements(intakeSubsystem, climbSubsystem, dealgaenatorSubsystem);
    }

    @Override
    public String getName() {
        return "hhhj";
    }

    @Override
    public void initialize() {
        intakeSubsystem.resetEncoder();
        climbSubsystem.resetEncoder();
        dealgaenatorSubsystem.resetEncoder();
        System.out.println("reset");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
