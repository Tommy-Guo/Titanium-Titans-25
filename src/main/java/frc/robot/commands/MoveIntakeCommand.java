package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public MoveIntakeCommand(IntakeSubsystem intakeSubsystem, double speed) {
        this.speed = speed;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.spin(speed);
    }

    @Override
    public boolean isFinished() {
        return speed == 0;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.spin(0);
    }
}