package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    private final double duration;
    private final Timer timer = new Timer();

    public TimedIntakeCommand(IntakeSubsystem intakeSubsystem, double speed, double duration) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        this.duration = duration;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intakeSubsystem.spin(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.spin(0);
        timer.stop();
    }
}