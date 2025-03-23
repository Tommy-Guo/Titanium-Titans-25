package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class TimedDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Translation2d direction;
    private final double duration;
    private final Timer timer;

    public TimedDriveCommand(SwerveSubsystem swerveSubsystem, Translation2d direction, double duration) {
        this.swerveSubsystem = swerveSubsystem;
        this.direction = direction;
        this.duration = duration;
        this.timer = new Timer();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(direction, 0, true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, true);
        timer.stop();
    }
}