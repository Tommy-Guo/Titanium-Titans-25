package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class DistanceDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Translation2d direction;
    private final double distanceInMeters;
    private Pose2d initialPose;

    public DistanceDriveCommand(SwerveSubsystem swerveSubsystem, Translation2d direction, double distanceInInches) {
        this.swerveSubsystem = swerveSubsystem;
        this.direction = direction;
        this.distanceInMeters = Units.inchesToMeters(distanceInInches);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        initialPose = swerveSubsystem.getPose();
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(direction, 0, true);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerveSubsystem.getPose();
        double distanceTraveled = currentPose.getTranslation().getDistance(initialPose.getTranslation());
        return distanceTraveled >= distanceInMeters;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, true);
    }
}