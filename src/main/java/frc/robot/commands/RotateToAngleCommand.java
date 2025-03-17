package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateToAngleCommand extends Command {
    private final SwerveSubsystem driveBase;
    private final PIDController rotController;
    private final double targetAngle;

    public RotateToAngleCommand(SwerveSubsystem drivebaseSubsystem, double targetAngle) {
        this.driveBase = drivebaseSubsystem;
        this.rotController = new PIDController(0.1, 0, 0); // Adjust PID values as needed
        this.targetAngle = targetAngle;
        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void initialize() {
        rotController.setSetpoint(targetAngle);
        rotController.setTolerance(2.0); // Tolerance in degrees
    }

    @Override
    public void execute() {
        double currentAngle = driveBase.getHeading().getDegrees();
        double rotSpeed = rotController.calculate(currentAngle);
        driveBase.drive(new Translation2d(), rotSpeed, false);
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new Translation2d(), 0, false);
    }
}