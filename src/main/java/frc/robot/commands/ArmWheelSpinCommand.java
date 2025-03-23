package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmWheelSpinCommand extends Command {
    private final ArmSubsystem s_ArmSubsystem;
    private double speed;

    public ArmWheelSpinCommand(ArmSubsystem s_ArmSubsystem, double speed) {
        this.s_ArmSubsystem = s_ArmSubsystem;
        this.speed = speed;
        addRequirements(s_ArmSubsystem);
    }

    @Override
    public void execute() {
        s_ArmSubsystem.spin(speed);
    }

    @Override
    public void end(boolean interrupted) {
        s_ArmSubsystem.stopSpin();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}