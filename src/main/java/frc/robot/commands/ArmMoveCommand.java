package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveCommand extends Command {

    private final ArmSubsystem s_ArmSubsystem;
    private final double Setpoint;

    public ArmMoveCommand(ArmSubsystem s_ArmSubsystem, double Setpoint) {
        this.s_ArmSubsystem = s_ArmSubsystem;
        this.Setpoint = Setpoint;
        addRequirements(s_ArmSubsystem);
    }
    
    @Override
    public void initialize() {
        s_ArmSubsystem.moveArmToPosition(Setpoint);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_ArmSubsystem.getArmPosition() - Setpoint) < 2.0;
    }
}