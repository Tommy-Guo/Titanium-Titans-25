package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlWheelsWithJoystickCommand extends Command {
    private final ArmSubsystem s_ArmSubsystem;
    private final CommandXboxController controller;

    public ControlWheelsWithJoystickCommand(ArmSubsystem s_ArmSubsystem, CommandXboxController controller) {
        this.s_ArmSubsystem = s_ArmSubsystem;
        this.controller = controller;
        addRequirements(s_ArmSubsystem);
    }

    @Override
    public void execute() {
        double speed = controller.getLeftY();
        s_ArmSubsystem.move(speed);
    }

    @Override
    public void end(boolean interrupted) {
        s_ArmSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}