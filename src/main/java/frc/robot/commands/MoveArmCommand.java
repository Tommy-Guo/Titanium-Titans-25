// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends Command {
    private final ArmSubsystem s_ArmSubsystem;
    private double setpoint;

    public MoveArmCommand(ArmSubsystem s_ArmSubsystem, double setpoint) {
        this.s_ArmSubsystem = s_ArmSubsystem;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (setpoint == 0) {
            s_ArmSubsystem.moveElevatorToPosition(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_ArmSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
