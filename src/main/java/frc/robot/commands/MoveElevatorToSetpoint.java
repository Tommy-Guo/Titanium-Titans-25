// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToSetpoint extends Command {

    private final ElevatorSubsystem s_ElevatorSubsystem;
    private final double Setpoint;

    /** Creates a new MoveElevatorToSetpoint. */
    public MoveElevatorToSetpoint(ElevatorSubsystem s_ElevatorSubsystem, double Setpoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.s_ElevatorSubsystem = s_ElevatorSubsystem;
        this.Setpoint = Setpoint;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Runs the elevator using Max Motion to the desired Setpoint specified in
        // Constants
        s_ElevatorSubsystem.moveElevatorToPosition(Setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // This is the end condition for the elevator subsystem,
        // The elevator will run until its encoder position is within 2 rotations of the
        // desired setpoint
        return Math.abs(s_ElevatorSubsystem.getElevator1Position() - Setpoint) < 2.0;
    }
}