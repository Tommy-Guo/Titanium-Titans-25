// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.depreciated;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {

    private final IntakeSubsystem s_IntakElevatorSubsystem;
    private final ElevatorSubsystem s_ElevatorSubsystem;
    private final double level;

    /** Creates a new MoveElevatorToSetpoint. */
    public AutoScore(ElevatorSubsystem s_ElevatorSubsystem, IntakeSubsystem intakeSubsystem, double level) {
        this.level = level;
        this.s_ElevatorSubsystem = s_ElevatorSubsystem;
        this.s_IntakElevatorSubsystem = intakeSubsystem;
        addRequirements(s_IntakElevatorSubsystem, s_ElevatorSubsystem);
    }

    private boolean complete = false;

    @Override
    public void execute() {
        s_ElevatorSubsystem.moveElevatorToPosition(level);
        while (Math.abs(s_ElevatorSubsystem.getElevator1Position() - level) < 2.0) {
            // Do nothing
        }
        s_IntakElevatorSubsystem.spin(0.2);
        complete = true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}