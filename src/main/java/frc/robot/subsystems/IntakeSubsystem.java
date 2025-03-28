// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax m_intake1 = new SparkMax(34, MotorType.kBrushless);
    private SparkMax m_intake2 = new SparkMax(33, MotorType.kBrushless);

    /** Creates a new ElevatorSubsystem. */
    public IntakeSubsystem() {

        m_intake1.configure(
                Configs.IntakeSubsystem.intakeMotor1config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_intake2.configure(
                Configs.IntakeSubsystem.intakeMotor2config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void spin(double speed) {
        m_intake1.set(speed);
        m_intake2.set(speed);
    }

    @Override
    public void periodic() {

    }
}