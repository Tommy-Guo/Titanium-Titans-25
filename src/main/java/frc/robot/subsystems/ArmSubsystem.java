// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsystem extends SubsystemBase {

    private double holdPosition;

    private static final double MAX_POSITION = 100.0; // Example max limit, adjust as needed
    private static final double MIN_POSITION = 0.0; // Min limit

    private SparkMax m_Arm = new SparkMax(ElevatorConstants.kElevator1CanId, MotorType.kBrushless);
    private SparkClosedLoopController m_ArmController = m_Arm.getClosedLoopController();
    private RelativeEncoder m_ArmEncoder = m_Arm.getEncoder();

    // Setup the second Elevator Motor
    private SparkMax m_Wheels = new SparkMax(ElevatorConstants.kElevator2CanId, MotorType.kBrushless);

    /** Creates a new ElevatorSubsystem. */
    public ArmSubsystem() {

        m_Arm.configure(
                Configs.ElevatorSubsystem.elevator1Config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_ArmEncoder.setPosition(0);

    }

    public void move(double speed) {
        double currentPosition = m_ArmEncoder.getPosition();
        if ((currentPosition <= MIN_POSITION && speed < 0) || (currentPosition >= MAX_POSITION && speed > 0)) {
            m_Arm.set(0); // Stop the motor if it exceeds bounds
        } else {
            m_Arm.set(speed);
            holdPosition = currentPosition; // Update hold position
        }
    }

    public void spin(double speed) {
        m_Wheels.set(speed);
    }

    public void moveElevatorToPosition(double Setpoint) {
        m_ArmController.setReference(Setpoint, ControlType.kMAXMotionPositionControl);
    }

    // Get the position of Elevator Motor 1
    public double getArmPosition() {
        return m_ArmEncoder.getPosition();
    }

    // Stop the Elevator Motors
    public void stop() {
        m_Arm.stopMotor();
        m_Wheels.stopMotor();
    }

    public void holdPosition() {
        m_ArmController.setReference(holdPosition, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", m_ArmEncoder.getPosition());
        holdPosition();
    }
}