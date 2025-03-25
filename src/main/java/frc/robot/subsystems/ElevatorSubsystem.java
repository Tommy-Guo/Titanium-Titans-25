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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  
  private SparkMax m_elevator1 = new SparkMax(ElevatorConstants.kElevator1CanId, MotorType.kBrushless);
  private SparkClosedLoopController m_elevator1Controller = m_elevator1.getClosedLoopController();
  private RelativeEncoder m_elevator1Encoder = m_elevator1.getEncoder();

  private SparkMax m_elevator2 = new SparkMax(ElevatorConstants.kElevator2CanId, MotorType.kBrushless);
  private SparkClosedLoopController m_elevator2Controller = m_elevator2.getClosedLoopController();
  private RelativeEncoder m_elevator2Encoder = m_elevator2.getEncoder();

  public ElevatorSubsystem() {

    m_elevator1.configure(
      Configs.ElevatorSubsystem.elevator1Config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_elevator2.configure(
      Configs.ElevatorSubsystem.elevator2Config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_elevator1Encoder.setPosition(0);
    m_elevator2Encoder.setPosition(0);

  }
  
  public void moveElevatorToPosition(double Setpoint) {
    m_elevator1Controller.setReference(Setpoint, ControlType.kMAXMotionPositionControl);
    m_elevator2Controller.setReference(Setpoint, ControlType.kMAXMotionPositionControl);
  }

  public double getElevator1Position() {
    return m_elevator1Encoder.getPosition();
  }

  public double getElevator2Position() {
    return m_elevator2Encoder.getPosition();
  }

  public void resetElevatorEncoders() {
    m_elevator1Encoder.setPosition(0);
    m_elevator2Encoder.setPosition(0);
  }

  public void stopElevatorMotors() {
    m_elevator1.stopMotor();
    m_elevator2.stopMotor();
  }

  @Override
  public void periodic() {
    
  }
}