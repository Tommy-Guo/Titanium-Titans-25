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
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    // Setup the first Elevator Motor
    private SparkMax m_Arm = new SparkMax(ArmConstants.kArmCanId, MotorType.kBrushless);
    private SparkClosedLoopController m_ArmController = m_Arm.getClosedLoopController();
    private RelativeEncoder m_ArmEncoder = m_Arm.getEncoder();

    // Setup the second Elevator Motor
    private SparkMax m_Wheels = new SparkMax(ArmConstants.kWheelsCanId, MotorType.kBrushless);

    /** Creates a new ElevatorSubsystem. */
    public ArmSubsystem() {

        m_Arm.configure(
                Configs.ArmSubsystem.armMotor,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_Wheels.configure(
                Configs.ArmSubsystem.wheelMotor,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_ArmEncoder.setPosition(0);

    }

    // This method moves the elevator to the specified setpoint using MaxMotion
    // MaxMotion is a trapozoidal profile and we can manipulate its PID, velocity
    // and acceleration in configs
    public void moveArmToPosition(double Setpoint) {
        m_ArmController.setReference(Setpoint, ControlType.kMAXMotionPositionControl);
    }

    // Get the position of Elevator Motor 1
    public double getArmPosition() {
        return m_ArmEncoder.getPosition();
    }

    // Reset the Elevator Encoders to 0
    public void resetArmEncoders() {
        m_ArmEncoder.setPosition(0);
    }

    public void spin(double speed) {
        m_Wheels.set(speed);
    }

    public void stopSpin() {
        m_Wheels.stopMotor();
    }

    // Stop the Elevator Motors
    public void stopArmMotors() {
        m_Arm.stopMotor();
    }

    @Override
    public void periodic() {

    }
}