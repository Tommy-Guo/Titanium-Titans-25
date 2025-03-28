package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Configs {

        public static final class ElevatorSubsystem {

                public static final SparkMaxConfig elevator1Config = new SparkMaxConfig();
                public static final SparkMaxConfig elevator2Config = new SparkMaxConfig();

                static {

                        elevator1Config
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .voltageCompensation(12)
                                        .inverted(true);

                        elevator1Config.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.22)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(6000)
                                        .maxAcceleration(7000)
                                        .allowedClosedLoopError(0.5);

                        // elevator1Config.closedLoop
                        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // .p(0.2)
                        // .outputRange(-1, 1).maxMotion
                        // .maxVelocity(8000)
                        // .maxAcceleration(10000)
                        // .allowedClosedLoopError(0.5);

                        elevator2Config
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .voltageCompensation(12)
                                        .inverted(false);

                        elevator2Config.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.22)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(6000)
                                        .maxAcceleration(7000)
                                        .allowedClosedLoopError(0.5);

                        // elevator2Config.closedLoop
                        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // .p(0.2)
                        // .outputRange(-1, 1).maxMotion
                        // .maxVelocity(8000)
                        // .maxAcceleration(10000)
                        // .allowedClosedLoopError(0.5);

                }

        }

        public static final class IntakeSubsystem {

                public static final SparkMaxConfig intakeMotor1config = new SparkMaxConfig();
                public static final SparkMaxConfig intakeMotor2config = new SparkMaxConfig();

                static {
                        intakeMotor1config
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .voltageCompensation(12)
                                        .inverted(true);
                        intakeMotor2config
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .voltageCompensation(12)
                                        .inverted(false);

                }
        }

        public static final class ArmSubsystem {

                public static final SparkMaxConfig armMotor = new SparkMaxConfig();
                static {
                        armMotor
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .voltageCompensation(12)
                                        .inverted(true);

                        armMotor.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.2)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(4500)
                                        .maxAcceleration(5000)
                                        .allowedClosedLoopError(0.5);

                }

                public static final SparkMaxConfig wheelMotor = new SparkMaxConfig();
                static {
                        wheelMotor
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .voltageCompensation(12)
                                        .inverted(true);

                }

        }

}