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
                    .p(0.2)
                    .outputRange(-1, 1).maxMotion
                    .maxVelocity(4000)
                    .maxAcceleration(8000)
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
                    .p(0.2)
                    .outputRange(-1, 1).maxMotion
                    .maxVelocity(4000)
                    .maxAcceleration(8000)
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

}