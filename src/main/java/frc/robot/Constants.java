// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static final class DrivebaseConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class ElevatorConstants {
    public static final int kElevator1CanId = 31;
    public static final int kElevator2CanId = 32;

    public static final double kLevel0 = 1;
    public static final double kLevel1 = 18;
    public static final double kLevel2 = 65; 
    public static final double kLevel3 = 121;
    public static final double kLevel4 = 235;
  }

  public static final class ArmConstants {
    public static final int kArmCanId = 51;
    public static final int kWheelsCanId = 56;

    public static final double armSafe = 0;
    public static final double armDown = 0;  
  }
}
