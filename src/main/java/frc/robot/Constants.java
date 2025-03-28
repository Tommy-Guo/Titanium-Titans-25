// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class ElevatorConstants {

    public static final int kElevator1CanId = 31;
    public static final int kElevator2CanId = 32;

    public static final double kHome = 0;
    public static final double kFeederStation = 14; // 14
    public static final double kTravel = 0;

    public static final double kLevel0 = 1;
    public static final double kLevel1 = 18;
    public static final double kLevel2 = 65; 
    public static final double kLevel3 = 121;
    public static final double kLevel4 = 235;
  }

  public static final class ArmConstants {
    
    public static final int kArmCanId = 33;
    public static final int kWheelsCanId = 34;

    public static final double armSafe = 0;
    public static final double armUse = 0;  
  }

  public static final double X_REEF_ALIGNMENT_P = 3.3;
  public static final double Y_REEF_ALIGNMENT_P = 3.3;
  public static final double ROT_REEF_ALIGNMENT_P = 0.058;

  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
  public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34; // Vertical pose
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
  public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16; // Horizontal pose
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double POSE_VALIDATION_TIME = 0.3;
}
