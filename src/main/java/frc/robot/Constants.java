// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; 
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.20; //s, 20ms + 110ms sprk max velocity lag
  //public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static final double MAX_SPEED  = Units.feetToMeters(1);
  // Maximum speed of the robot in meters per second, used to limit acceleration.


  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorCOnstants{
    public static final double L1_height = 0.46;
    public static final double L2_height = 0.81;
    public static final double L3_height = 1.21;
    public static final double L4_height = 1.83;

    public static final double Pully_ratio = 0.5;
    public static final double falcon_ratio = 3;

    public static final double error_tol = 0.05;

    public static final int sparkmax_id = 0;
  }

  public static final class AutoConstants
  {

    // Hold time on motor brakes when disabled
    public static final double Rotation_stop_threshhold = 0.01; // radians
    public static final double Distance_Threshold = 0.2; // meters
  }

  public static final class moveToSideConstants{
    public static final double distance_left = 0.2;
    public static final double distance_right = 0.3;
    public static final double speed = 0.1;
  }
}