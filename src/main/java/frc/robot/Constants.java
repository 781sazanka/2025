// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
  public static class Controller {
    public static final int controllerXboxID = 0;
    public static final int driveXboxID = 1;
    public static final int shooterXboxLeftAxisId = 1;
    public static final int shooterXboxRightAxisId = 5;
    public static final int pivotXboxUpAxisId = 2;
    public static final int pivotXboxDownAxisId = 6;
  }

  public static class CommandStatus {
    public static final boolean testShooter = true;
    public static final boolean testClimb = true;
    public static final boolean testPivot = true;
    public static final boolean testAutoPilot = false; // don't change it unless you're pretty sure what you're doing
  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

 
}