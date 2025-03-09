// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.security.cert.X509CRL;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

import java.util.Arrays;


public class SwerveSubsystem extends SubsystemBase
{

    private SwerveDrive swervedrive;
    public SwerveSubsystem(){

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        double maximumSpeed = Constants.MAX_SPEED;
        File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    
        try {
            swervedrive = new SwerveParser(directory).createSwerveDrive(
            maximumSpeed
          );
        } catch (Exception e) {
          throw new RuntimeException(e);
        }

        swervedrive.resetOdometry(new Pose2d());
        swervedrive.resetDriveEncoders();
        swervedrive.setHeadingCorrection(true);
        //swervedrive.setChassisSpeeds(new ChassisSpeeds(0,0,0));

        //swervedrive.setAngularVelocityCompensation(true,true,0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    }

    @Override
    public void periodic()
    {
    /* 

    //こちら大会本番で使うかもしれないフィールドビジョンローカライゼーション　動くかどうかはまじで知らん。

      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (limelightMeasurement.tagCount >= 2) {  // 複数のタグが見える場合のみ測定を信頼する
        swervedrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        swervedrive.addVisionMeasurement(
              limelightMeasurement.pose,
              limelightMeasurement.timestampSeconds
      );
      }

    */
  
    }
  
    @Override
    public void simulationPeriodic()
    {
    }

    public Command getAutonomousCommand(String pathName)
    {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
    }

    public Command Stop(){
      return run(() -> swervedrive.lockPose());
    }

    public Command drivetotarget() {
      return run(() -> {
          RawDetection[] results = LimelightHelpers.getRawDetections("");
          if (results.length > 0) {
              RawDetection tag = results[0];
              double tx = tag.txnc;
              double ty = tag.tync;
              double area = tag.ta;
  
              double top_left_x = tag.corner0_X;
              double top_left_y = tag.corner0_Y;
              double top_right_x = tag.corner1_X;
              double top_right_y = tag.corner1_Y;
              double bottom_left_x = tag.corner3_X;
              double bottom_left_y = tag.corner3_Y;
              double bottom_right_x = tag.corner2_X;
              double bottom_right_y = tag.corner2_Y;
  
              // Rotation calculation (still may need fine-tuning)
              double dist_right = top_right_y - bottom_right_y;
              double dist_left = top_left_y - bottom_left_y;
              double rotation = (dist_left - dist_right) / 480 * Math.PI / 2; // adjust scaling factor as needed
  
              // Speed adjustment (consider refining this)
              double speed = Math.max(0.00, 0.1 - area); // Avoiding negative speeds
  
              // Angle calculation (you might want to adjust the scaling factor here)
              double angle = (tx - 320) / 320 * Math.PI / 4;
  
              // Translation2d used to control forward movement and rotation
              Translation2d translation = new Translation2d(speed, new Rotation2d(angle));
  
              // Convert translation to chassis speeds
              ChassisSpeeds movement = swervedrive.swerveController.getTargetSpeeds(
                  translation.getX(),
                  translation.getY(),
                  rotation,
                  swervedrive.getOdometryHeading().getRadians(),
                  swervedrive.getMaximumChassisVelocity()
              );
  
              // Drive the robot based on calculated speeds
              swervedrive.drive(movement, new Translation2d(0, 0));
  
              // Optionally log data for debugging
              SmartDashboard.putNumber("tx", tx);
              SmartDashboard.putNumber("ty", ty);
              SmartDashboard.putNumber("area", area);
              SmartDashboard.putNumber("rotation", rotation);
              SmartDashboard.putNumber("speed", speed);
              SmartDashboard.putNumber("angle", angle);
          }
      });
  }
  
  



    public Command resetpos(){
      return run(() -> swervedrive.setChassisSpeeds(new ChassisSpeeds(0,0.1,0)));
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier heading)
    {
    return run(() -> {

    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),translationY.getAsDouble()), 0.8);

    double rotation = heading.getAsDouble();

    // Make the robot move
    ChassisSpeeds movement = swervedrive.swerveController.getTargetSpeeds(scaledInputs.getX(), 
                                                                          scaledInputs.getY()*-1,
                                                                          rotation,
                                                                          swervedrive.getOdometryHeading().getRadians(),
                                                                          swervedrive.getMaximumChassisVelocity());

    //swervedrive.drive(movement,false,new Translation2d(0.343, new Rotation2d(Math.PI/4) ));

    swervedrive.driveFieldOriented(movement,new Translation2d(0,0 ));

    
    SmartDashboard.putNumber("input LX", translationX.getAsDouble() );  
    SmartDashboard.putNumber("input LY", translationY.getAsDouble()); 
    SmartDashboard.putNumber("rotation", rotation );  
    SmartDashboard.putNumber("VX m/s", movement.vxMetersPerSecond );  
    SmartDashboard.putNumber("VY m/s", movement.vyMetersPerSecond);  
    SmartDashboard.putNumber("angular velocity", movement.omegaRadiansPerSecond);                                        
    });
    }

    public SwerveDrive getSwerveDrive()
    {
      return swervedrive;
    }
}