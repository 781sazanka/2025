// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
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

    public Command drivetotarget (){
      return run(() -> {
        double[] results = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumberArray("results",results);
        if (results.length == 6){
          double tx = results[0];
          double ty = results[1];
          double yaw = results[4]/180*Math.PI;
  
          ChassisSpeeds movement = swervedrive.swerveController.getTargetSpeeds(tx,ty,yaw,swervedrive.getOdometryHeading().getRadians(),swervedrive.getMaximumChassisVelocity());
  
          SmartDashboard.putNumber("tx from code", tx );  
          SmartDashboard.putNumber("ty from code", ty);  
          SmartDashboard.putNumber("yaw from code", yaw);  
          swervedrive.driveFieldOriented(movement,new Translation2d(0,0));

        }

        SmartDashboard.putString("MODE", "Driving to target");


      });

    }

    public Command resetpos(){
      return run(() -> {
        swervedrive.setChassisSpeeds(new ChassisSpeeds(0,0,0));
        SmartDashboard.putString("MODE", "Resetting...");
      });
      
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

    swervedrive.driveFieldOriented(movement,new Translation2d(0,0));

    
    SmartDashboard.putNumber("input LX", translationX.getAsDouble() );  
    SmartDashboard.putNumber("input LY", translationY.getAsDouble()); 
    SmartDashboard.putNumber("rotation", rotation );  
    SmartDashboard.putNumber("VX m/s", movement.vxMetersPerSecond );  
    SmartDashboard.putNumber("VY m/s", movement.vyMetersPerSecond);  
    SmartDashboard.putNumber("angular velocity", movement.omegaRadiansPerSecond);  
    
    SmartDashboard.putString("MODE", "Normal Running");
    });
    }

    public SwerveDrive getSwerveDrive()
    {
      return swervedrive;
    }
}