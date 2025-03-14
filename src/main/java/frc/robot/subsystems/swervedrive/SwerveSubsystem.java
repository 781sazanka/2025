// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.Units;

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
    

    NamedCommands.registerCommand("STOP", this.Stop());
    RobotConfig config = null;
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (IOException e) {
      // Handle exception as needed
      e.printStackTrace();
    } catch (ParseException e){
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getpose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false),
            this // Reference to this subsystem to set requirements
    );
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

    public Pose2d getpose(){
      return swervedrive.getPose();
    }
    public void resetPose(Pose2d pose){
      swervedrive.resetOdometry(pose);
    }
    public ChassisSpeeds getSpeed(){
      return swervedrive.getRobotVelocity();
    }

    public Command getAutonomousCommand(String pathName)
    {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    SmartDashboard.putString("MODE", "Getting Auto" );
    return new PathPlannerAuto(pathName);
    }

    public Command Stop(){
      SmartDashboard.putString("MODE", "Stopped" );
      return run(() -> swervedrive.lockPose());
    }

    public void drive(ChassisSpeeds speeds){
      swervedrive.drive(speeds);
    }


    public Command resetpos(){
      SmartDashboard.putString("MODE", "resetting..." );
      return run(() -> swervedrive.setChassisSpeeds(new ChassisSpeeds(0,0.1,0)));
    }

    public Command driveFromController(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier heading)
    {
    return run(() -> {

    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),translationY.getAsDouble()), 0.8);

    double rotation = heading.getAsDouble();

    // Make the robot move
    ChassisSpeeds movement = swervedrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY()*-1,rotation,swervedrive.getOdometryHeading().getRadians(),swervedrive.getMaximumChassisVelocity());

    drive(movement);

    
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