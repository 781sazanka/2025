// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;



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
        //swervedrive.setChassisSpeeds(new ChassisSpeeds(0,0,0));

        //swervedrive.setAngularVelocityCompensation(true,true,0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    }

    @Override
    public void periodic()
    {
      // When vision is enabled we must manually update odometry in SwerveDrive
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

    public Command resetpos(){
      return run(() -> swervedrive.setChassisSpeeds(new ChassisSpeeds(0,0.1,0)));
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,DoubleSupplier headingY)
    {
    return run(() -> {

    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                        translationY.getAsDouble()), 0.8);

    // Make the robot move
    ChassisSpeeds movement = swervedrive.swerveController.getTargetSpeeds(scaledInputs.getX(), 
                                                                          scaledInputs.getY()*-1,
                                                                          headingX.getAsDouble(),
                                                                          headingY.getAsDouble()*-1,
                                                                          swervedrive.getOdometryHeading().getRadians(),
                                                                          swervedrive.getMaximumChassisVelocity());

    swervedrive.drive(movement,false,new Translation2d(0.343, new Rotation2d(Math.PI/4) ));

    
    SmartDashboard.putNumber("input LX", translationX.getAsDouble() );  
    SmartDashboard.putNumber("input LY", translationY.getAsDouble()); 
    SmartDashboard.putNumber("input RX", headingX.getAsDouble() );  
    SmartDashboard.putNumber("input RY", headingY.getAsDouble() );  
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