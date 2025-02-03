// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
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

        swervedrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swervedrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swervedrive.setAngularVelocityCompensation(true,
                                                   true,
                                                   0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
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

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,DoubleSupplier headingY)
    {
    return run(() -> {

    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                        translationY.getAsDouble()), 0.8);

    // Make the robot move
    swervedrive.drive(swervedrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                headingX.getAsDouble(),
                                                headingY.getAsDouble(),
                                                swervedrive.getOdometryHeading().getRadians(),
                                                swervedrive.getMaximumChassisVelocity()));

    SmartDashboard.getNumber("heading x", headingX.getAsDouble() );  
    SmartDashboard.getNumber("heading y", headingX.getAsDouble() );  
    SmartDashboard.getNumber("input x", scaledInputs.getX() );  
    SmartDashboard.getNumber("input y", scaledInputs.getY() );  
    SmartDashboard.getNumber("heading", swervedrive.getOdometryHeading().getRadians() );                                        
    });
    }

    public SwerveDrive getSwerveDrive()
    {
      return swervedrive;
    }
}