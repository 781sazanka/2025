// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import java.io.IOException;
import java.rmi.dgc.DGC;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.LimelightHelpers;
import frc.robot.Commands.driveToTarget;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.Units;

import java.util.Arrays;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import swervelib.SwerveModule;

import edu.wpi.first.wpilibj.AnalogGyro;

import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class SwerveSubsystem extends SubsystemBase

{

    private SwerveDrive swervedrive;
    private AHRS gyro;
    private AnalogGyro gyro2;
    private boolean endauto;
    private boolean bot_oriented;
    private ChassisSpeeds movement;

    private int multx;

    public SwerveSubsystem(){
        endauto = false;
        bot_oriented = true;       
        
        if (isRed()){
          multx = 1;
        } else{
          multx = -1;
        }
        

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
        swervedrive.setHeadingCorrection(false);
        //swervedrive.setChassisSpeeds(new ChassisSpeeds(0,0,0));

        swervedrive.setAngularVelocityCompensation(true,true,0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    

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
            (speeds, feedforwards) -> drive_ignoreconstraints(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(0.01, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.01, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> isRed(),
            this // Reference to this subsystem to set requirements
    );
    }

    public Boolean isRed(){
      Boolean lol = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
      return lol;
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



    swervedrive.updateOdometry();

    
    }

    public boolean hasended(){
      return endauto == true;
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
    return (new PathPlannerAuto(pathName));
    }

    public Command Stop(){
      endauto=false;
      SmartDashboard.putString("MODE", "Stopped" );
      return run(() -> {swervedrive.lockPose();});
    }

    public void drive(ChassisSpeeds speeds){
      double speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
      if (speed > Constants.MAX_SPEED){
        ChassisSpeeds newspeed = speeds.times(Constants.MAX_SPEED/speed);
        swervedrive.drive(newspeed,false,new Translation2d());
      }else{
        swervedrive.drive(speeds,false,new Translation2d());
      }
        
    }

    public void drive_ignoreconstraints(ChassisSpeeds speeds){
      swervedrive.drive(speeds,false,new Translation2d());
    }

    public void drive_ignoreconstraints_feildoriented(ChassisSpeeds speeds){
      swervedrive.driveFieldOriented(speeds);
    }

    public void drive(ChassisSpeeds speeds,boolean atmax){

      double speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
      if ((atmax == true)|(speed > Constants.MAX_SPEED)){
        ChassisSpeeds newspeed = speeds.times(Constants.MAX_SPEED/speed);
        swervedrive.drive(newspeed,false,new Translation2d());
      } else{
        swervedrive.drive(speeds,false,new Translation2d());
      }
        
    }

    public void drive(ChassisSpeeds speeds,double speedMetersPerSecond){

      double speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
      ChassisSpeeds newspeed = speeds.times(Constants.MAX_SPEED/speed);
      drive(newspeed);
        
    }


    public Command resetpos(){
      SmartDashboard.putString("MODE", "resetting..." );
      return run(() -> swervedrive.setChassisSpeeds(new ChassisSpeeds(1,0,0)));
    }

    public Command switchFeildOriented(){
      return runOnce(() -> {bot_oriented =  !bot_oriented;swervedrive.resetOdometry(new Pose2d());});
    }

    public Command driveFromController(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier heading)
    {
    return run(() -> {

    double x_input = translationX.getAsDouble();
    double y_input = translationY.getAsDouble();
    double rotation = heading.getAsDouble();



    if(Math.abs(translationX.getAsDouble()) < Constants.OperatorConstants.DEADBAND){
      x_input = 0.0;
    }

    if(Math.abs(translationY.getAsDouble()) < Constants.OperatorConstants.DEADBAND){
      y_input = 0.0;
    }

    if(Math.abs(heading.getAsDouble()) < Constants.OperatorConstants.DEADBAND){
      rotation = 0.0;
    }


    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(x_input,y_input), 0.9);



    movement = new ChassisSpeeds(scaledInputs.getY()*Constants.MAX_SPEED, scaledInputs.getX()*-1*Constants.MAX_SPEED,rotation*Constants.Rotation_constant);

    
    if (bot_oriented == true){
      drive_ignoreconstraints(movement);
    }else{
      drive_ignoreconstraints_feildoriented(movement);
    }
  

    
    SmartDashboard.putNumber("input LX", translationX.getAsDouble() );  
    SmartDashboard.putNumber("input LY", translationY.getAsDouble()); 
    SmartDashboard.putNumber("rotation", rotation );  
    SmartDashboard.putNumber("VX m/s", movement.vxMetersPerSecond );  
    SmartDashboard.putNumber("VY m/s", movement.vyMetersPerSecond);  
    SmartDashboard.putNumber("angular velocity", movement.omegaRadiansPerSecond);   
    SmartDashboard.putBoolean("bot oriented", bot_oriented);                                        
    });
    }

    public SwerveDrive getSwerveDrive()
    {
      return swervedrive;
    }
}