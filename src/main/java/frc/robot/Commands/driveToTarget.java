package frc.robot.Commands;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.wpilibj.Timer;

import swervelib.SwerveInputStream.*;

import edu.wpi.first.math.controller.PIDController;





public class driveToTarget extends Command{
    private final SwerveSubsystem swerveSubsystem;
    public double rotation;
    public double distance;
    public double x;
    public double y;
    SwerveDrive swervedrive;
    SwerveInputStream inputStream;

    PIDController pid_x;
    PIDController pid_y;
    PIDController pid_rot;

    Timer timer;
    boolean started;

    public driveToTarget(SwerveSubsystem inputSubsystem){
        swerveSubsystem = inputSubsystem;
        swervedrive = swerveSubsystem.getSwerveDrive();

    }

    @Override
    public void initialize() {
      setvalues();
      timer = new Timer();
      started = false;
      pid_x = new PIDController(0.5, 0, 0);
      pid_y = new PIDController(0.5, 0, 0);
      pid_rot = new PIDController(0.5, 0, 0);
      pid_rot.enableContinuousInput(0, Math.PI*2);

      pid_x.setSetpoint(0);
      pid_x.setTolerance(AutoConstants.x_threshold);

      pid_y.setSetpoint(AutoConstants.target_distance);
      pid_y.setTolerance(AutoConstants.Distance_Threshold);

      pid_rot.setSetpoint(AutoConstants.rotation_offset);
      pid_rot.setTolerance(AutoConstants.Rotation_stop_threshhold);
    }

    public double getx(){
      return x;
    }
    public double gety(){
      return y;
    }

    public double getrot(){
      return rotation;
    }

    public boolean isattarget(){
      if (pid_x.atSetpoint()&pid_y.atSetpoint()&pid_rot.atSetpoint()){
        return true;
      }else{
        return false;
      }
    }

  
    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(AutoConstants.time_threshold)){

          return true;
        }else{
          return false;
        }

    }

    @Override
    public void execute() {
            SmartDashboard.putString("MODE", "Driving to target" );  
      
            setvalues();

            rotation = -1 * rotation;


            double x_move = pid_x.calculate(x,0);
            double y_move = pid_y.calculate(y,AutoConstants.target_distance);
            double rot_move = pid_rot.calculate(rotation,0);

    
            ChassisSpeeds movement = new ChassisSpeeds(y_move, -1*x_move,rot_move);


    
            SmartDashboard.putNumber("drivetotarget/x", x );  
            SmartDashboard.putNumber("drivetotarget/y", y); 
            SmartDashboard.putNumber("drivetotarget/rotation", rotation );  
            SmartDashboard.putNumber("drivetotarget/VX", movement.vxMetersPerSecond );  
            SmartDashboard.putNumber("drivetotarget/VY", movement.vyMetersPerSecond);  
            SmartDashboard.putNumber("drivetotarget/angular velocity", movement.omegaRadiansPerSecond); 

            swerveSubsystem.drive(movement);

            
            if(isattarget() & started==false){
              timer.restart();
              started = true;
            }
            
            if(isattarget() == false){
              started = false;
              timer.stop();
            }

                                        
          }

    public double getspeed(double current_distance){
        return Math.pow(Constants.AutoConstants.target_distance - distance, 5);
    }
      

    public void setvalues(){
      double[] results = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
      Pose3d pose3d = LimelightHelpers.toPose3D(results);

      if ((0 == pose3d.getX())|(0 == pose3d.getZ())){

      }else{ 
        Rotation3d rotation3d = pose3d.getRotation();
        rotation = rotation3d.getY() - AutoConstants.rotation_offset;

        
        x = pose3d.getMeasureX().in(Units.Meter);
        y = pose3d.getMeasureZ().in(Units.Meter);

      }

      distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

      SmartDashboard.putNumber("Target/x", x );  
      SmartDashboard.putNumber("Target/y", y); 
      SmartDashboard.putNumber("Target/dist", distance); 
    }

    @Override
    public void end(boolean interrupted) {
      //swerveSubsystem.Stop();
  
    }
}
