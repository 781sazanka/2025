package frc.robot.Commands;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.kinematics.ChassisSpeeds;




public class driveToTarget extends Command{
    private final SwerveSubsystem swerveSubsystem;
    public double rotation;
    public double distance;
    public double x;
    public double y;
    SwerveDrive swervedrive;

    public driveToTarget(SwerveSubsystem inputSubsystem){
        swerveSubsystem = inputSubsystem;
        swervedrive = swerveSubsystem.getSwerveDrive();

    }

    @Override
    public void initialize() {
        setvalues();
    }
  
    @Override
    public boolean isFinished() {
        if ((rotation < Constants.AutoConstants.Rotation_stop_threshhold) & (distance < Constants.AutoConstants.Distance_Threshold)){
          return true;
        }else{
          return false;
        }
    }

    @Override
    public void execute() {
            SmartDashboard.putString("MODE", "Driving to target" );  
      
            setvalues();
    
            ChassisSpeeds movement = swervedrive.swerveController.getTargetSpeeds(x, y,rotation,swervedrive.getOdometryHeading().getRadians(),swervedrive.getMaximumChassisVelocity());
    
            SmartDashboard.putNumber("drivetotarget/x", x );  
            SmartDashboard.putNumber("drivetotarget/y", y); 
            SmartDashboard.putNumber("drivetotarget/rotation", rotation );  
            SmartDashboard.putNumber("drivetotarget/VX", movement.vxMetersPerSecond );  
            SmartDashboard.putNumber("drivetotarget/VY", movement.vyMetersPerSecond);  
            SmartDashboard.putNumber("drivetotarget/angular velocity", movement.omegaRadiansPerSecond); 
            
            swerveSubsystem.drive(movement);
                                        
          }
      

    public void setvalues(){
      double[] results = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
      Pose2d pose = LimelightHelpers.toPose2D(results);

      rotation = pose.getRotation().getRadians();
      x = pose.getMeasureX().in(Units.Meter);
      y = pose.getY();

      distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
}
