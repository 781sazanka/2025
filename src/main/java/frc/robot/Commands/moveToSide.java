package frc.robot.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.moveToSideConstants;





public class moveToSide extends Command{
    private final SwerveSubsystem swerveSubsystem;
    SwerveDrive swervedrive;
    public Timer timer;
    public double dist;
    public double mult;

    public double time_needed;
    public double speed;

    public moveToSide(SwerveSubsystem inputSubsystem,Boolean toright){
        speed = Constants.moveToSideConstants.speed;
        if (speed > Constants.MAX_SPEED){
          speed = Constants.MAX_SPEED;
        }
        swerveSubsystem = inputSubsystem;
        swervedrive = swerveSubsystem.getSwerveDrive();
        if (toright){
          dist = moveToSideConstants.distance_right;
          mult = 1;
        }else{
          dist = moveToSideConstants.distance_left;
          mult = -1;
        }

        timer = new Timer();

        time_needed = dist/speed;

    }
    @Override
    public void initialize(){
      timer.restart();
    }

    @Override
    public boolean isFinished(){
      return timer.hasElapsed(time_needed);
    }
      

    @Override
    public void execute() {
      ChassisSpeeds movement = new ChassisSpeeds(speed*mult,0 ,0);
      swervedrive.drive(movement);                                  
      }

}
