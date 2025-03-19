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

import frc.robot.subsystems.Elevator;




public class setHeight extends Command{

  Elevator elevator;
  double target;


  public setHeight(Elevator e,double target_height){
    elevator = e;
    target = target_height;
  }

  @Override
  public void initialize() {
  }

  @Override
  public boolean isFinished() {
    return elevator.isatHeight(target);
  }

  @Override
  public void execute() {
    

  }
    

  public void setvalues(){

  }
}
