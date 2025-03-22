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

import edu.wpi.first.math.controller.PIDController;




public class setHeight extends Command{

  Elevator elevator;
  double target;

  PIDController pid;

  public setHeight(Elevator e,double target_height){
    elevator = e;
    target = target_height;
    pid.setSetpoint(target_height);
    pid.setTolerance(Constants.ElevatorCOnstants.error_tol);
  }

  @Override
  public void initialize() {
    pid = new PIDController(0.5, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public void execute() {
    elevator.setMovement(pid.calculate(elevator.getCurrentHeight()));
  }
    

}
