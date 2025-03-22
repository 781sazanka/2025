package frc.robot.Commands;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Elevator;




public class moveEndEffector extends Command{

  private SparkMax sparkmax;
  private double speed;

  public moveEndEffector(SparkMax inputTalon,double inputSpeed){
    sparkmax = inputTalon;
    speed = inputSpeed;
  }

  @Override
  public void initialize() {
    sparkmax.set(0);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs((sparkmax.get()-speed)) > Constants.endEffectorConstants.spikeThreshold){
      sparkmax.set(0);
      return true;
    }else{
      return false;
    }
    
  }

  @Override
  public void execute() {
    sparkmax.set(speed);
  }

  public void insert(){

  }

  
}
