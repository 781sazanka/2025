// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Commands.driveToTarget;
import frc.robot.Commands.moveToSide;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  final         CommandGenericHID driverXbox = new CommandGenericHID(3);
  private final SwerveSubsystem drivebase  = new SwerveSubsystem();
  private final SendableChooser<Command> autoChooser;

  private Command autCommand;




  public RobotContainer()
  {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();



    Command stop = drivebase.Stop();
    stop.addRequirements(drivebase);
    driverXbox.button(2).whileTrue(stop);

    Command reset = drivebase.resetpos();
    reset.addRequirements(drivebase);
    driverXbox.button(3).whileTrue(reset);

    Command Drivetotarget = new driveToTarget(drivebase);
    Drivetotarget.addRequirements(drivebase);
    driverXbox.button(4).whileTrue(Drivetotarget);

    Command goLeft = new moveToSide(drivebase,false);
    goLeft.addRequirements(drivebase);
    driverXbox.button(5).onTrue(goLeft);

    Command goRight = new moveToSide(drivebase,true);
    goRight.addRequirements(drivebase);
    driverXbox.button(6).onTrue(goRight);

    NamedCommands.registerCommand("Drive To Target", Drivetotarget);
    NamedCommands.registerCommand("Reset", reset);
    NamedCommands.registerCommand("stop", stop);
    NamedCommands.registerCommand("goRight", goRight);
    NamedCommands.registerCommand("goLeft", goLeft);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  public void configureBindings(){
    
    Command swerve_Command = drivebase.driveFromController(() -> driverXbox.getRawAxis(0),() -> driverXbox.getRawAxis(1),() -> driverXbox.getRawAxis(2));
    drivebase.setDefaultCommand(swerve_Command);

  }
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    autCommand = autoChooser.getSelected();
    autCommand.addRequirements(drivebase);
    return autCommand;
  }

  public void setMotorBrake(boolean brake)
  {

  }
}