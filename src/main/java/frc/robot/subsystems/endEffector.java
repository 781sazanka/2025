// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Commands.moveEndEffector;

public class endEffector extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonFX talon;
  private boolean isrunning;



  public endEffector() {
    talon = new TalonFX(Constants.endEffectorConstants.talonID);
  }


  public Command intake() {
    return new moveEndEffector(talon,-1);
  }

  public Command shoot() {
    return run(
        () -> {
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
