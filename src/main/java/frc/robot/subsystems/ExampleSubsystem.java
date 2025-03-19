// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ElevatorCOnstants;;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonFX motor;

  public ExampleSubsystem() {
    motor = new TalonFX(ElevatorCOnstants.sparkmax_id);
    
  }


  public Command exampleMethodCommand() {

    return runOnce(
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
