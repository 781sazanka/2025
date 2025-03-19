// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ElevatorCOnstants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX talon;
  public Elevator() {
    talon = new TalonFX(ElevatorCOnstants.sparkmax_id);
  }

  public void setvoltage(double volts) {
    talon.setVoltage(volts);
  }

  public boolean isatHeight(double target_height){ 

    return Math.abs(target_height - getCurrentHeight()) < ElevatorCOnstants.error_tol;

  }

  public double getCurrentHeight(){
    double getCurrentHeight = 0;
    return getCurrentHeight;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
