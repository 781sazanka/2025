// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.proto.ElevatorFeedforwardProto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ElevatorCOnstants;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX talon_1;
  TalonFX talon_2;
  PIDController pid_controller_1;
  PIDController pid_controller_2;

  

  public Elevator() {
    talon_1 = new TalonFX(ElevatorCOnstants.Talon1_ID);
    
    talon_2 = new TalonFX(ElevatorCOnstants.Talon2_ID);

    pid_controller_1 = new PIDController(0.5,0,0);
    pid_controller_2 = new PIDController(0.5,0,0);
  }

  public void setMovement(double volts) {
    talon_1.set(volts);

    talon_1.get();
    talon_2.set(volts);
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
