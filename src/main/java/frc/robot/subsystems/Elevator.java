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

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.Timer;


public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX talon_1;
  TalonFX talon_2;
  PIDController pid_controller_1;
  PIDController pid_controller_2;
  double currentHeight;
  Timer elevatorTimer;
  

  public Elevator() {
    talon_1 = new TalonFX(ElevatorCOnstants.Talon1_ID);
    
    talon_2 = new TalonFX(ElevatorCOnstants.Talon2_ID);

    pid_controller_1 = new PIDController(0.5,0,0);
    pid_controller_2 = new PIDController(0.5,0,0);

    currentHeight = 0;
  }

  public void setMovement(double volts) {
    talon_1.set(pid_controller_1.calculate(talon_1.get(),volts));
    talon_2.set(pid_controller_2.calculate(talon_2.get(),volts));
  }

  public double getAnglemean(){
    return (talon_1.getPosition().getValueAsDouble() + talon_2.getPosition().getValueAsDouble())/2;
  }

  public Command stopelevator() {
    return run(() -> setMovement(0));
   }

  public Command moveup() {
   return run(() -> setMovement(1));
  }

  public Command movedown() {
    return run(() -> setMovement(-1));
  }

  public Command setSpeed(DoubleSupplier input) {
    return run(() -> setMovement(input.getAsDouble()));
  }

  public boolean isatHeight(double target_height){ 
    return Math.abs(target_height - getCurrentHeight()) < ElevatorCOnstants.error_tol;
  }

  public double getMeanMovements(){
    return (talon_1.get() + talon_2.get())/2;
  }

  public double getCurrentHeight(){
    return getAnglemean() * 2/ 25;
  }

  public void resetHeightNumber(){
    currentHeight = 0;
  }

  public Command resetHeight(){
    return run(() -> {resetHeightNumber();});
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentHeight = getCurrentHeight();
    SmartDashboard.putNumber("currentheight", getCurrentHeight());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
