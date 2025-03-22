// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase;

public class arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private SparkMax sparkmax;

  private RelativeEncoder encoder;


  public arm() {
    sparkmax = new SparkMax(Constants.armCOnstants.sparkmaxID,MotorType.kBrushless);
    encoder = sparkmax.getEncoder();
  }

  public void setZero(){
    rotatetoangle(0);
  }


  public void rotatetoangle(double angle){
    encoder.setPosition(angle);

  }

  public Command setToAngle(DoubleSupplier input){
    return run(() -> {rotatetoangle(input.getAsDouble());});
  }

  public Command setToAngle(Double input){
    return run(() -> {rotatetoangle(input);});
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
