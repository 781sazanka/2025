package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class newsubsystem extends SubsystemBase {
    final SparkMax m_motor1 = new SparkMax(5, MotorType.kBrushless);
    //private TalonFX m_motor1 = new TalonFX(1,"rio");
    public newsubsystem(){
    }

    public void forwards(){
        m_motor1.set(0.9);
    }

    public void stop(){
        m_motor1.set(0);
    }


    public void backwards(){
        m_motor1.set(-0.9);
    }
}
