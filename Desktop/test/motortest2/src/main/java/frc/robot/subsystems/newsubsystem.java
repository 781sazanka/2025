package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class newsubsystem extends SubsystemBase {
    //private TalonFX m_motor1 = new TalonFX(1,"rio"); 
    final SparkMax m_motor1 = new SparkMax(4, MotorType.kBrushless);
    public newsubsystem(){}

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


