package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class newsubsystem extends SubsystemBase {
    final SparkMax m_motor1 = new SparkMax(5, MotorType.kBrushless);
    public newsubsystem(){
    }

    public void forwards(){
        m_motor1.set(0.1);
    }

    public void stop(){
        m_motor1.set(0);
    }


    public void backwards(){
        m_motor1.set(-0.1);
    }
}
