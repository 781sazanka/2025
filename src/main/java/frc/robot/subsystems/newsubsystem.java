package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class newsubsystem extends SubsystemBase {
    final CANSparkMax m_motor1 = CANSparkMax(5, MotorType.kBrushless);
    public newsubsystem(){
        m_motor1.restoreFactoryDefaults();
        m_motor1.setIdleMode(IdleMode.kBrake);
    }

    public void forwards(){
        m_motor1.set(1);
    }


    public void backwards(){
        m_motor1.set(-1);
    }
}
