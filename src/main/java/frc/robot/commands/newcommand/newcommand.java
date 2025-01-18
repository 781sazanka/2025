package frc.robot.commands.newcommand;

import frc.robot.subsystems.newsubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class newcommand extends Command {
    private final newsubsystem newsubsystem;

    private final Supplier<Boolean> isButtonPressed_1;
    private final Supplier<Boolean> isButtonPressed_2;

    public newcommand(newsubsystem input_subsystem,Supplier<Boolean> button_Supplier_1,Supplier<Boolean> button_Supplier_2){
        newsubsystem = input_subsystem;
        isButtonPressed_1 = button_Supplier_1;
        isButtonPressed_2 = button_Supplier_2;

        addRequirements(newsubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(isButtonPressed_1.get()){
            newsubsystem.forwards();
        }else if(isButtonPressed_2.get()){
            newsubsystem.backwards();
        }else{
            newsubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
