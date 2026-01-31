package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkFlex; // For the SPARK Flex controller
import com.revrobotics.spark.SparkMax;  // For the SPARK MAX controller (if using a solo adapter)
import com.revrobotics.spark.SparkLowLevel.MotorType; // To specify brushless motor type
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController; // For advanced control like PID loops (optional)
import com.revrobotics.spark.SparkRelativeEncoder; // For accessing the built-in encoder (optional)
import edu.wpi.first.wpilibj.DigitalInput;


import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class Intake extends SubsystemBase{
    
    // private final TalonFX leaderIntake = new TalonFX(24);
    // private final TalonFX followerIntake = new TalonFX(25);

    private final SparkMax leaderIntake = new SparkMax(24, MotorType.kBrushless);
    private final SparkMax followerIntake = new SparkMax(25, MotorType.kBrushless);

    private final SparkMax pivotArm = new SparkMax(28, MotorType.kBrushless);

    // private final SparkFlex pivotArm = new SparkFlex(28, MotorType.kBrushless);

    private final DigitalInput lowerLimitSwitch = new DigitalInput(1);
    private final DigitalInput upperLimitSwitch = new DigitalInput(2);

    private double setSpeed = 0.5;


    
    public Command runIntake(){
        return new RunCommand(() -> {
            leaderIntake.set(setSpeed);
            followerIntake.set(-setSpeed);
        }
        // , this 
        );
    }

    public Command lowerIntake(){
        return new RunCommand(() -> {
            if (lowerLimitSwitch.get()){
                pivotArm.set(0.0);  // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
                System.out.println("Switch = false");
            } else {
                pivotArm.set(0.1);
                System.out.println("Switch = true");
            }
        }
        // ensures when this command runs, it has sole control of the intake subsystem
        , this
        );
    }
    
    public Command raiseIntake(){
        return new RunCommand(() -> {
            if (!(upperLimitSwitch.get())){
                pivotArm.set(-0.1); // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
            } else {
                pivotArm.set(0);
            }
        }
        , this
        );
    }

    public Command stopAll(){
        return new RunCommand(()->{
            leaderIntake.set(0);
            followerIntake.set(0);
            pivotArm.set(0);
        },
        this
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
