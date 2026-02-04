package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    private final SparkFlex leaderIntake = new SparkFlex(24, MotorType.kBrushless);

    // private final SparkMax leaderIntake = new SparkMax(24, MotorType.kBrushless);
    // private final SparkMax followerIntake = new SparkMax(25, MotorType.kBrushless);

    private final SparkMax pivotArm = new SparkMax(28, MotorType.kBrushless);

    // private final SparkFlex pivotArm = new SparkFlex(28, MotorType.kBrushless);

    // when requesting a digital input, the boolean value will always be true if it is unplugged. 
    private final DigitalInput lowerLimitSwitch = new DigitalInput(3);
    private final DigitalInput upperLimitSwitch = new DigitalInput(2);

    private final double stopSpeed = 0.0;
    private final double setSpeed = 0.1;
    private final double pivotSpeed = 0.1;


    
    public Command runIntake(){
        return new RunCommand(() -> {
            leaderIntake.set(setSpeed);
            // followerIntake.set(-setSpeed);
        }
        // , this 
        );
    }

    // Press and hold verision
    public Command lowerIntake(){
        return new RunCommand(() -> {
            if (!lowerLimitSwitch.get()){
                // System.out.println("Lower limit switch triggered");
                pivotArm.set(stopSpeed);  // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
            } else {
                pivotArm.set(pivotSpeed);
                // System.out.println("Lowering intake");
            }
        }
        // ensures when this command runs, it has sole control of the intake subsystem
        , this
        );
    }
     // Press and hold verision
    public Command raiseIntake(){
        return new RunCommand(() -> {
            if (!upperLimitSwitch.get()){ 
                pivotArm.set(stopSpeed); // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
                // System.out.println("Upper limit switch triggered");
            } else {
                pivotArm.set(-pivotSpeed);
                // System.out.println("Raising intake");
            }
        }
        , this
        );
    }
// // test code for auto deploy
//     private boolean intakeDeployed = true;
private boolean intakeDeployed = true; 
    // public Command toggleIntake(){
    //     return new InstantCommand(()->{
    //         if(intakeDeployed){
    //             raiseIntakeAuto()
    //             .schedule();
    //         } else {
    //             lowerIntakeAuto()
    //             .schedule();
    //         }
    //         intakeDeployed = !intakeDeployed;
    //     }
    //     // , this 
    //     );
    // }

    public Command toggleIntake(){
        return new ConditionalCommand(
            raiseIntakeAuto(), // runs when intakeDeployed = true
            lowerIntakeAuto(), // runs when intakeDeployed = false
            () -> intakeDeployed // the condition used to determine what command to run
        );
    }
    
    public Command lowerIntakeAuto(){
        return new RunCommand(()->{
            pivotArm.set(pivotSpeed);
        }
        , this)
        .until(()-> !lowerLimitSwitch.get()) // runs until limit switch is triggered
        .unless(()-> !lowerLimitSwitch.get()) // prevents running if limit switch is already triggered
        .finallyDo(interrupted -> { 
            pivotArm.set(stopSpeed); 
            if (!interrupted) {
                intakeDeployed = true;
            }
        });
    }

    public Command raiseIntakeAuto(){
        return new RunCommand(()->{
            pivotArm.set(-pivotSpeed);
        }
        , this)
        .until(()-> !upperLimitSwitch.get())
        .unless(()-> !upperLimitSwitch.get())
        .finallyDo(interrupted -> {
            pivotArm.set(stopSpeed);
            if (!interrupted) {
                intakeDeployed = false;
            }
        });
    }


    public Command stopAll(){
        return new RunCommand(()->{
            // leaderIntake.set(stopSpeed);
            // followerIntake.set(stopSpeed);
            pivotArm.set(stopSpeed);
        },
        this
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("lower limit", lowerLimitSwitch.get());
        SmartDashboard.putBoolean("upper limit", upperLimitSwitch.get());
    }
}
