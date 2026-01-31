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
    
    private final TalonFX leaderIntake = new TalonFX(24);
    private final TalonFX followerIntake = new TalonFX(25);

    private final SparkFlex pivotArm = new SparkFlex(28, MotorType.kBrushless);

    private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(4);
    private final DigitalInput pivotLimitSwitch = new DigitalInput(5);

    private double setSpeed = 0.5;
    
    // gets encoder angle in degress and sets soft limits
    private double pivotCurAngle = pivotEncoder.get() * 360;
    private final double pivotMinAngle = 0.0;
    private final double pivotMaxAngle = 90.0;

    public Intake(){
    // prevents intake jerk
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;

    // SparkFlexConfig pivotConfig = new SparkFlexConfig();

    leaderIntake.getConfigurator().apply(intakeConfig);
    followerIntake.getConfigurator().apply(intakeConfig);

    }
    
    public Command runIntake(){
        return new RunCommand(() -> {
            leaderIntake.set(setSpeed);
            followerIntake.set(-setSpeed);
        }
        // , this 
        );
    }

    // public boolean safeZone(){
    //     double pivotCurAngle = pivotEncoder.get() * 360;
    //     if (pivotCurAngle >= pivotMinAngle && pivotCurAngle <= pivotMaxAngle){
    //         return true; 
    //     } else { 
    //         return false; 
    //     }
    // }

    public boolean canRaise(){
        double pivotCurAngle = pivotEncoder.get() * 360;
        if (pivotCurAngle < pivotMaxAngle){
            return true; 
        } else { 
            return false; 
        }
    }

    public boolean canLower(){
        double pivotCurAngle = pivotEncoder.get() * 360;
        if (pivotCurAngle > pivotMinAngle){
            return true; 
        } else { 
            return false; 
        }
    }

    public Command lowerIntake(){
        return new RunCommand(() -> {
            if (!pivotLimitSwitch.get() && canLower()){
                pivotArm.set(0.01);  // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
            } else {
                pivotArm.set(0);
            }
        }
        // ensures when this command runs, it has sole control of the intake subsystem
        , this
        );
    }
    
    public Command raiseIntake(){
        return new RunCommand(() -> {
            if (canRaise()){
                pivotArm.set(-0.01); // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
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
