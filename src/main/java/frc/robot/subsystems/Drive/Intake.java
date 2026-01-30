package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake extends SubsystemBase{
    
    private final TalonFX intakeMotor = new TalonFX(24);
    private final TalonFX shooterMotor = new TalonFX(25);
    private final CANSparkFlex pivotArm = new SparkMax(26, MotorType.kBrushless);

}
