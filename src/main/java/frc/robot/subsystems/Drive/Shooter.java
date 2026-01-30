package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 


public class Shooter {

        TalonFX shooter_intake = new TalonFX(22);

        TalonFX main_shooter = new TalonFX(23);
        
        double setSpeed = 0.25;
        double intakeSpeed = setSpeed;
        double shooterSpeed;


    public Command enableShooterIntake(CommandXboxController controllerValue){
        return new RunCommand(() -> {

        shooter_intake.set(controllerValue.getRightTriggerAxis());
        // m_follower.set(followerSpeed);

    });}

    public Command enableMainShooter(CommandXboxController controllerValue){
        return new RunCommand(()-> {

        // main_shooter.set(controllerValue.getLeftTriggerAxis());
        main_shooter.set(controllerValue.getLeftTriggerAxis());

    });}


    public Command stopShooterIntake(){
        return new InstantCommand(()->{
            shooter_intake.set(0);
            // m_follower.set(0);
        });
    }

    public Command stopMainShooter(){
        return new InstantCommand(()->{
            main_shooter.set(0);

        });
    }
}
