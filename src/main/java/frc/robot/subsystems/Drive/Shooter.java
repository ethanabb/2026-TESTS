package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 


public class Shooter extends SubsystemBase{

    private final TalonFX intakeMotor = new TalonFX(22);
    private final TalonFX shooterMotor = new TalonFX(23);


    public Command runShooterIntake(CommandXboxController controllerValue){
        return new RunCommand(() -> 
            intakeMotor.set(controllerValue.getRightTriggerAxis())
            // ,this
    );}

    public Command runShooter(CommandXboxController controllerValue){
        return new RunCommand(()-> 
            shooterMotor.set(controllerValue.getLeftTriggerAxis())
            // ,this
    );}

    public Command stopShooterIntake(){
        return new InstantCommand(()->
            intakeMotor.set(0)
            // ,this
        );
    }
    public Command stopShooter(){
        return new InstantCommand(()->
            shooterMotor.set(0)
            // ,this
        );
    }

    public Command stopAll(){
        return new RunCommand(()->{
                shooterMotor.set(0);
                intakeMotor.set(0);
            },
            // ", this" makes sure that only the shooter subsystem object can only run command at a time
            this
        );
    }
}
