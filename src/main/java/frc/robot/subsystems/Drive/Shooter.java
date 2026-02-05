package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;



public class Shooter extends SubsystemBase{

    private final TalonFX intakeMotor = new TalonFX(21);
    private final TalonFX shooterMotor = new TalonFX(23);
    private final TalonFX MotorFollower = new TalonFX(24);
    private final TalonFX MotorFollower2 = new TalonFX(25);

    // Bang-bang control parameters
    private double shooterTargetVelocity = 0; // Target velocity in rotations per second
    private final double BANG_BANG_ON_POWER = 1.0; // Full power when below setpoint
    private final double BANG_BANG_OFF_POWER = 0.00; // Power when at/above setpoint

    public Shooter() {
    //     // Configure shooter motors to coast mode for better bang-bang control
    //     shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    //     MotorFollower.setNeutralMode(NeutralModeValue.Coast);
    //     MotorFollower2.setNeutralMode(NeutralModeValue.Coast);

    //     // Intake can stay in brake mode for better control
    //     intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    //     // Configure current limiting to protect motors
    //     CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
    //         .withSupplyCurrentLimit(40)           // Supply current limit (amps)
    //         .withSupplyCurrentLimitEnable(true)   // Enable supply current limiting
    //         .withStatorCurrentLimit(60)           // Stator current limit (amps)
    //         .withStatorCurrentLimitEnable(true);  // Enable stator current limiting

    //     shooterMotor.getConfigurator().apply(currentLimits);

    //     MotorFollower.getConfigurator().apply(currentLimits);
    //     MotorFollower2.getConfigurator().apply(currentLimits);
    //     intakeMotor.getConfigurator().apply(currentLimits);
        // in init function, set slot 0 gains
        // PID Controller for shooter
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        shooterMotor.getConfigurator().apply(slot0Configs);
        MotorFollower.getConfigurator().apply(slot0Configs);
        MotorFollower2.getConfigurator().apply(slot0Configs);


    }
    


    public Command runShooterIntake(CommandXboxController controllerValue){
        return new RunCommand(() -> 
            intakeMotor.set(controllerValue.getRightTriggerAxis())
            // ,this
    );}

    
    public Command runReverseShooterIntake(CommandXboxController controllerValue){
        return new RunCommand(() -> 
            intakeMotor.set(-1)
            // ,this
    );}

    public Command runShooter(CommandXboxController controllerValue){
        return new RunCommand(()-> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooterMotor.set(-speed);
            MotorFollower.set(speed);
            MotorFollower2.set(-speed);
        }
        // ,this
        );
    };

    public Command stopShooterIntake(){
        return new InstantCommand(()->
            intakeMotor.set(0)
            // ,this
        );
    }
    public Command stopShooter(){
        return new InstantCommand(()->{
            shooterMotor.set(0);
            MotorFollower.set(0);
            MotorFollower2.set(0);
        }
        ,this
        );
    }

    public Command stopAll(){
        return new RunCommand(()->{
                shooterMotor.set(0);
                MotorFollower.set(0);
                MotorFollower2.set(0);
                intakeMotor.set(0);

            },
            // ", this" makes sure that only the shooter subsystem object can only run command at a time
            this
        );
    }

    // Bang-bang control methods

    /**
     * Sets the target velocity for bang-bang control
     * @param velocityRPS Target velocity in rotations per second
     */
    public void setShooterTargetVelocity(double velocityRPS) {
        this.shooterTargetVelocity = velocityRPS;
    }

    /**
     * Gets the current shooter velocity
     * @return Current velocity in rotations per second
     */
    public double getShooterVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Applies bang-bang control to the shooter motor
     * Turns motor to full power if below setpoint, off if at/above setpoint
     */
    private void applyBangBangControl() {
        double currentVelocity = getShooterVelocity();

        if (currentVelocity < shooterTargetVelocity) {
            shooterMotor.set(-BANG_BANG_ON_POWER);
            MotorFollower.set(BANG_BANG_ON_POWER);
            MotorFollower2.set(-BANG_BANG_ON_POWER);
        } else {
            shooterMotor.set(BANG_BANG_OFF_POWER);
            MotorFollower.set(BANG_BANG_OFF_POWER);
            MotorFollower2.set(BANG_BANG_OFF_POWER);
        }
    }

    /**
     * Command to run shooter with bang-bang control at a target velocity
     * @param targetVelocityRPS Target velocity in rotations per second
     * @return Command that maintains the target velocity using bang-bang control
     */
    public Command runShooterBangBang(double targetVelocityRPS) {
        return new RunCommand(() -> {
            setShooterTargetVelocity(targetVelocityRPS);
            applyBangBangControl();
        }, this);
    }

    /**
     * Checks if the shooter is at the target velocity
     * @param tolerance Acceptable tolerance in rotations per second
     * @return True if within tolerance of target
     */
    public boolean isAtTargetVelocity(double tolerance) {
        return Math.abs(getShooterVelocity() - shooterTargetVelocity) <= tolerance;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter 23" , shooterMotor.get());
        SmartDashboard.putNumber("Shooter 23 motor output", shooterMotor.getMotorOutputStatus().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 23 voltage", shooterMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 23 stator current", shooterMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 23 supply voltage", shooterMotor.getSupplyVoltage().getValueAsDouble());


        SmartDashboard.putNumber("Shooter 24" , MotorFollower.get());
        SmartDashboard.putNumber("Shooter 24 motor output", MotorFollower.getMotorOutputStatus().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 24 voltage", MotorFollower.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 24 stator current", MotorFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 24 supply voltage", MotorFollower.getSupplyVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter 25" , MotorFollower2.get());
        SmartDashboard.putNumber("Shooter 25 motor output", MotorFollower2.getMotorOutputStatus().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 25 voltage", MotorFollower2.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 25 stator current", MotorFollower2.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 25 supply voltage", MotorFollower2.getSupplyVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter 21" , intakeMotor.get());
        SmartDashboard.putNumber("Shooter 21 motor output", intakeMotor.getMotorOutputStatus().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 21 voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 21 stator current", intakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 21 supply voltage", intakeMotor.getSupplyVoltage().getValueAsDouble());
    }
}
