package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 


public class Shooter extends SubsystemBase{

    private final TalonFX intakeMotor = new TalonFX(20);
    private final TalonFX shooterMotor = new TalonFX(21);
    // private final TalonFX MotorFollower = new TalonFX(24);
    // private final TalonFX MotorFollower2 = new TalonFX(25);

    // Bang-bang control parameters
    private double shooterTargetVelocity = 0; // Target velocity in rotations per second
    private final double BANG_BANG_ON_POWER = 1.0; // Full power when below setpoint
    private final double BANG_BANG_OFF_POWER = 0.01; // Power when at/above setpoint

    public Shooter() {
        // Configure shooter motors to coast mode for better bang-bang control
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        // MotorFollower.setNeutralMode(NeutralModeValue.Coast);
        // MotorFollower2.setNeutralMode(NeutralModeValue.Coast);

        // Intake can stay in brake mode for better control
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        // Configure current limiting to protect motors
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)           // Supply current limit (amps)
            .withSupplyCurrentLimitEnable(true)   // Enable supply current limiting
            .withStatorCurrentLimit(60)           // Stator current limit (amps)
            .withStatorCurrentLimitEnable(true);  // Enable stator current limiting

        shooterMotor.getConfigurator().apply(currentLimits);
        // MotorFollower.getConfigurator().apply(currentLimits);
        // MotorFollower2.getConfigurator().apply(currentLimits);
        intakeMotor.getConfigurator().apply(currentLimits);
    }

    public Command runShooterIntake(CommandXboxController controllerValue){
        return new RunCommand(() -> 
            intakeMotor.set(controllerValue.getRightTriggerAxis())
            // ,this
    );}

    public Command runShooter(CommandXboxController controllerValue){
        return new RunCommand(()-> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooterMotor.set(-speed);
            // MotorFollower.set(speed);
            // MotorFollower2.set(speed);
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
        return new InstantCommand(()->
            shooterMotor.set(0)
            // MotorFollower.set(0);
            // MotorFollower2.set(0);
            // ,this
        );
    }

    public Command stopAll(){
        return new RunCommand(()->{
                shooterMotor.set(0);
                // MotorFollower.set(0);
                // MotorFollower2.set(0);
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
            shooterMotor.set(BANG_BANG_ON_POWER);
            // MotorFollower.set(BANG_BANG_ON_POWER);
            // MotorFollower2.set(BANG_BANG_ON_POWER);
        } else {
            shooterMotor.set(BANG_BANG_OFF_POWER);
            // MotorFollower.set(BANG_BANG_OFF_POWER);
            // MotorFollower2.set(BANG_BANG_OFF_POWER);
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
}
