
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{
  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;

  /**
   * Initializes the {@link SwerveDrive} from the given JSON config directory.
   *
   * <p>The robot starts at the field origin (0, 0, 0°). Call {@link #resetPoseForAlliance()} once
   * the Driver Station has reported the alliance (e.g., from a trigger on DS attachment or in
   * {@code autonomousInit}) to place the robot on the correct half of the field. PathPlanner
   * automatically resets the pose to the path start when an autonomous routine begins, so this
   * mainly matters for teleop-only operation.
   *
   * @param directory Folder containing the YAGSL swerve JSON config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Telemetry verbosity controls how much data is published to NetworkTables.
    // HIGH is useful during development/tuning but can cause loop overruns in competition.
    // Use LOW or MACHINE for competition matches.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

    try
    {
      // Use the field origin as a safe default; pose is corrected later by resetPoseForAlliance()
      // or by PathPlanner at the start of each autonomous routine.
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, new Pose2d());
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    // Heading correction steers toward a locked heading when no rotation input is given.
    // Only enable when controlling the robot by absolute angle, not angular velocity.
    swerveDrive.setHeadingCorrection(false);

    // Cosine compensation reduces each module's drive speed proportional to its angle error,
    // which smooths motion but produces discrepancies in simulation that don't match real life.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

    // Counteracts translational skew that worsens as angular velocity increases.
    // The coefficient (0.1) is a starting point — increase it if skew is still noticeable.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

    // When enabled, absolute encoders resynchronize with motor encoders periodically
    // while modules are stationary. Useful if absolute encoders drift over time.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    // Register this drivebase with PathPlanner so it can execute generated paths.
    setupPathPlanner();
  }

  /**
   * Constructs the swerve drive from explicit configuration objects instead of JSON files.
   * Used primarily in unit tests or simulation environments where deploying JSON is impractical.
   *
   * @param driveCfg      {@link SwerveDriveConfiguration} describing the physical module layout.
   * @param controllerCfg Swerve controller (heading PID) configuration.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED, new Pose2d());
  }

  /**
   * Configures PathPlanner's {@link AutoBuilder} so the drivebase can follow generated paths.
   *
   * <p>Robot configuration is loaded from the PathPlanner GUI settings file deployed with the
   * robot code. If the settings file is missing or malformed a {@link RuntimeException} is thrown
   * at startup so the problem is caught immediately rather than silently producing wrong behaviour
   * during autonomous.
   *
   * <p>Translation and rotation PID gains (5.0, 0, 0) are conservative starting values.
   * Tune kP upward during path-following tests until tracking is accurate without oscillation.
   */
  private void setupPathPlanner()
  {
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e)
    {
      throw new RuntimeException(
          "Failed to load PathPlanner robot config. "
          + "Configure the robot in the PathPlanner app and redeploy.", e);
    }

    AutoBuilder.configure(
        this::getPose,                       // Supplies current robot pose to PathPlanner
        this::resetOdometry,                 // Resets odometry to PathPlanner's starting pose
        this::getRobotVelocity,              // Supplies current robot-relative chassis speeds
        (speeds, feedforwards) -> drive(speeds), // Drives the robot at the commanded speeds
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0, 0),     // Translation PID — tune kP first
            new PIDConstants(5.0, 0, 0)      // Rotation PID    — tune kP first
        ),
        config,
        this::isRedAlliance,                 // Mirrors paths automatically for red alliance
        this                                 // This subsystem is required by path-following commands
    );
  }

  /**
   * Resets the robot's odometry to the correct alliance-relative starting position.
   *
   * <p>Call this once the Driver Station has reported the alliance (e.g., from a
   * {@link edu.wpi.first.wpilibj2.command.button.RobotModeTriggers} trigger or at the start of
   * {@code autonomousInit}). PathPlanner autos reset the pose automatically, so this is mainly
   * useful for teleop-only matches or pre-match setup.
   *
   * <ul>
   *   <li>Blue alliance: x = 1 m, y = 4 m, heading = 0°
   *   <li>Red alliance:  x = 16 m, y = 4 m, heading = 180°
   * </ul>
   */
  public void resetPoseForAlliance()
  {
    boolean blueAlliance = DriverStation.getAlliance().isPresent()
                           && DriverStation.getAlliance().get() == Alliance.Blue;
    Pose2d startingPose = blueAlliance
        ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
        : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180));
    resetOdometry(startingPose);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Generates a SysId command sequence that characterizes the drive motors.
   * Run this during a test session to measure kS, kV, and kA for the drive feedforward.
   *
   * @return SysId drive characterization command.
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Generates a SysId command sequence that characterizes the angle (steering) motors.
   * Run this during a test session to measure kS, kV, and kA for the angle feedforward.
   *
   * @return SysId angle characterization command.
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a command that steers all swerve modules to 0° (facing forward).
   * Useful for verifying module zero offsets after re-calibrating absolute encoders.
   *
   * @return Command that centers all modules.
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a command that drives the robot straight forward at 1 m/s (robot-relative) until
   * the command is cancelled, then coasts to a stop.
   * Intended for simple drive-forward autonomous routines.
   *
   * @return Command that drives forward until interrupted.
   */
  public Command driveForward()
  {
    return run(() -> swerveDrive.drive(new Translation2d(1, 0), 0, false, false))
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Replaces the feedforward model used for all swerve drive motors.
   * Apply updated gains here after running SysId characterization.
   *
   * @param kS Static gain: minimum voltage to overcome friction (V).
   * @param kV Velocity gain: voltage per m/s.
   * @param kA Acceleration gain: voltage per m/s².
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using joystick translation and angular velocity inputs.
   * All three axes are cubed so the robot responds gently near the stick center and
   * reaches full speed only at the extremes.
   *
   * @param translationX     X-axis stick input (forward/backward), in [-1, 1].
   * @param translationY     Y-axis stick input (strafe left/right), in [-1, 1].
   * @param angularRotationX Rotation stick input (CCW positive), in [-1, 1].
   * @return Field-oriented drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                              DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Cube normalized inputs for smooth low-speed control, then scale to m/s.
      Translation2d cubedInputs = SwerveMath.cubeTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()));

      swerveDrive.drive(
          new Translation2d(
              cubedInputs.getX() * swerveDrive.getMaximumChassisVelocity(),
              cubedInputs.getY() * swerveDrive.getMaximumChassisVelocity()),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * Command to drive the robot using joystick translation and a heading setpoint defined by a
   * second joystick. The right joystick X/Y components define the desired facing direction;
   * the swerve controller holds that heading with a PID loop. Translation inputs are cubed for
   * smooth low-speed response.
   *
   * @param translationX X-axis stick input (forward/backward), in [-1, 1].
   * @param translationY Y-axis stick input (strafe left/right), in [-1, 1].
   * @param headingX     X component of the heading joystick (right stick X).
   * @param headingY     Y component of the heading joystick (right stick Y).
   * @return Field-oriented drive command with PID heading hold.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                              DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return run(() -> {
      Translation2d cubedInputs = SwerveMath.cubeTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()));

      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
          cubedInputs.getX(), cubedInputs.getY(),
          headingX.getAsDouble(), headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Drives the robot with the given field-relative translation and rotation using closed-loop
   * velocity control. Open-loop is disabled to maintain consistent speed under varying loads.
   *
   * @param translation   Field-relative velocity vector in m/s.
   *                      Positive X is away from the blue alliance wall (field-North);
   *                      positive Y is toward the left wall from the driver's perspective (field-West).
   * @param rotation      Angular rate in rad/s, CCW positive.
   * @param fieldRelative {@code true} for field-relative control, {@code false} for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Drives the robot at the given field-oriented {@link ChassisSpeeds}.
   *
   * @param velocity Field-relative chassis speeds.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Returns a command that continuously drives the robot at field-oriented {@link ChassisSpeeds}
   * supplied by the given {@link Supplier}. Intended to be used as a default command with a
   * {@link swervelib.SwerveInputStream} supplier.
   *
   * @param velocity Supplier of field-relative chassis speeds.
   * @return Continuous field-oriented drive command.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  /**
   * Drives the robot at the given robot-relative {@link ChassisSpeeds}.
   * Used by PathPlanner and other autonomous controllers that output robot-relative commands.
   *
   * @param velocity Robot-relative chassis speeds.
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  /**
   * Returns the swerve drive kinematics object.
   * Used for odometry, path following, and module state calculations.
   *
   * @return {@link SwerveDriveKinematics} for this drivebase.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset
   * separately; however, if either is reset externally this must also be called to keep odometry
   * consistent.
   *
   * @param initialHolonomicPose The pose to which odometry is reset.
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Returns the robot's current pose (position and heading) as reported by the pose estimator.
   *
   * @return Current {@link Pose2d}.
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Sets chassis speeds directly with closed-loop velocity control, bypassing field-orientation.
   * Prefer {@link #drive(ChassisSpeeds)} for most use cases.
   *
   * @param chassisSpeeds Target robot-relative chassis speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Sends a trajectory to SmartDashboard / Shuffleboard for display on the field widget.
   *
   * @param trajectory The trajectory to display.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Zeroes the gyro and resets the stored heading to 0° (facing away from the blue alliance wall).
   * Call this while the robot is physically pointing forward before a match.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Returns {@code true} if the current alliance is red, {@code false} if blue or unknown.
   * Used to mirror PathPlanner paths for the red alliance and to set the correct starting heading.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Zeroes the gyro and adjusts the stored heading so that "forward" matches the current alliance's
   * forward direction.
   *
   * <ul>
   *   <li>Blue alliance: heading set to 0° (facing away from the blue wall).
   *   <li>Red alliance: heading set to 180° (facing away from the red wall).
   * </ul>
   */
  public void zeroGyroWithAlliance()
  {
    zeroGyro();
    if (isRedAlliance())
    {
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    }
  }

  /**
   * Sets all drive motors to brake or coast mode.
   * Brake mode is recommended while the robot is enabled to hold position.
   * Coast mode is recommended while disabled so the robot can be pushed during inspection.
   *
   * @param brake {@code true} for brake mode, {@code false} for coast mode.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Returns the robot's current heading as a {@link Rotation2d} derived from the pose estimator
   * rather than the raw gyro. Corrections applied via {@link #resetOdometry} are reflected here.
   *
   * @return Current robot heading.
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Computes target {@link ChassisSpeeds} from dual-joystick input where the right joystick
   * defines the desired facing direction. Translation inputs are cubed for smooth low-speed
   * response.
   *
   * @param xInput   Forward/backward joystick input, in [-1, 1].
   * @param yInput   Strafe joystick input, in [-1, 1].
   * @param headingX X component of the heading joystick.
   * @param headingY Y component of the heading joystick.
   * @return {@link ChassisSpeeds} targeting the joystick-defined heading.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput,
                                       double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        headingX, headingY,
        getHeading().getRadians(), Constants.MAX_SPEED);
  }

  /**
   * Computes target {@link ChassisSpeeds} from translation joystick input and an explicit angle
   * setpoint. Translation inputs are cubed for smooth low-speed response.
   *
   * @param xInput Forward/backward joystick input, in [-1, 1].
   * @param yInput Strafe joystick input, in [-1, 1].
   * @param angle  Desired robot facing angle.
   * @return {@link ChassisSpeeds} targeting the given angle.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        angle.getRadians(), getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  /**
   * Returns the robot's current field-relative velocity (vx, vy, omega).
   *
   * @return Field-relative {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Returns the robot's current robot-relative velocity (vx, vy, omega).
   * This is the value PathPlanner uses as feedback during path following.
   *
   * @return Robot-relative {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Returns the underlying {@link SwerveController} for advanced heading control operations.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Returns the {@link SwerveDriveConfiguration} describing the physical module layout and
   * motor configuration of this drivebase.
   *
   * @return {@link SwerveDriveConfiguration} for the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Locks the swerve modules in an X pattern to resist being pushed.
   * Call this when the robot is disabled and needs to hold position on a slope.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Returns the robot's current pitch angle as reported by the IMU.
   * Useful for detecting whether the robot is climbing a ramp.
   *
   * @return Pitch angle as a {@link Rotation2d}.
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Provides direct access to the underlying {@link SwerveDrive} object for advanced operations
   * not exposed by this subsystem (e.g., configuring individual modules or reading raw sensor data).
   *
   * @return The {@link SwerveDrive} instance.
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
