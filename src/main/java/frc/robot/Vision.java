package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;




public class Vision extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-four";

    private final SwerveDrive swerveDrive;
    private final Field2d field = new Field2d();

    public Vision(SwerveSubsystem swerve)
    {
        this.swerveDrive = swerve.getSwerveDrive();
        SmartDashboard.putData("Field", field);
    }


    private void updatePoseEstimation() {
      boolean useMegaTag2 = true;
      boolean doRejectUpdate = false;
      

    if (useMegaTag2) {
        // Send robot heading to Limelight for MegaTag2
        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME,
            swerveDrive.getYaw().getDegrees(),
            0, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        // Reject if spinning too fast
        if (Math.abs(swerveDrive.getGyro().getYawAngularVelocity().magnitude()) > 720) {
            doRejectUpdate = true;
        }
        // Reject if no tags seen
        if (mt2 == null || mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            // Very low standard deviations = trust vision heavily
            swerveDrive.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds,
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5))
            );
        }

    } else {
        // MegaTag1 fallback
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        if (mt1 == null || mt1.tagCount == 0) {
            doRejectUpdate = true;
        } else if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            // Single tag - check quality
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }

        if (!doRejectUpdate) {
            swerveDrive.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds,
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5))
            );
        }
    }
}


  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {
    

    if (!LimelightHelpers.getTV("limelight-four")) {
      return 0.0; // if we don't have a valid target, don't turn at all
    }
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .002;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    double targetingAngularVelocity = LimelightHelpers.getTX(LIMELIGHT_NAME) * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DrivebaseConstants.kMaxAngularSpeedRadiansPerSecond;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= 1.0;

    return targetingAngularVelocity;
  }

  // Target TA value - tune this to your desired stopping distance
  private static final double TARGET_TA = 0.8;      
  private static final double TA_TOLERANCE = 0.08;  // stops too far away? decrease. jitters at the target? increase. decrease this     stop when within this tolerance
  private static final double MIN_SPEED = 0.0;      // robot doesnt reach target? increase this    minimum speed to overcome friction
  private static final double MAX_SPEED = 0.06;      // robot moves too fast? decrease this    maximum speed cap

  // Range control - drives until TA reaches TARGET_TA
  double limelight_range_proportional()
  {
    if (!LimelightHelpers.getTV("limelight-four")) {
      return 0.0;
    }

    double currentTA = LimelightHelpers.getTA(LIMELIGHT_NAME);
    double error = currentTA - TARGET_TA;  // positive = move forward, negative = move back

    // If within tolerance, stop
    if (Math.abs(error) < TA_TOLERANCE) {
      return 0.0;
    }

    double kP = 0.5;
    double targetingForwardSpeed = error * kP;

    // Apply min/max limits while preserving direction
    double sign = Math.signum(targetingForwardSpeed);
    double magnitude = Math.abs(targetingForwardSpeed);
    magnitude = Math.max(MIN_SPEED, Math.min(MAX_SPEED, magnitude));
    targetingForwardSpeed = sign * magnitude;

    targetingForwardSpeed *= DrivebaseConstants.kMaxSpeedMetersPerSecond;

    return targetingForwardSpeed;
  }

  @Override
  public void periodic() {

    updatePoseEstimation();
    SmartDashboard.putNumber("Vision/Aim Output", limelight_aim_proportional());
    SmartDashboard.putNumber("Vision/Range Output", limelight_range_proportional());
    SmartDashboard.putNumber("Vision/Raw TX", LimelightHelpers.getTX(LIMELIGHT_NAME));
    SmartDashboard.putNumber("Vision/Raw TY", LimelightHelpers.getTY(LIMELIGHT_NAME));

    // Update Field2d with current robot pose
    Pose2d swervePose = swerveDrive.getPose();
    field.setRobotPose(swervePose);

    // Debug: compare raw Limelight pose vs fused swerve pose
    if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
      Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);

      // Show raw limelight pose as a separate object on the field
      field.getObject("Limelight Pose").setPose(limelightPose);

      SmartDashboard.putNumber("Vision/LL Pose X", limelightPose.getX());
      SmartDashboard.putNumber("Vision/LL Pose Y", limelightPose.getY());
      SmartDashboard.putNumber("Vision/Swerve Pose X", swervePose.getX());
      SmartDashboard.putNumber("Vision/Swerve Pose Y", swervePose.getY());
    }
  }
}
