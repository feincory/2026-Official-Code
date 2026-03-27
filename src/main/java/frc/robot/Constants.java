// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  // Intake Constants
  public static double intakedeployposition = 13.9;
  public static double intakemidstopposition = 8;
  public static double intakeretractposition = 0;
  public static double intakeOscillationFrequencyHz = .7;
  public static double intakeOscillationAutoCancelSpeedMps = 1.0;
  public static int kintakehomeswt = 0;

  // shooter constancts
  public static int khoodhomeswt = 1;

  // Climber Constants
  public static double climbspeed = .5;
  public static double climbDriveSpeedMetersPerSec = 0.5;

  // turret constants
  public static int potanalogport = 0;

  public static final class AutoAimConstants {
    public static final String LATERAL_OFFSET_INCHES_KEY = "AutoAim/LateralOffsetInches";
    public static final String DISTANCE_OFFSET_INCHES_KEY = "AutoAim/DistanceOffsetInches";
    public static final String TURRET_ROTATION_LEAD_SECONDS_KEY = "AutoAim/TurretRotationLeadSec";
    public static final String TURRET_ROTATION_COMP_SCALE_KEY = "AutoAim/TurretRotationCompScale";
    public static final String TURRET_READY_TOLERANCE_DEG_KEY = "AutoAim/TurretReadyToleranceDeg";
    public static final String SHOOTER_READY_TOLERANCE_RPS_KEY = "AutoAim/ShooterReadyToleranceRps";
    public static final String AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY =
        "AutoAim/AutoShootMaxRobotSpeedMps";
    public static final String ZONE_SPLIT_Y_METERS_KEY = "AutoAim/ZoneSplitY";
    public static final String OWN_ZONE_MAX_X_METERS_KEY = "AutoAim/OwnZoneMaxX";
    public static final String PASS_NO_SHOOT_CENTER_Y_KEY = "AutoAim/PassNoShootCenterY";
    public static final String PASS_NO_SHOOT_WIDTH_METERS_KEY = "AutoAim/PassNoShootWidthMeters";
    public static final String BLUE_PASS_BOTTOM_X_KEY = "AutoAim/BluePassBottomX";
    public static final String BLUE_PASS_BOTTOM_Y_KEY = "AutoAim/BluePassBottomY";
    public static final String BLUE_PASS_TOP_X_KEY = "AutoAim/BluePassTopX";
    public static final String BLUE_PASS_TOP_Y_KEY = "AutoAim/BluePassTopY";

    public static final double FIELD_LENGTH_METERS = 16.541;
    public static final double LATERAL_OFFSET_INCHES_DEFAULT = 0;
    public static final double DISTANCE_OFFSET_INCHES_DEFAULT = 6;
    public static final double TURRET_ROTATION_LEAD_SECONDS_DEFAULT = 0.03;
    public static final double TURRET_ROTATION_COMP_SCALE_DEFAULT = 2.5;
    public static final double TURRET_READY_TOLERANCE_DEG_DEFAULT = 2.54;
    public static final double SHOOTER_READY_TOLERANCE_RPS_DEFAULT = 3.0;
    public static final double AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT = 0.15;
    public static final double ZONE_SPLIT_Y_METERS_DEFAULT = 4.0;
    public static final double OWN_ZONE_MAX_X_METERS_DEFAULT = 4.25;
    public static final double PASS_NO_SHOOT_CENTER_Y_DEFAULT = 4.03;
    public static final double PASS_NO_SHOOT_WIDTH_METERS_DEFAULT = 0.90;
    public static final Translation2d BLUE_PASS_TARGET_BOTTOM_DEFAULT_FIELD_METERS =
        new Translation2d(3.5, 2.0);
    public static final Translation2d BLUE_PASS_TARGET_TOP_DEFAULT_FIELD_METERS =
        new Translation2d(3.5, 5.1);

    private AutoAimConstants() {}
  }

  public static final class MovingShotConstants {
    public static final String LATERAL_OFFSET_INCHES_KEY = "MovingShot/LateralOffsetInches";
    public static final String DISTANCE_OFFSET_INCHES_KEY = "MovingShot/DistanceOffsetInches";
    public static final String TURRET_ROTATION_LEAD_SECONDS_KEY =
        "MovingShot/TurretRotationLeadSec";
    public static final String TURRET_ROTATION_COMP_SCALE_KEY =
        "MovingShot/TurretRotationCompScale";
    public static final String TURRET_READY_TOLERANCE_DEG_KEY =
        "MovingShot/TurretReadyToleranceDeg";
    public static final String SHOOTER_READY_TOLERANCE_RPS_KEY =
        "MovingShot/ShooterReadyToleranceRps";
    public static final String AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY =
        "MovingShot/AutoShootMaxRobotSpeedMps";
    public static final String DRIVER_MAX_LINEAR_SCALE_KEY = "MovingShot/DriverMaxLinearScale";
    public static final String DRIVER_MAX_OMEGA_SCALE_KEY = "MovingShot/DriverMaxOmegaScale";
    public static final String TARGET_LEAD_SECONDS_KEY = "MovingShot/TargetLeadSec";
    public static final String USE_MOTION_COMPENSATION_KEY = "MovingShot/UseMotionCompensation";

    public static final double LATERAL_OFFSET_INCHES_DEFAULT = 0;
    public static final double DISTANCE_OFFSET_INCHES_DEFAULT = 6;
    public static final double TURRET_ROTATION_LEAD_SECONDS_DEFAULT = 0.03;
    public static final double TURRET_ROTATION_COMP_SCALE_DEFAULT = 5.0;
    public static final double TURRET_READY_TOLERANCE_DEG_DEFAULT = 3.75;
    // AutoAimConstants.TURRET_READY_TOLERANCE_DEG_DEFAULT;
    public static final double SHOOTER_READY_TOLERANCE_RPS_DEFAULT =
        AutoAimConstants.SHOOTER_READY_TOLERANCE_RPS_DEFAULT;
    public static final double AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT = 0.6;
    public static final double DRIVER_MAX_LINEAR_SCALE_DEFAULT = 0.10;
    public static final double DRIVER_MAX_OMEGA_SCALE_DEFAULT = 0.10;
    public static final double TARGET_LEAD_SECONDS_DEFAULT = 1.8;
    public static final boolean USE_MOTION_COMPENSATION_DEFAULT = true;

    private MovingShotConstants() {}
  }

  public static final class MovingPassShotConstants {
    public static final String LATERAL_OFFSET_INCHES_KEY = "MovingPassShot/LateralOffsetInches";
    public static final String DISTANCE_OFFSET_INCHES_KEY = "MovingPassShot/DistanceOffsetInches";
    public static final String TURRET_ROTATION_LEAD_SECONDS_KEY =
        "MovingPassShot/TurretRotationLeadSec";
    public static final String TURRET_ROTATION_COMP_SCALE_KEY =
        "MovingPassShot/TurretRotationCompScale";
    public static final String TURRET_READY_TOLERANCE_DEG_KEY =
        "MovingPassShot/TurretReadyToleranceDeg";
    public static final String SHOOTER_READY_TOLERANCE_RPS_KEY =
        "MovingPassShot/ShooterReadyToleranceRps";
    public static final String AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY =
        "MovingPassShot/AutoShootMaxRobotSpeedMps";
    public static final String DRIVER_MAX_LINEAR_SCALE_KEY = "MovingPassShot/DriverMaxLinearScale";
    public static final String DRIVER_MAX_OMEGA_SCALE_KEY = "MovingPassShot/DriverMaxOmegaScale";
    public static final String TARGET_LEAD_SECONDS_KEY = "MovingPassShot/TargetLeadSec";
    public static final String USE_MOTION_COMPENSATION_KEY = "MovingPassShot/UseMotionCompensation";

    public static final double LATERAL_OFFSET_INCHES_DEFAULT = 0;
    public static final double DISTANCE_OFFSET_INCHES_DEFAULT = 10;
    public static final double TURRET_ROTATION_LEAD_SECONDS_DEFAULT = 0.03;
    public static final double TURRET_ROTATION_COMP_SCALE_DEFAULT = 6.0;
    public static final double TURRET_READY_TOLERANCE_DEG_DEFAULT = 7;

    public static final double SHOOTER_READY_TOLERANCE_RPS_DEFAULT = 8;

    public static final double AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT = 2.0;
    public static final double DRIVER_MAX_LINEAR_SCALE_DEFAULT = 0.35;
    public static final double DRIVER_MAX_OMEGA_SCALE_DEFAULT = 0.4;
    public static final double TARGET_LEAD_SECONDS_DEFAULT = 1.5;
    public static final boolean USE_MOTION_COMPENSATION_DEFAULT = true;

    private MovingPassShotConstants() {}
  }
}
