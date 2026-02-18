package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShooterCalc {
  private double lateralAimOffsetMeters = 0;
  private double distanceOffsetMeters = 0;

  /** Result for one auto-aim/auto-shot calculation. */
  public static class ShotSolution {
    private final double distanceMeters;
    private final double effectiveDistanceMeters;
    private final Translation2d turretFieldPosition;
    private final Rotation2d goalBearingField;
    private final Rotation2d turretAngleRobotFrame;
    private final double turretCommandDegrees;
    private final double shooterRps;
    private final double hoodPercent;

    public ShotSolution(
        double distanceMeters,
        double effectiveDistanceMeters,
        Translation2d turretFieldPosition,
        Rotation2d goalBearingField,
        Rotation2d turretAngleRobotFrame,
        double turretCommandDegrees,
        double shooterRps,
        double hoodPercent) {
      this.distanceMeters = distanceMeters;
      this.effectiveDistanceMeters = effectiveDistanceMeters;
      this.turretFieldPosition = turretFieldPosition;
      this.goalBearingField = goalBearingField;
      this.turretAngleRobotFrame = turretAngleRobotFrame;
      this.turretCommandDegrees = turretCommandDegrees;
      this.shooterRps = shooterRps;
      this.hoodPercent = hoodPercent;
    }

    public double getDistanceMeters() {
      return distanceMeters;
    }

    public double getEffectiveDistanceMeters() {
      return effectiveDistanceMeters;
    }

    public Translation2d getTurretFieldPosition() {
      return turretFieldPosition;
    }

    public Rotation2d getGoalBearingField() {
      return goalBearingField;
    }

    public Rotation2d getTurretAngleRobotFrame() {
      return turretAngleRobotFrame;
    }

    public double getTurretCommandDegrees() {
      return turretCommandDegrees;
    }

    public double getShooterRps() {
      return shooterRps;
    }

    public double getHoodPercent() {
      return hoodPercent;
    }

    /**
     * @deprecated Use {@link #getShooterRps()}
     */
    @Deprecated
    public double getShooterRpm() {
      return getShooterRps();
    }

    /**
     * @deprecated Use {@link #getHoodPercent()}
     */
    @Deprecated
    public double getHoodAngleDegrees() {
      return getHoodPercent();
    }
  }

  private final Translation2d turretOffsetFromRobotCenterMeters;
  private final Rotation2d turretZeroDirectionInRobotFrame;
  private final boolean turretPositiveClockwise;
  private final double turretMinCommandDegrees;
  private final double turretMaxCommandDegrees;
  private final NavigableMap<Double, Double> shooterRpsByDistanceMeters;
  private final NavigableMap<Double, Double> hoodPercentByDistanceMeters;

  /**
   * Default constructor. Robot-specific values should be tuned for your mechanism.
   *
   * <p>Offset convention: +X is robot forward, +Y is robot left.
   */
  public ShooterCalc() {
    this(
        new Translation2d(0.0, 0.0),
        Rotation2d.fromDegrees(180.0),
        true,
        -225.0,
        225.0,
        defaultShooterRpsTable(),
        defaultHoodTable());
  }

  public ShooterCalc(
      Translation2d turretOffsetFromRobotCenterMeters,
      Rotation2d turretZeroDirectionInRobotFrame,
      boolean turretPositiveClockwise,
      double turretMinCommandDegrees,
      double turretMaxCommandDegrees,
      NavigableMap<Double, Double> shooterRpsByDistanceMeters,
      NavigableMap<Double, Double> hoodPercentByDistanceMeters) {
    this.turretOffsetFromRobotCenterMeters = turretOffsetFromRobotCenterMeters;
    this.turretZeroDirectionInRobotFrame = turretZeroDirectionInRobotFrame;
    this.turretPositiveClockwise = turretPositiveClockwise;
    this.turretMinCommandDegrees = turretMinCommandDegrees;
    this.turretMaxCommandDegrees = turretMaxCommandDegrees;
    this.shooterRpsByDistanceMeters = new TreeMap<>(shooterRpsByDistanceMeters);
    this.hoodPercentByDistanceMeters = new TreeMap<>(hoodPercentByDistanceMeters);
  }

  /**
   * Main shot calculation using field pose and known goal center.
   *
   * @param robotPose field-relative pose from odometry/vision fusion
   * @param goalCenterField goal center position in field coordinates (meters)
   */
  public ShotSolution calculateShot(Pose2d robotPose, Translation2d goalCenterField) {
    // Rotate robot-relative turret offset into the field frame.
    Translation2d turretFieldOffset =
        turretOffsetFromRobotCenterMeters.rotateBy(robotPose.getRotation());
    Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretFieldOffset);

    Translation2d turretToGoalField = goalCenterField.minus(turretFieldPosition);
    if (Math.abs(lateralAimOffsetMeters) > 1e-9 && turretToGoalField.getNorm() > 1e-9) {
      // Positive offset means "aim left" from the shooter's point of view.
      Translation2d leftUnitVector =
          new Translation2d(-turretToGoalField.getY(), turretToGoalField.getX())
              .div(turretToGoalField.getNorm());
      Translation2d adjustedGoal =
          goalCenterField.plus(leftUnitVector.times(lateralAimOffsetMeters));
      turretToGoalField = adjustedGoal.minus(turretFieldPosition);
    }
    double distanceMeters = turretToGoalField.getNorm();
    Rotation2d goalBearingField =
        new Rotation2d(Math.atan2(turretToGoalField.getY(), turretToGoalField.getX()));

    // Angle from robot forward (+X) to goal direction, CCW positive.
    Rotation2d turretAngleRobotFrame = goalBearingField.minus(robotPose.getRotation());

    // Convert to turret command space (for your subsystem's chosen sign/zero conventions).
    Rotation2d turretFromZero = turretAngleRobotFrame.minus(turretZeroDirectionInRobotFrame);
    double turretCommandDegrees = MathUtil.inputModulus(turretFromZero.getDegrees(), -180.0, 180.0);
    if (turretPositiveClockwise) {
      turretCommandDegrees = -turretCommandDegrees;
    }
    turretCommandDegrees =
        MathUtil.clamp(turretCommandDegrees, turretMinCommandDegrees, turretMaxCommandDegrees);

    double effectiveDistanceMeters = Math.max(0.0, distanceMeters + distanceOffsetMeters);
    double shooterRps = interpolate(effectiveDistanceMeters, shooterRpsByDistanceMeters);
    double hoodPercent = interpolate(effectiveDistanceMeters, hoodPercentByDistanceMeters);

    return new ShotSolution(
        distanceMeters,
        effectiveDistanceMeters,
        turretFieldPosition,
        goalBearingField,
        turretAngleRobotFrame,
        turretCommandDegrees,
        shooterRps,
        hoodPercent);
  }

  public void setLateralAimOffsetMeters(double lateralAimOffsetMeters) {
    this.lateralAimOffsetMeters = lateralAimOffsetMeters;
  }

  public double getLateralAimOffsetMeters() {
    return lateralAimOffsetMeters;
  }

  public void setDistanceOffsetMeters(double distanceOffsetMeters) {
    this.distanceOffsetMeters = distanceOffsetMeters;
  }

  public double getDistanceOffsetMeters() {
    return distanceOffsetMeters;
  }

  /**
   * Convenience method when all you have is robot XY and gyro yaw.
   *
   * @param robotXMeters robot X in field frame
   * @param robotYMeters robot Y in field frame
   * @param gyroYawDegrees field-relative robot heading
   */
  public ShotSolution calculateShot(
      double robotXMeters,
      double robotYMeters,
      double gyroYawDegrees,
      double goalXFieldMeters,
      double goalYFieldMeters) {
    return calculateShot(
        new Pose2d(robotXMeters, robotYMeters, Rotation2d.fromDegrees(gyroYawDegrees)),
        new Translation2d(goalXFieldMeters, goalYFieldMeters));
  }

  private static double interpolate(double x, NavigableMap<Double, Double> table) {
    if (table.isEmpty()) {
      return 0.0;
    }
    Map.Entry<Double, Double> lower = table.floorEntry(x);
    Map.Entry<Double, Double> upper = table.ceilingEntry(x);
    if (lower == null) {
      return upper.getValue();
    }
    if (upper == null) {
      return lower.getValue();
    }
    if (Math.abs(upper.getKey() - lower.getKey()) < 1e-9) {
      return lower.getValue();
    }
    double t = (x - lower.getKey()) / (upper.getKey() - lower.getKey());
    return lower.getValue() + t * (upper.getValue() - lower.getValue());
  }

  private static NavigableMap<Double, Double> defaultShooterRpsTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(1.5, 43.0);
    table.put(2.5, 50.0);
    table.put(3.5, 57.0);
    table.put(4.5, 63.0);
    table.put(5.5, 72.0);
    return table;
  }

  private static NavigableMap<Double, Double> defaultHoodTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(1.5, 0.25);
    table.put(2.5, 0.35);
    table.put(3.5, 0.45);
    table.put(4.5, 0.55);
    table.put(5.5, 0.65);
    return table;
  }

  @Deprecated
  public static class scoringinformation {
    private final double distanceToGoal;
    private final double angletogoalRadians;

    /**
     * Create scoring information from robot and goal positions.
     *
     * @param robotx robot X position
     * @param roboty robot Y position
     * @param goalx goal X position
     * @param goaly goal Y position
     */
    public scoringinformation(
        double robotx,
        double roboty,
        double goalx,
        double goaly,
        double turretoffsetx,
        double turretoffsety,
        double turretoffsetdegrees,
        double gyroyaw,
        double turrethypot,
        double turretxdynamic,
        double turretydynamic) {
      ShotSolution shot =
          new ShooterCalc(
                  new Translation2d(turretoffsetx, turretoffsety),
                  Rotation2d.fromDegrees(180.0),
                  true,
                  -225.0,
                  225.0,
                  defaultShooterRpsTable(),
                  defaultHoodTable())
              .calculateShot(
                  new Pose2d(robotx, roboty, Rotation2d.fromRadians(gyroyaw)),
                  new Translation2d(goalx, goaly));

      this.distanceToGoal = shot.getDistanceMeters();
      this.angletogoalRadians = shot.getGoalBearingField().getRadians();
    }

    /** Distance from robot to goal (same units as inputs). */
    public double getDistanceToGoal() {
      return distanceToGoal;
    }

    /** Angle to goal in radians (atan2(dy, dx)). */
    public double getAngleToGoalRadians() {
      return angletogoalRadians;
    }

    /** Angle to goal in degrees. */
    public double getAngleToGoalDegrees() {
      return Math.toDegrees(angletogoalRadians);
    }
  }
}
