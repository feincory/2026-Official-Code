// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class ShooterCalc {

  public static class scoringinformation {
    private final double robotx;
    private final double roboty;
    private final double goalx;
    private final double goaly;

    private final double deltax;
    private final double deltay;
    private final double distanceToGoal;
    private final double angletogoalRadians;

    private final double turretoffsetx;
    private final double turretoffsety;
    private final double turretoffsetdegrees;
    private final double gyroyaw;
    private final double turrethypot;
    private final double turretxdynamic;
    private final double turretydynamic;

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
      this.robotx = robotx;
      this.roboty = roboty;
      this.goalx = goalx;
      this.goaly = goaly;
      this.turretoffsetx = turretoffsetx;
      this.turretoffsety = turretoffsety;
      this.gyroyaw = gyroyaw;

      this.turrethypot = Math.hypot(turretoffsetx, turretoffsety);
      this.turretoffsetdegrees = Math.atan2(turretoffsetx, turretoffsety);

      this.turretxdynamic = Math.cos(turretoffsetdegrees + gyroyaw) * turrethypot;
      this.turretydynamic = Math.sin(turretoffsetdegrees + gyroyaw) * turrethypot;

      this.deltax = this.goalx - (this.robotx + turretxdynamic);
      this.deltay = this.goaly - (this.roboty + turretydynamic);
      this.distanceToGoal = Math.hypot(deltax, deltay);
      // atan2 returns angle from robot->goal relative to +X axis in radians
      this.angletogoalRadians = Math.atan2(deltay, deltax);
    }

    /** Distance from robot to goal (same units as inputs). */
    public double getDistanceToGoal() {
      System.out.print(distanceToGoal);
      return distanceToGoal;
    }

    /** Angle to goal in radians (atan2(dy, dx)). */
    public double getAngleToGoalRadians() {
      return angletogoalRadians;
    }

    /** Angle to goal in degrees. */
    public double getAngleToGoalDegrees() {
      System.out.print(angletogoalRadians);
      return Math.toDegrees(angletogoalRadians);
    }
  }
}
