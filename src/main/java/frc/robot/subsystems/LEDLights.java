package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class LEDLights extends SubsystemBase {

  /* ================= Constants ================= */

  private static final int CANDLE_ID = 50;
  private static final int LED_COUNT = 140;

  private static final double AUTO_DATA_DELAY_SEC = 4.0;
  private static final double ENDGAME_TIME_SEC = 20.0;
  private static final double WARNING_TIME_SEC = 5.0;

  private static final RGBWColor pink = RGBWColor.fromHex("#ff00bfff").orElseThrow();
  // private static final RGBWColor blue = RGBWColor.fromHex("#3b00dcff").orElseThrow();
  private static final RGBWColor yellow = RGBWColor.fromHex("#ffea00ff").orElseThrow();
  private static final RGBWColor green = RGBWColor.fromHex("#00ff15ff").orElseThrow();
  private static final RGBWColor red = RGBWColor.fromHex("#ff0000ff").orElseThrow();
  private static final RGBWColor off = RGBWColor.fromHex("#00000000").orElseThrow();
  // private static final RGBWColor purple = RGBWColor.fromHex("#6f0b9aff").orElseThrow();
  /* ================= Hub Schedule =================
   * Match time is seconds REMAINING.
   * Replace these values with the EXACT ones from the manual if needed.
   */

  // Our hub goes inactive FIRST
  private static final HubWindow[] EARLY_WINDOWS = {
    new HubWindow(135, 120), new HubWindow(105, 90), new HubWindow(75, 60)
  };

  // Our hub goes inactive SECOND
  private static final HubWindow[] LATE_WINDOWS = {
    new HubWindow(120, 105), new HubWindow(90, 75), new HubWindow(60, 45)
  };

  /* ================= Hardware ================= */

  private final CANdle candle = new CANdle(CANDLE_ID);

  /* ================= Game Data ================= */

  private boolean autoDataChecked = false;
  private boolean ourHubInactivatesFirst = false;
  private double teleopStartTime = 0.0;

  /* ================= LED States ================= */

  private enum LedState {
    ENDGAME,
    HUB_GOING_ACTIVE,
    HUB_ACTIVE,
    HUB_GOING_INACTIVE,
    HUB_INACTIVE,
    DEFAULT_RAINBOW
  }

  /* ================= Helper Class ================= */

  private static class HubWindow {
    final double start; // inclusive (sec remaining)
    final double end; // exclusive

    HubWindow(double start, double end) {
      this.start = start;
      this.end = end;
    }

    boolean isActive(double matchTime) {
      return matchTime <= start && matchTime > end;
    }
  }

  /* ================= Constructor ================= */

  public LEDLights() {
    setRainbow();
  }

  /* ================= Lifecycle ================= */

  /** Call from Robot.teleopInit() */
  public void onTeleopInit() {
    teleopStartTime = Timer.getFPGATimestamp();
    autoDataChecked = false;
  }

  /* ================= Periodic ================= */

  @Override
  public void periodic() {
    checkForAutoGameData();
    LedState state = determineState();
    applyState(state);
  }

  /* ================= Auto Game Data ================= */

  private void checkForAutoGameData() {

    if (autoDataChecked) return;

    if (Timer.getFPGATimestamp() - teleopStartTime < AUTO_DATA_DELAY_SEC) {
      return;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();

    if (gameData == null || gameData.isEmpty() || allianceOpt.isEmpty()) {
      return;
    }

    char inactiveFirst = gameData.charAt(0);
    Alliance alliance = allianceOpt.get();

    ourHubInactivatesFirst =
        (inactiveFirst == 'R' && alliance == Alliance.Red)
            || (inactiveFirst == 'B' && alliance == Alliance.Blue);

    autoDataChecked = true;
  }

  /* ================= PUBLIC OUTPUTS ================= */

  /** Output #1 */
  public boolean isHubActive() {
    double t = DriverStation.getMatchTime();
    return autoDataChecked && t > 0 && isHubActiveInternal(t);
  }

  /** Output #2 */
  public double getHubActiveRemainingTime() {
    double t = DriverStation.getMatchTime();
    if (!isHubActive()) return 0.0;

    for (HubWindow w : getWindows()) {
      if (w.isActive(t)) {
        return t - w.end;
      }
    }
    return 0.0;
  }

  /** Output #3 */
  public double getTimeUntilNextActive() {
    double t = DriverStation.getMatchTime();
    if (!autoDataChecked || t <= 0) return -1.0;

    if (isHubActiveInternal(t)) return 0.0;

    for (HubWindow w : getWindows()) {
      if (t > w.start) {
        return t - w.start;
      }
    }
    return -1.0;
  }

  /* ================= State Logic ================= */

  private LedState determineState() {

    double matchTime = DriverStation.getMatchTime();

    // Endgame
    if (matchTime > 0 && matchTime <= ENDGAME_TIME_SEC) {
      return LedState.ENDGAME;
    }

    if (autoDataChecked && matchTime > 0) {

      boolean active = isHubActiveInternal(matchTime);
      double timeToChange = active ? getHubActiveRemainingTime() : getTimeUntilNextActive();

      if (active) {
        if (timeToChange <= WARNING_TIME_SEC) {
          return LedState.HUB_GOING_INACTIVE;
        }
        return LedState.HUB_ACTIVE;
      } else {
        if (timeToChange > 0 && timeToChange <= WARNING_TIME_SEC) {
          return LedState.HUB_GOING_ACTIVE;
        }
        return LedState.HUB_INACTIVE;
      }
    }

    return LedState.DEFAULT_RAINBOW;
  }

  /* ================= Hub Logic ================= */

  private boolean isHubActiveInternal(double matchTime) {
    for (HubWindow w : getWindows()) {
      if (w.isActive(matchTime)) {
        return true;
      }
    }
    return false;
  }

  private HubWindow[] getWindows() {
    return ourHubInactivatesFirst ? EARLY_WINDOWS : LATE_WINDOWS;
  }

  /* ================= LED Rendering ================= */

  private void applyState(LedState state) {

    switch (state) {
      case ENDGAME:
        flash(pink); // pink
        break;

      case HUB_GOING_ACTIVE:
        flash(yellow); // yellow
        break;

      case HUB_ACTIVE:
        solid(green); // yellow
        break;

      case HUB_GOING_INACTIVE:
        flash(yellow); // yellow
        break;

      case HUB_INACTIVE:
        solid(red); // red
        break;

      case DEFAULT_RAINBOW:
      default:
        setRainbow();
        break;
    }
  }

  /* ================= Helpers ================= */

  private void solid(RGBWColor Color) {
    candle.setControl(new SolidColor(0, LED_COUNT).withColor(Color));
  }

  private void flash(RGBWColor Color) {
    if (flashOn()) solid(Color);
    else solid(off);
  }

  private void setRainbow() {
    candle.setControl(new RainbowAnimation(0, LED_COUNT));
  }

  private boolean flashOn() {
    return ((int) (Timer.getFPGATimestamp() * 4)) % 2 == 0;
  }
}
