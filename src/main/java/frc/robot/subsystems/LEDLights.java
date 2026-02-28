package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class LEDLights extends SubsystemBase {
  private static final int kCandleId = 50;
  private static final String kCanBus = "canivore";
  private static final int kLedStart = 0;
  private static final int kLedCount = 140;

  private static final double kTransitionEnd = 130.0;
  private static final double kShift1End = 105.0;
  private static final double kShift2End = 80.0;
  private static final double kShift3End = 55.0;
  private static final double kShift4End = 30.0;

  private static final double kTeleopGameDataReadyTime = 136.0;
  private static final double kWarningSeconds = 5.0;
  private static final double kEndgameWarningStart = 20.0;
  private static final double kEndgameStart = 15.0;

  // Flip this if field behavior proves the mapping is opposite.
  private static final boolean kWinWhenGameDataNotAlliance = true;

  public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  public static final RGBWColor WHITE = new RGBWColor(255, 255, 255);
  public static final RGBWColor PINK = RGBWColor.fromHex("#ff00bfff").orElseThrow();
  public static final RGBWColor YELLOW = RGBWColor.fromHex("#ffea00ff").orElseThrow();
  public static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);

  private final CANdle candle = new CANdle(kCandleId, kCanBus);

  private LedMode currentMode = null;
  private double teleopStartMatchTime = -1.0;

  private enum LedMode {
    RAINBOW,
    SOLID_GREEN,
    SOLID_BLUE,
    FLASH_GREEN,
    FLASH_PINK,
    FLASH_YELLOW,
    FLASH_WHITE
  }

  private enum TeleopWindow {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    SHIFT5,
    ENDGAME,
    COMPLETE
  }

  private static class HubTiming {
    final boolean active;
    final double activeRemainingSec;
    final double nextActiveSec;

    HubTiming(boolean active, double activeRemainingSec, double nextActiveSec) {
      this.active = active;
      this.activeRemainingSec = Math.max(activeRemainingSec, 0.0);
      this.nextActiveSec = Math.max(nextActiveSec, 0.0);
    }
  }

  public LEDLights() {
    var config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 0.5;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(config);

    for (int i = 0; i < 8; i++) {
      candle.setControl(new EmptyAnimation(i));
    }

    applyMode(LedMode.RAINBOW);
  }

  public void onTeleopInit() {
    teleopStartMatchTime = DriverStation.getMatchTime();
  }

  private void solid(RGBWColor color) {
    candle.setControl(new SolidColor(kLedStart, kLedCount).withColor(color));
  }

  private void flash(RGBWColor color) {
    candle.setControl(
        new com.ctre.phoenix6.controls.StrobeAnimation(kLedStart, kLedCount)
            .withSlot(0)
            .withColor(color)
            .withFrameRate(5));
  }

  private void rainbow() {
    candle.setControl(new RainbowAnimation(kLedStart, kLedCount).withSlot(0));
  }

  private void applyMode(LedMode mode) {
    if (mode == currentMode) {
      return;
    }

    candle.setControl(new EmptyAnimation(0));
    switch (mode) {
      case RAINBOW -> rainbow();
      case SOLID_GREEN -> solid(GREEN);
      case SOLID_BLUE -> solid(BLUE);
      case FLASH_GREEN -> flash(GREEN);
      case FLASH_PINK -> flash(PINK);
      case FLASH_YELLOW -> flash(YELLOW);
      case FLASH_WHITE -> flash(WHITE);
    }
    currentMode = mode;
  }

  private TeleopWindow getTeleopWindow(double matchTimeSec) {
    if (matchTimeSec > kTransitionEnd) return TeleopWindow.TRANSITION;
    if (matchTimeSec > kShift1End) return TeleopWindow.SHIFT1;
    if (matchTimeSec > kShift2End) return TeleopWindow.SHIFT2;
    if (matchTimeSec > kShift3End) return TeleopWindow.SHIFT3;
    if (matchTimeSec > kShift4End) return TeleopWindow.SHIFT4;
    if (matchTimeSec > 0.0) return TeleopWindow.ENDGAME;
    return TeleopWindow.COMPLETE;
  }

  private Optional<Boolean> didWeWinAuto() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    String gameData = DriverStation.getGameSpecificMessage();

    if (alliance.isEmpty() || gameData.isEmpty()) {
      return Optional.empty();
    }

    char allianceChar = alliance.get() == Alliance.Red ? 'R' : 'B';
    char gameChar = Character.toUpperCase(gameData.charAt(0));
    if (gameChar != 'R' && gameChar != 'B') {
      return Optional.empty();
    }

    boolean wonAuto =
        kWinWhenGameDataNotAlliance ? (gameChar != allianceChar) : (gameChar == allianceChar);

    SmartDashboard.putString("LED/AllianceChar", String.valueOf(allianceChar));
    SmartDashboard.putString("LED/GameDataChar", String.valueOf(gameChar));
    SmartDashboard.putBoolean("LED/WonAutoFromGameData", wonAuto);
    return Optional.of(wonAuto);
  }

  private Optional<Boolean> isHubActiveForAlliance(double matchTimeSec) {
    Optional<Boolean> wonAutoOpt = didWeWinAuto();
    if (wonAutoOpt.isEmpty()) {
      return Optional.empty();
    }

    boolean shift1Active = wonAutoOpt.get();
    return switch (getTeleopWindow(matchTimeSec)) {
      case TRANSITION, COMPLETE -> Optional.of(false);
      case SHIFT1 -> Optional.of(shift1Active);
      case SHIFT2 -> Optional.of(!shift1Active);
      case SHIFT3 -> Optional.of(shift1Active);
      case SHIFT4 -> Optional.of(!shift1Active);
      case SHIFT5 -> Optional.of(shift1Active);
      case ENDGAME -> Optional.of(true);
    };
  }

  private HubTiming getHubTiming(double matchTimeSec) {
    Optional<Boolean> activeOpt = isHubActiveForAlliance(matchTimeSec);
    if (activeOpt.isEmpty()) {
      return new HubTiming(false, 0.0, 0.0);
    }

    boolean active = activeOpt.get();
    TeleopWindow window = getTeleopWindow(matchTimeSec);
    if (active) {
      double remaining =
          switch (window) {
            case SHIFT1 -> matchTimeSec - kShift1End;
            case SHIFT2 -> matchTimeSec - kShift2End;
            case SHIFT3 -> matchTimeSec - kShift3End;
            case SHIFT4 -> matchTimeSec - kShift4End;
            default -> 0.0;
          };
      return new HubTiming(true, remaining, 0.0);
    }

    double nextActive =
        switch (window) {
          case TRANSITION -> matchTimeSec - kTransitionEnd;
          case SHIFT1 -> matchTimeSec - kShift1End;
          case SHIFT2 -> matchTimeSec - kShift2End;
          case SHIFT3 -> matchTimeSec - kShift3End;
          case SHIFT4 -> matchTimeSec - kShift4End;
          default -> 0.0;
        };
    return new HubTiming(false, 0.0, nextActive);
  }

  private LedMode getTeleopMode(double matchTimeSec) {
    if (matchTimeSec > kTeleopGameDataReadyTime) {
      return LedMode.RAINBOW;
    }

    if (getTeleopWindow(matchTimeSec) == TeleopWindow.TRANSITION) {
      Optional<Boolean> wonAutoOpt = didWeWinAuto();
      if (wonAutoOpt.isEmpty()) {
        return LedMode.RAINBOW;
      }
      return wonAutoOpt.get() ? LedMode.FLASH_GREEN : LedMode.FLASH_WHITE;
    }

    HubTiming timing = getHubTiming(matchTimeSec);
    SmartDashboard.putBoolean("LED/HubActive", timing.active);
    SmartDashboard.putNumber("LED/ActiveRemainingSec", timing.activeRemainingSec);
    SmartDashboard.putNumber("LED/NextActiveInSec", timing.nextActiveSec);

    if (matchTimeSec <= kEndgameWarningStart && matchTimeSec > kEndgameStart) {
      return LedMode.FLASH_PINK;
    }

    if (timing.active) {
      if (timing.activeRemainingSec <= kWarningSeconds) {
        if (getTeleopWindow(matchTimeSec) == TeleopWindow.ENDGAME) {
          return LedMode.SOLID_GREEN;
        }
        return LedMode.FLASH_GREEN;
      }
      return LedMode.SOLID_GREEN;
    }

    if (timing.nextActiveSec > 0.0 && timing.nextActiveSec <= kWarningSeconds) {
      return LedMode.FLASH_YELLOW;
    }

    return LedMode.SOLID_BLUE;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !DriverStation.isTeleopEnabled()) {
      SmartDashboard.putBoolean("LED/HubActive", false);
      SmartDashboard.putNumber("LED/ActiveRemainingSec", 0.0);
      SmartDashboard.putNumber("LED/NextActiveInSec", 0.0);
      applyMode(LedMode.RAINBOW);
      return;
    }

    double matchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("LED/TeleopStartMatchTime", teleopStartMatchTime);
    applyMode(getTeleopMode(matchTime));
  }
}
