// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
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
  private static final RGBWColor kRed = new RGBWColor(255, 0, 0);
  private static final RGBWColor kWhite = new RGBWColor(255, 255, 255);
  private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor kPink = RGBWColor.fromHex("#ff00bfff").orElseThrow();
  private static final RGBWColor kGreen = RGBWColor.fromHex("#67b13cff").orElseThrow();
  private static final RGBWColor kBlue = RGBWColor.fromHex("#2600ffff").orElseThrow();

  // 2026 teleop countdown windows (seconds remaining)
  private static final double kTransitionEnd = 130.0; // 140-130 transition
  private static final double kShift1End = 110.0;
  private static final double kShift2End = 90.0;
  private static final double kShift3End = 70.0;
  private static final double kShift4End = 50.0;
  private static final double kShift5End = 30.0;

  // Game data delivered 3s into teleop (140 -> 137 on countdown)
  private static final double kTeleopGameDataAvailableMatchTime = 137.0;

  // Set false if field behavior indicates opposite winner mapping.
  // true  => win when game data != our alliance
  // false => win when game data == our alliance
  private static final boolean kWinWhenGameDataNotAlliance = true;

  private static final int kSlot0StartIdx = 0;
  private static final int kSlot0EndIdx = 140;

  private final CANdle m_candle = new CANdle(50, "canivore");

  private enum LedMode {
    Rainbow,
    SolidRed,
    SolidBlue,
    SolidPink,
    SolidViolet,
    FlashRed,
    FlashGreen,
    FlashWhite
  }

  private enum TeleopWindow {
    Transition,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    Shift5,
    Endgame,
    Complete
  }

  private LedMode m_lastMode = null;

  private static class HubTiming {
    final boolean hubActive;
    final double activeRemainingSec;
    final double nextActiveSec;

    HubTiming(boolean hubActive, double activeRemainingSec, double nextActiveSec) {
      this.hubActive = hubActive;
      this.activeRemainingSec = Math.max(activeRemainingSec, 0.0);
      this.nextActiveSec = Math.max(nextActiveSec, 0.0);
    }
  }

  public LEDLights() {
    var cfg = new CANdleConfiguration();
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    m_candle.getConfigurator().apply(cfg);

    for (int i = 0; i < 8; ++i) {
      m_candle.setControl(new EmptyAnimation(i));
    }

    applyMode(LedMode.Rainbow);
  }

  private void flashingColor(RGBWColor color, int frameRateHz) {
    m_candle.setControl(
        new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx)
            .withSlot(0)
            .withColor(color)
            .withFrameRate(frameRateHz));
  }

  private void solidColor(RGBWColor color) {
    m_candle.setControl(new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(color));
  }

  private void rainbow() {
    m_candle.setControl(new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
  }

  private void applyMode(LedMode mode) {
    if (mode == m_lastMode) {
      return;
    }

    // Clear animation slot first so old strobe/rainbow state cannot persist.
    m_candle.setControl(new EmptyAnimation(0));

    switch (mode) {
      case Rainbow -> rainbow();
      case SolidRed -> solidColor(kRed);
      case SolidBlue -> solidColor(kBlue);
      case SolidPink -> solidColor(kPink);
      case SolidViolet -> solidColor(kViolet);
      case FlashRed -> flashingColor(kRed, 5);
      case FlashGreen -> flashingColor(kGreen, 5);
      case FlashWhite -> flashingColor(kWhite, 5);
    }

    m_lastMode = mode;
  }

  private TeleopWindow getTeleopWindow(double matchTimeSec) {
    if (matchTimeSec > kTransitionEnd) return TeleopWindow.Transition;
    if (matchTimeSec > kShift1End) return TeleopWindow.Shift1;
    if (matchTimeSec > kShift2End) return TeleopWindow.Shift2;
    if (matchTimeSec > kShift3End) return TeleopWindow.Shift3;
    if (matchTimeSec > kShift4End) return TeleopWindow.Shift4;
    if (matchTimeSec > kShift5End) return TeleopWindow.Shift5;
    if (matchTimeSec > 0.0) return TeleopWindow.Endgame;
    return TeleopWindow.Complete;
  }

  private Optional<Boolean> didWeWinAuto() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    String gameData = DriverStation.getGameSpecificMessage();

    if (alliance.isEmpty() || gameData.isEmpty()) {
      SmartDashboard.putString("LED/AllianceChar", "?");
      SmartDashboard.putString("LED/GameDataChar", "?");
      SmartDashboard.putBoolean("LED/GameDataMatchesAlliance", false);
      SmartDashboard.putBoolean("LED/WonAutoFromGameData", false);
      return Optional.empty();
    }

    char allianceChar = alliance.get() == Alliance.Red ? 'R' : 'B';
    char gameChar = Character.toUpperCase(gameData.charAt(0));
    if (gameChar != 'R' && gameChar != 'B') {
      SmartDashboard.putString("LED/AllianceChar", String.valueOf(allianceChar));
      SmartDashboard.putString("LED/GameDataChar", String.valueOf(gameChar));
      SmartDashboard.putBoolean("LED/GameDataMatchesAlliance", false);
      SmartDashboard.putBoolean("LED/WonAutoFromGameData", false);
      return Optional.empty();
    }

    boolean matchesAlliance = gameChar == allianceChar;
    boolean wonAuto =
        kWinWhenGameDataNotAlliance ? (gameChar != allianceChar) : (gameChar == allianceChar);

    SmartDashboard.putString("LED/AllianceChar", String.valueOf(allianceChar));
    SmartDashboard.putString("LED/GameDataChar", String.valueOf(gameChar));
    SmartDashboard.putBoolean("LED/GameDataMatchesAlliance", matchesAlliance);
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
      case Transition -> Optional.of(false);
      case Shift1 -> Optional.of(shift1Active);
      case Shift2 -> Optional.of(!shift1Active);
      case Shift3 -> Optional.of(shift1Active);
      case Shift4 -> Optional.of(!shift1Active);
      case Shift5 -> Optional.of(shift1Active);
      case Endgame, Complete -> Optional.of(false);
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
      double activeRemaining =
          switch (window) {
            case Shift1 -> matchTimeSec - kShift1End;
            case Shift2 -> matchTimeSec - kShift2End;
            case Shift3 -> matchTimeSec - kShift3End;
            case Shift4 -> matchTimeSec - kShift4End;
            case Shift5 -> matchTimeSec - kShift5End;
            default -> 0.0;
          };
      return new HubTiming(true, activeRemaining, 0.0);
    }

    double nextActiveSec =
        switch (window) {
          case Transition -> matchTimeSec - kTransitionEnd;
          case Shift1 -> matchTimeSec - kShift1End;
          case Shift2 -> matchTimeSec - kShift2End;
          case Shift3 -> matchTimeSec - kShift3End;
          case Shift4 -> matchTimeSec - kShift4End;
          case Shift5 -> matchTimeSec - kShift5End;
          default -> 0.0;
        };

    return new HubTiming(false, 0.0, nextActiveSec);
  }

  private LedMode getAutoMode() {
    return LedMode.Rainbow;
  }

  private LedMode getTeleopMode(double matchTimeSec) {
    HubTiming timing = getHubTiming(matchTimeSec);
    SmartDashboard.putBoolean("LED/HubActive", timing.hubActive);
    SmartDashboard.putNumber("LED/ActiveRemainingSec", timing.activeRemainingSec);
    SmartDashboard.putNumber("LED/NextActiveInSec", timing.nextActiveSec);

    if (matchTimeSec <= 0.0) {
      return LedMode.Rainbow;
    }

    if (matchTimeSec <= 10.0) {
      return LedMode.SolidViolet;
    }

    if (matchTimeSec <= 35.0 && matchTimeSec > 30.0) {
      return LedMode.SolidPink;
    }

    // Transition window: after 3s into teleop, flash auto result (green/white).
    if (getTeleopWindow(matchTimeSec) == TeleopWindow.Transition) {
      if (matchTimeSec > kTeleopGameDataAvailableMatchTime) {
        return LedMode.Rainbow;
      }
      Optional<Boolean> wonAutoOpt = didWeWinAuto();
      if (wonAutoOpt.isEmpty()) {
        return LedMode.Rainbow;
      }
      return wonAutoOpt.get() ? LedMode.FlashGreen : LedMode.FlashWhite;
    }

    if (timing.hubActive) {
      if (timing.activeRemainingSec <= 5.0) {
        return LedMode.SolidBlue;
      }
      return LedMode.SolidRed;
    }

    if (timing.nextActiveSec <= 5.0 && timing.nextActiveSec > 0.0) {
      return LedMode.FlashRed;
    }

    return LedMode.Rainbow;
  }

  @Override
  public void periodic() {
    LedMode targetMode;

    if (DriverStation.isDisabled()) {
      targetMode = LedMode.Rainbow;
      SmartDashboard.putBoolean("LED/HubActive", false);
      SmartDashboard.putNumber("LED/ActiveRemainingSec", 0.0);
      SmartDashboard.putNumber("LED/NextActiveInSec", 0.0);
    } else if (DriverStation.isAutonomousEnabled()) {
      targetMode = getAutoMode();
      SmartDashboard.putBoolean("LED/HubActive", false);
      SmartDashboard.putNumber("LED/ActiveRemainingSec", 0.0);
      SmartDashboard.putNumber("LED/NextActiveInSec", 0.0);
    } else if (DriverStation.isTeleopEnabled()) {
      targetMode = getTeleopMode(DriverStation.getMatchTime());
    } else {
      targetMode = LedMode.Rainbow;
      SmartDashboard.putBoolean("LED/HubActive", false);
      SmartDashboard.putNumber("LED/ActiveRemainingSec", 0.0);
      SmartDashboard.putNumber("LED/NextActiveInSec", 0.0);
    }

    applyMode(targetMode);
  }
}
