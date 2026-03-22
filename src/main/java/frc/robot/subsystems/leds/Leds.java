package frc.robot.subsystems.leds;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  private CANdle candle = new CANdle(LedConstants.candleCanId, LedConstants.candleCanBus);
  private RainbowAnimation rainbowAnimation = new RainbowAnimation(0, 300);
  private StrobeAnimation strobeAnimation =
      new StrobeAnimation(0, 300).withColor(new RGBWColor(255, 255, 255)).withFrameRate(3);
  private ColorFlowAnimation bluePreMatch =
      new ColorFlowAnimation(0, 300).withColor(new RGBWColor(19, 48, 255)).withFrameRate(400);
  private ColorFlowAnimation redPreMatch =
      new ColorFlowAnimation(0, 300).withColor(new RGBWColor(255, 48, 19)).withFrameRate(400);
  private ColorFlowAnimation colorFlow =
      new ColorFlowAnimation(0, 300).withColor(new RGBWColor(255, 255, 255)).withFrameRate(400);

  public Leds() {
    var config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = LedConstants.brightnessScalar;
    tryUntilOk(5, () -> candle.getConfigurator().apply(config, 0.25));

    candle.setControl(rainbowAnimation.withSlot(0));
    Logger.recordOutput("LED/LEDColor", "RAINBOW");
  }

  public Command BlinkLEDs() {
    return Commands.sequence(
        runOnce(() -> candle.setControl(strobeAnimation)),
        Commands.waitSeconds(1),
        runOnce(() -> candle.setControl(rainbowAnimation)));
  }

  public Command RedLEDs() {
    return runEnd(
        () -> {
          Logger.recordOutput("LED/LEDColor", "RED");
          candle.setControl(redPreMatch);
        },
        () -> {
          Logger.recordOutput("LED/LEDColor", "RAINBOW");
          candle.setControl(rainbowAnimation);
        });
  }

  public Command BlueLEDs() {
    return runEnd(
        () -> {
          Logger.recordOutput("LED/LEDColor", "BLUE");
          candle.setControl(bluePreMatch);
        },
        () -> {
          Logger.recordOutput("LED/LEDColor", "RAINBOW");
          candle.setControl(rainbowAnimation);
        });
  }

  public Command ColorFlow() {
    return runEnd(
        () -> {
          Logger.recordOutput("LED/LEDColor", "COLOR_FLOW_WHITE");
          candle.setControl(colorFlow);
        },
        () -> {
          Logger.recordOutput("LED/LEDColor", "RAINBOW");
          candle.setControl(rainbowAnimation);
        });
  }

  // public Command MatchLedsToAlliance(Alliance alliance) {
  //   ControlRequest activeControl;
  //   switch(alliance) {
  //     case Blue:
  //       activeControl = bluePreMatch;
  //   }
  // }
}
