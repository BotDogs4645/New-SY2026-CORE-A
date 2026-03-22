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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  private CANdle candle = new CANdle(LedConstants.candleCanId, LedConstants.candleCanBus);
  private RainbowAnimation rainbowAnimation = new RainbowAnimation(0, 300);
  private StrobeAnimation strobeAnimation =
      new StrobeAnimation(0, 300).withColor(new RGBWColor(255, 255, 255)).withFrameRate(3);
  private ColorFlowAnimation blueColorFlow =
      new ColorFlowAnimation(0, 300).withColor(new RGBWColor(0, 0, 255)).withFrameRate(400);
  private ColorFlowAnimation redColorFlow =
      new ColorFlowAnimation(0, 300).withColor(new RGBWColor(255, 0, 0)).withFrameRate(400);
  private ColorFlowAnimation greenColorFlow =
      new ColorFlowAnimation(0, 300).withColor(new RGBWColor(0, 255, 0)).withFrameRate(400);
  private ColorFlowAnimation whiteColorFlow =
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

  public Command RedColorFlow() {
    return runEnd(
        () -> {
          Logger.recordOutput("LED/LEDColor", "COLOR_FLOW_RED");
          candle.setControl(redColorFlow);
        },
        () -> {
          Logger.recordOutput("LED/LEDColor", "RAINBOW");
          candle.setControl(rainbowAnimation);
        });
  }

  public Command BlueColorFlow() {
    return runEnd(
        () -> {
          Logger.recordOutput("LED/LEDColor", "COLOR_FLOW_BLUE");
          candle.setControl(blueColorFlow);
        },
        () -> {
          Logger.recordOutput("LED/LEDColor", "RAINBOW");
          candle.setControl(rainbowAnimation);
        });
  }

  public Command GreenColorFlow() {
    return runEnd(
        () -> {
          Logger.recordOutput("LED/LEDColor", "COLOR_FLOW_GREEN");
          candle.setControl(greenColorFlow);
        },
        () -> {
          Logger.recordOutput("LED/LEDColor", "RAINBOW");
          candle.setControl(rainbowAnimation);
        });
  }

  public Command WhiteColorFlow() {
    return runEnd(
        () -> {
          Logger.recordOutput("LED/LEDColor", "COLOR_FLOW_WHITE");
          candle.setControl(whiteColorFlow);
        },
        () -> {
          Logger.recordOutput("LED/LEDColor", "RAINBOW");
          candle.setControl(rainbowAnimation);
        });
  }

  public Command TrueFalseColorFlow(BooleanSupplier isTrue) {
    return runEnd(
        () -> {
          if (isTrue.getAsBoolean()) {
            Logger.recordOutput("LED/LEDColor", "COLOR_FLOW_GREEN");
            candle.setControl(greenColorFlow);
          } else {
            Logger.recordOutput("LED/LEDColor", "COLOR_FLOW_RED");
            candle.setControl(redColorFlow);
          }
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
