package frc.robot.subsystems.leds;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

  private CANdle candle = new CANdle(LedConstants.candleCanId);
  private RainbowAnimation rainbowAnimation = new RainbowAnimation(0, 300);
  private StrobeAnimation strobeAnimation =
      new StrobeAnimation(0, 300).withColor(new RGBWColor(255, 255, 255)).withFrameRate(3);

  public Leds() {
    var config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = LedConstants.brightnessScalar;
    tryUntilOk(5, () -> candle.getConfigurator().apply(config, 0.25));

    candle.setControl(rainbowAnimation);
  }

  public Command BlinkLEDs() {
    return Commands.sequence(
        runOnce(() -> candle.setControl(strobeAnimation)),
        Commands.waitSeconds(1),
        runOnce(() -> candle.setControl(rainbowAnimation)));
  }
}
