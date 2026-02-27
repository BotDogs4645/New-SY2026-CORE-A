package frc.robot.subsystems.leds;

import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class Leds extends FullSubsystem {

  private LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  public Leds(LedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);
  }

  @Override
  public void periodicAfterScheduler() {}
}
