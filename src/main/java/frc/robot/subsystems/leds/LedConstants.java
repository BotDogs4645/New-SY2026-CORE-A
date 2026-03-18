package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;

public final class LedConstants {
  public static final int candleCanId = 25;
  public static final CANBus candleCanBus = new CANBus("CANivore");
  public static final double brightnessScalar = 0.3;
}
