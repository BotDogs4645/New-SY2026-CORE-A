package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.CANBus;

public final class KickerConstants {

  public static final int motorCanId = 15;
  public static final CANBus motorCanBus = new CANBus("CANivore");
  public static final boolean motorInverted = true;

  public static final double kV = 0.1205;
  public static final double kP = 0.05;

  public static final double defaultSpeedRadPerSec = 320;

  public static final double stalledSpeedThresholdRadPerSec = 2;
  public static final double stalledCurrentThreshold = 10;

  // not yet implemented (amps)
  public static final double supplyCurrentLimit = 1000;
}
