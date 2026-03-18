package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
  public static final int motorCanId = 16;
  public static final CANBus motorCanBus = new CANBus("CANivore");
  public static final boolean motorInverted = true;

  public static final double defaultSpeedRadPerSec = 300;

  // not yet implemented (amps)
  public static final double supplyCurrentLimit = 1000;

  public static final double shooterWheelRadiusMeters = 0.03;
}
