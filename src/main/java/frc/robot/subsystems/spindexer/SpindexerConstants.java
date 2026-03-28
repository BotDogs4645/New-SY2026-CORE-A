package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;

public final class SpindexerConstants {

  public static final int motorCanId = 22;
  public static final CANBus motorCanBus = new CANBus("CANivore");

  public static final double motionMagicCruiseVelocity = 0.0;
  public static final double motionMagicAcceleration = 0.0;
  public static final double motionMagicJerk = 0.0;

  public static final boolean isInverted = true;
  public static final double activeSpeed = 0.3;

  public static final double stalledCurrentThreshold = 20;
  public static final double stalledSpeedThresholdRadPerSec = 6;
}
