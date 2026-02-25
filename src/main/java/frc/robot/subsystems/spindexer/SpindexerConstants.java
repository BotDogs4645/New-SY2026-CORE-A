package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;

public final class SpindexerConstants {

  public static final int motorCanId = 6;
  public static final CANBus motorCanBus = new CANBus("canivore");

  public static final double motionMagicCruiseVelocity = 0.0;
  public static final double motionMagicAcceleration = 0.0;
  public static final double motionMagicJerk = 0.0;
}
