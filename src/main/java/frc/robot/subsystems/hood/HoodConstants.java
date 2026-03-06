package frc.robot.subsystems.hood;

import com.ctre.phoenix6.CANBus;

public final class HoodConstants {

  public static final int motorCanId = 18;
  public static final int thorughboreEncoderId = 19;
  public static final CANBus motorCanBus = new CANBus("CANivore");

  public static final double motionMagicCruiseVelocity = 10;
  public static final double motionMagicAcceleration = 4;
  public static final double motionMagicJerk = 0.0;

  // Inches
  public static final double hubHeight = 1.82;
  public static final double gravity = 9.8;

  public static final double ballExitAngle = 0.065;

  public static final double exitVelo = 8.5;

  public static final double gearRatio = 45;
  public static final double offset = 0;
}
