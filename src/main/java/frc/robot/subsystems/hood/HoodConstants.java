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
  public static final double hubHeight = 72;
  public static final double gravity = 386.09;

  public static final double ballExitAngle = Math.toRadians(35);

  public static final double exitVelo = 334.646;

  public static final double gearRatio = 45;
  public static final double offset = 0;
}
