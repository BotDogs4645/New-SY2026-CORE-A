package frc.robot.subsystems.hood;

import com.ctre.phoenix6.CANBus;

public final class HoodConstants {

  public static final int motorCanId = 18;
  public static final int thorughboreEncoderId = 19;
  public static final CANBus motorCanBus = new CANBus("CANivore");
  public static final CANBus encoderCanBus = new CANBus("CANivore");

  // public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 7.38);
  // public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", 1.6);
  public static final double kP = 11;
  public static final double kI = 4;
  public static final double kD = 0.1;
  public static final double kG = 0.33;

  public static final double motionMagicCruiseVelocity = 14;
  public static final double motionMagicAcceleration = 9;
  public static final double motionMagicJerk = 0.0;

  public static final double fowardSoftLimit = 0.382;
  public static final double reverseSoftLimit = 0;

  // Inches
  public static final double hubHeight = 1.82;
  public static final double gravity = 9.8;

  public static final double ballExitAngle = 0.065;

  public static final double exitVelo = 8.5;

  public static final double gearRatio = 45;
  public static final double offset = 0;
}
