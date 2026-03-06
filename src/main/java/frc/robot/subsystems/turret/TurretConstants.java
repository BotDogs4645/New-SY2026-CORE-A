package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;

public final class TurretConstants {

  public static final int motorCanId = 23;
  public static final CANBus motorCanBus = new CANBus("CANivore");

  public static final double kP = 5;
  public static final double kI = 0.2;
  public static final double kD = 0.4;
  public static final double kS = 0.04;

  public static final double motionMagicCruiseVelocity = 20;
  public static final double motionMagicAcceleration = 15;
  public static final double motionMagicJerk = 0.0;

  // in radians:
  public static final double encoderStartingPosition = 0.0;
  // in radians:
  public static final double physicalStartingPosition = 0;
  public static final double reverseLimit = -1.39;
  public static final double forwardLimit = 1.39;

  public static final double gearRatio = 3.2959;
}
