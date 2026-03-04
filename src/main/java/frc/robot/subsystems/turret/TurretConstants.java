package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;

public final class TurretConstants {

  public static final int motorCanId = 23;
  public static final CANBus motorCanBus = new CANBus("CANivore");

  public static final double motionMagicCruiseVelocity = 1;
  public static final double motionMagicAcceleration = 1;
  public static final double motionMagicJerk = 0.0;

  // in radians:
  public static final double encoderStartingPosition = 0.0;
  // in radians:
  public static final double physicalStartingPosition = 0;
  public static final double reverseLimit = -1.19;
  public static final double forwardLimit = 1.09;

  public static final double gearRatio = 16;
}
