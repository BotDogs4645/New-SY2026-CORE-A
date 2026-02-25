package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;

public final class TurretConstants {

  public static final int motorCanId = 6;
  public static final CANBus motorCanBus = new CANBus("canivore");

  public static final double motionMagicCruiseVelocity = 0.0;
  public static final double motionMagicAcceleration = 0.0;
  public static final double motionMagicJerk = 0.0;

  // in radians:
  public static final double encoderStartingPosition = 0.0;
  // in radians:
  public static final double physicalStartingPosition = -(Math.PI / 2);
  public static final double reverseLimit = 0.0;
  public static final double forwardLimit = 0.5;

  public static final double gearRatio = 16;
}
