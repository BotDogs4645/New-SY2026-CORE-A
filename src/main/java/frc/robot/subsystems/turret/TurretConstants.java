package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;

public final class TurretConstants {

  public static final int motorCanId = 23;
  public static final CANBus motorCanBus = new CANBus("CANivore");

  public static final double kP = 5.5;
  public static final double kI = 0.3;
  public static final double kD = 0.4;
  public static final double kS = 0.05;

  public static final double motionMagicCruiseVelocity = 20;
  public static final double motionMagicAcceleration = 15;
  public static final double motionMagicJerk = 0.0;

  // in real turret radians:
  public static final double encoderStartingPosition = -Math.PI;
  public static final double physicalStartingPosition = 0;
  public static final double hardReverseLimit = -4.6;
  public static final double hardForwardLimit = 2.4;

  public static final double offsetPeriodicDegrees = 0.6;
  public static final double staticOverrideOffsetPeriodicDegrees = 4;

  public static final double gearRatio = 3.3333;

  public static final double turretXOffsetMeters = 0.22225;
  public static final double turretYOffsetMeters = -0.14;
  public static final double turretZOffsetMeters = 0.4445;
}
