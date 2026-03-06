package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
  public static final int shooterMotorCanId = 16;
  public static final CANBus shooterMotorCanBus = new CANBus("CANivore");
  public static final boolean shooterMotorInverted = true;

  public static final int kickerMotorCanId = 15;
  public static final CANBus kickerMotorCanBus = new CANBus("CANivore");
  public static final boolean kickerMotorInverted = true;

  public static final double shooterDefaultSpeedRadPerSec = 600;
  public static final double kickerDefaultSpeedRadPerSec = 600;

  // not yet implemented (amps)
  public static final double shooterSupplyCurrentLimit = 1000;
  public static final double kickerSupplyCurrentLimit = 1000;
}
