package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
  public static final int shooterMotorCanId = 16;
  public static final CANBus shooterMotorCanBus = new CANBus("CANivore");
  public static final boolean shooterMotorInverted = true;

  public static final int kickerMotorCanId = 15;
  public static final CANBus kickerMotorCanBus = new CANBus("CANivore");
  public static final boolean kickerMotorInverted = true;

  // for active duty cycle out, these values are the proportion of the supply voltage to apply
  public static final double shooterActiveVoltageProportion = 0.1;
  public static final double kickerActiveVoltageProportion = 0.1;

  // not yet implemented (amps)
  public static final double shooterSupplyCurrentLimit = 1000;
  public static final double kickerSupplyCurrentLimit = 1000;
}
