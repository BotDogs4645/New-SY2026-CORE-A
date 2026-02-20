package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  public static final int shooterMotorCanId = 8;
  public static final String shooterMotorCanBus = "canivore";

  public static final int kickerMotorCanId = 9;
  public static final String kickerMotorCanBus = "canivore";

  // for active duty cycle out, these values are the proportion of the supply voltage to apply
  public static final double shooterActiveVoltageProportion = 0.1;
  public static final double kickerActiveVoltageProportion = 0.1;
}
