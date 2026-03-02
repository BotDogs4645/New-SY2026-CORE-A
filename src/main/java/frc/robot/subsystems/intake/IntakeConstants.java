package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public final class IntakeConstants {
  public static final int rollerMotorCanId = 17;
  public static final CANBus rollerMotorCanBus = new CANBus("canivore");

  public static final int armMotorCanID = 19;
  public static final CANBus armMotorCanBus = new CANBus("canivore");

  // for active duty cycle out, these values are the proportion of the supply voltage to apply
  public static final double rollerActiveVoltageProportion = 0.1;
  public static final double armActiveVoltageProportion = 0.1;

  public static final double kRollerInRadPerSec = 30;
  public static final double kArmUpRad = 0.82;
  public static final double kArmDownRad = 0.124512;
  public static final double kArmDownRadHalf = 0.75;
}
