package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public final class IntakeConstants {
  public static final int rollerMotorCanId = 17;
  public static final CANBus rollerMotorCanBus = new CANBus("CANivore");

  public static final int armMotorCanID = 19;
  public static final CANBus armMotorCanBus = new CANBus("CANivore");

  // for active duty cycle out, these values are the proportion of the supply voltage to apply
  public static final double rollerActiveVoltageProportion = 0.1;
  public static final double armActiveVoltageProportion = 0.1;

  public static final double kRollerInRadPerSec = 30;
  public static final double kArmUpRad = 0;
  public static final double kArmDownRad = 98.155;
  public static final double kArmDownRadHalf = 28.815;
}
