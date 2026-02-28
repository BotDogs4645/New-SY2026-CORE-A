package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public final class IntakeConstants {
  public static final int rollerMotorCanId = 41;
  public static final CANBus rollerMotorCanBus = new CANBus("canivore");

  public static final int armMotorCanID = 67;
  public static final CANBus armMotorCanBus = new CANBus("canivore");

  // for active duty cycle out, these values are the proportion of the supply voltage to apply
  public static final double rollerActiveVoltageProportion = 0.1;
  public static final double armActiveVoltageProportion = 0.1;

  // not yet implemented (amps)
  public static final double rollerSupplyCurrentLimit = 1000;
  public static final double armSupplyCurrentLimit = 1000;

  public static final double kRollerInRadPerSec = 67.67;
}
