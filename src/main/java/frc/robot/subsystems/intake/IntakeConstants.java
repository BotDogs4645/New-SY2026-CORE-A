package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public final class IntakeConstants {
  public static final int rollerMotorCanId = 17;
  public static final CANBus rollerMotorCanBus = new CANBus("CANivore");

  public static final boolean rollersInverted = false;

  public static final int armMotorCanID = 19;
  public static final CANBus armMotorCanBus = new CANBus("CANivore");

  public static final double armKP = 5.0;
  public static final double armKG = 0.1;
  public static final double armMotionMagicCruiseVelocity = 13.0;
  public static final double armMotionMagicAcceleration = 5.0;
  public static final double armPeakForwardVoltage = 3.0;
  public static final double armPeakReverseVoltage = -3.0;

  // for active duty cycle out, these values are the proportion of the supply voltage to apply
  public static final double intakingRollerOutput = 0.32;
  public static final double armDownRollerOutput = 0.35;

  public static final double kRollerInRadPerSec = 30;
  public static final double kArmUpRad = 0;
  public static final double kArmDownRad = 98.155;
  public static final double kArmHalfDownRad = 28.815;

  public enum ArmMechanismPosition {

    // in radians
    ARM_UP(0.0),
    ARM_HALF_DOWN(27.37),
    DISLODGE_BALLS(85),
    ARM_DOWN(90);

    public final Double motorPositionRad;

    ArmMechanismPosition(double motorPositionRad) {
      this.motorPositionRad = motorPositionRad;
    }
  }
}
