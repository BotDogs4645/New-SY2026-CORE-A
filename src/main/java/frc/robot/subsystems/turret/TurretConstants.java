package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TurretConstants {

  public static final int motorCanId = 6;
  public static final String motorCanBus = "canivore";

  public static final Pose2d HUB_POSE2D = new Pose2d(0, 0, Rotation2d.fromRotations(0));
}
