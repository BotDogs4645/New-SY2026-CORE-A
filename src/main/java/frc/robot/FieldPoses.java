package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldPoses {
  public static final Pose2d HUB = new Pose2d(4.6256, 4.0345, Rotation2d.fromRotations(0));
  public static final Pose2d OUTPOST_INTAKE = new Pose2d(0.564, 0.668, Rotation2d.fromRotations(0));
  public static final Pose2d TOWER_CLIMB_RIGHT =
      new Pose2d(1.699, 3.239, Rotation2d.fromDegrees(-53.13));
  public static final Pose2d TOWER_CLIMB_LEFT =
      new Pose2d(1.699, 4.100, Rotation2d.fromDegrees(-53.13));
}
