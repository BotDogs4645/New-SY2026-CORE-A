package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class PathPlanOnTheFly {

  public static class AllianceSide {
    public double insideX;
    public double outsideX;

    public AllianceSide(double insideX, double outsideX) {
      this.insideX = insideX;
      this.outsideX = outsideX;
    }

    public double getInsideX() {
      return insideX;
    }

    public double getOutsideX() {
      return outsideX;
    }
  }

  public static final AllianceSide blueSide = new AllianceSide(3.086, 5.940);
  public static final AllianceSide redSide =
      new AllianceSide(
          FieldConstants.fieldLength - blueSide.getInsideX(),
          FieldConstants.fieldLength - blueSide.getOutsideX());

  public static final double lowerY = 0.630;
  public static final double upperY = FieldConstants.fieldWidth - lowerY;
  public static final List<AllianceSide> allianceList = List.of(blueSide, redSide);

  private static PathConstraints constraints =
      new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this
  // path.

  public static PathPlannerPath getNavigateTrenchPath(Pose2d currentPose) {
    double optimalRotationDegrees = getOptimalChassisRotationDegreees(currentPose);
    double currentX = currentPose.getX();

    double closestY = currentPose.getY() < (FieldConstants.fieldWidth / 2) ? lowerY : upperY;
    double closestX = blueSide.getInsideX();
    double correspondingX = blueSide.getOutsideX();

    for (AllianceSide side : allianceList) {
      if (Math.abs(side.getInsideX() - currentX) < Math.abs(closestX - currentX)) {
        closestX = side.getInsideX();
        correspondingX = side.getOutsideX();
      }
      if (Math.abs(side.getOutsideX() - currentX) < Math.abs(closestX - currentX)) {
        closestX = side.getOutsideX();
        correspondingX = side.getInsideX();
      }
    }

    Logger.recordOutput("PathPlan/optimalRotation", Rotation2d.fromDegrees(optimalRotationDegrees));
    List<Pose2d> poseList =
        List.of(
            // new Pose2d(
            //     currentPose.getX(),
            //     currentPose.getY(),
            //     Rotation2d.fromDegrees(optimalRotationDegrees)),
            new Pose2d(closestX, closestY, Rotation2d.fromDegrees(optimalRotationDegrees)),
            new Pose2d(correspondingX, closestY, Rotation2d.fromDegrees(optimalRotationDegrees)));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poseList);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    optimalRotationDegrees)) // Goal end state. You can set a holonomic rotation
            // here. If using a differential drivetrain, the
            // rotation will have no effect.
            );
    path.preventFlipping = true;
    return path;
  }

  public static double getOptimalChassisRotationDegreees(Pose2d currentPose) {
    double degreesTo180 = Math.abs(180 - currentPose.getRotation().getDegrees());
    double degreesToNegative180 = Math.abs(-180 - currentPose.getRotation().getDegrees());
    if (degreesTo180 < 90) {
      return 180;
    } else {
      if (degreesToNegative180 < 90) {
        return -180;
      } else {
        return 0;
      }
    }
  }

  public static Command NavigateTrench(Drive drive) {
    return Commands.defer(
        () ->
            AutoBuilder.pathfindThenFollowPath(getNavigateTrenchPath(drive.getPose()), constraints),
        Set.of(drive));
  }
}
