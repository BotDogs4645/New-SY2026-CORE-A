package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** IO implementation for QuestNav */
public class VisionIOQuestNav implements VisionIO {
  private final QuestNav questNav;
  private final Transform3d robotToQuest;
  private final Alert lowBatteryAlert =
      new Alert("QuestNav battery below 30%! Charge it!!!", AlertType.kWarning);
  private final String name;
  private Transform3d calibrationTransform = new Transform3d();

  public VisionIOQuestNav(String name) {
    this.name = name;
    this.questNav = new QuestNav();
    this.robotToQuest = VisionConstants.robotToQuestTransform;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    questNav.commandPeriodic();
    inputs.name = name;

    inputs.connected = questNav.isConnected();
    questNav
        .getBatteryPercent()
        .ifPresent(battery -> lowBatteryAlert.set(battery < 30.0 && questNav.isConnected()));
    if (questNav.getBatteryPercent().isPresent()) {
      inputs.batteryLevel = questNav.getBatteryPercent().getAsInt();
    }

    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    List<PoseObservation> observations = new LinkedList<>();

    for (PoseFrame frame : frames) {
      if (!frame.isTracking()) continue;

      Pose3d questPose = frame.questPose3d();
      Logger.recordOutput("Vision/QuestNav/RawQuestPose", questPose);
      Pose3d robotPose =
          questPose.transformBy(robotToQuest.inverse()).transformBy(calibrationTransform);

      observations.add(
          new PoseObservation(
              frame.dataTimestamp(),
              robotPose,
              0.0, // no ambiguity concept
              2, // placeholder tagCount
              0.0, // no distance metric
              PoseObservationType.QUESTNAV));
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.tagIds = new int[0];
  }

  /** reset QuestNav pose */
  public void setPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(robotToQuest);
    questNav.setPose(questPose);
  }

  public void setInitialPose(Pose2d robotPose) {
    Pose3d desiredRobotPose =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
    setPose(desiredRobotPose);
    // PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    // if (frames.length > 0) {
    //   PoseFrame latestFrame = frames[frames.length - 1];
    //   if (latestFrame.isTracking()) {
    //     Pose3d questPose = latestFrame.questPose3d();
    //     Pose3d currentRobotPose = questPose.transformBy(robotToQuest.inverse());
    //     Pose3d desiredRobotPose =
    //         new Pose3d(
    //             robotPose.getX(),
    //             robotPose.getY(),
    //             0,
    //             new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    //     calibrationTransform = new Transform3d(currentRobotPose, desiredRobotPose);
    //     Logger.recordOutput("Vision/QuestNav/CalibrationTransform", calibrationTransform);
    //   }
    // }
  }

  public void resetToZero() {
    Pose3d pose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    setPose(pose);
  }
}
