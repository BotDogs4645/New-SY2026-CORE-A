package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public final class ElasticNotifications {

  public final class AutoShot {
    public static Elastic.Notification turretCannotReach =
        new Elastic.Notification(
            Elastic.NotificationLevel.ERROR,
            "Turret Cannot Reach",
            "Turret is unable to reach an optimal scoring position - rotate the robot!");

    public static final Alert turretCannotReachAlert =
        new Alert("Shooter Alerts", "Turret cannot reach scoring position", AlertType.kError);

    public static Elastic.Notification outOfBounds =
        new Elastic.Notification(
            Elastic.NotificationLevel.ERROR,
            "Out of Bounds",
            "You cannot shoot from this location - move back to your alliance side");

    public static final Alert outOfBoundsAlert =
        new Alert("Shooter Alerts", "Cannot shoot - return to alliance side", AlertType.kError);
  }
}
