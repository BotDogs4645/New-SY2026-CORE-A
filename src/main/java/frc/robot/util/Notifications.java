package frc.robot.util;

import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class Notifications {
  public class AutoShot {
    public static final Notification turretCannotReachNotification =
        new Notification(
            NotificationLevel.ERROR, "Shooter Alerts", "Turret unable to reach scoring position");
    public static final Notification shooterRangeNotification =
        new Notification(
            NotificationLevel.ERROR,
            "Shooter Alerts",
            "Cannot shoot - target out of shooter range");
  }
}
