package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public final class Alerts {

  public final class AutoShot {

    public static final Alert turretCannotReachAlert =
        new Alert("Shooter Alerts", "Turret unable to reach scoring position", AlertType.kError);
    public static final Alert outOfBoundsAlert =
        new Alert("Shooter Alerts", "Cannot shoot - return to alliance side", AlertType.kError);
    public static final Alert shooterRangeAlert =
        new Alert("Shooter Alerts", "Cannot shoot - target out of range", AlertType.kError);
    public static final Alert staticAimAlert =
        new Alert("Shooter Alerts", "Shooter Driver Override", AlertType.kWarning);

    public static void resetAutoShotAlerts() {
      turretCannotReachAlert.set(false);
      outOfBoundsAlert.set(false);
      shooterRangeAlert.set(false);
      staticAimAlert.set(false);
    }
  }
}
