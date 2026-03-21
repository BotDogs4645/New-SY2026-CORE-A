// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double fieldWidth = 16.5409;
  public static final double fieldLength = 8.0693;

  // tuning mode - enables dashboard-tunable values
  public static final boolean tuningMode = true;

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static class PathPlannerConstants {
    public static RobotConfig config;

    static {
      try {
        PathPlannerConstants.config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    public static final PIDConstants translationPID = new PIDConstants(5, 0, 0);
    public static final PIDConstants rotationPID = new PIDConstants(5, 0, 0);

    // Speed and acceleration limits for on the fly path generation
    public static final PathConstraints pathConstraints =
        new PathConstraints(
            MetersPerSecond.of(1),
            MetersPerSecondPerSecond.of(10),
            RotationsPerSecond.of(0.75),
            RotationsPerSecondPerSecond.of(5));
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
