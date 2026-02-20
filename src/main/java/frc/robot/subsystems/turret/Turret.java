// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private TurretIO io;
  private NeutralOut neutralControlRequest = new NeutralOut();
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command followHub(Supplier<Pose2d> currentPoseSupplier) {
    return followTargetPosition(currentPoseSupplier, TurretConstants.HUB_POSE2D);
  }

  public Command followTargetPosition(Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose) {
    MotionMagicDutyCycle controlRequest =
        new MotionMagicDutyCycle(getOptimalPosition(currentPoseSupplier.get(), targetPose));
    return runEnd(
        () -> {
          io.setMotorControl(controlRequest);
          controlRequest.withPosition(getOptimalPosition(currentPoseSupplier.get(), targetPose));
        },
        () -> io.setMotorControl(neutralControlRequest));
  }

  // returns the position to spin the motor to in order to follow a target
  public double getOptimalPosition(Pose2d curPose, Pose2d targetPose) {
    return 0;
  }
}
