// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCurve extends CommandBase {

  DriveSubsystem drive;
  double targetDistance;
  double speed;
  double otherSpeed;
  double targetAngle;
  double radiusOfTurn;

  double startingLeftDistance;
  double startingRightDistance;

  /** Creates a new DriveCurve. */
  public DriveCurve(DriveSubsystem d, double a, double s, double o, double r) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = d;
    targetAngle = a;
    speed = s;
    otherSpeed = o;
    radiusOfTurn = r;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetOdometry(new Pose2d());

    startingLeftDistance = drive.getLeftDistance();
    startingRightDistance = drive.getRightDistance();

    targetDistance = (radiusOfTurn + Constants.DriveConstants.circumferenceWithWB / 2) * (targetAngle / 360.0);

    otherSpeed = speed * ((radiusOfTurn - (Constants.DriveConstants.circumferenceWithWB / 2)
        / (radiusOfTurn + (Constants.DriveConstants.circumferenceWithWB / 2))));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setWheelSpeeds(new DifferentialDrive.WheelSpeeds(speed, otherSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setWheelSpeeds(new DifferentialDrive.WheelSpeeds(0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTraveled = Math.abs(drive.getLeftDistance() - startingLeftDistance);
        if (Math.abs(distanceTraveled) >= Math.abs(targetDistance)) {
            return true;
        }
        return false;
  }
}
