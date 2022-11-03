// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class StartIndexAndShooter extends CommandBase {

  private Shooter m_shooter;
  private Index m_index;

  /** Creates a new StartShooting. */
  public StartIndexAndShooter(Shooter s, Index i) {
    m_shooter = s;
    m_index = i;
    addRequirements(m_shooter, m_index);
    // Use addRequirements() here to dclare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.startShooting();

    if(m_shooter.getFlywheelSpeed() >= (0.95 * Constants.ShooterConstants.Flywheel.FlywheelVelocity)) {
      m_index.startFeeder();
      m_index.startHopper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
