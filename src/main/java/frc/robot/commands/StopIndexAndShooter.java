// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class StopIndexAndShooter extends CommandBase {

  private Index m_index;
  private Shooter m_shooter;

  /** Creates a new StopIndex. */
  public StopIndexAndShooter(Index i, Shooter s) {
    m_index = i;
    m_shooter = s;
    addRequirements(m_shooter, m_index);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.stopShooting();
    m_index.stopFeeder();
    m_index.stopHopper();
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
