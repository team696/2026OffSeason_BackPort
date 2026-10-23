// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterTestCommand extends Command {
  //CONFIG
  static final Current FEEDROLLER_TRIP = Amps.of(3);
  static final Current SHOOTERS_TRIP = Amps.of(3);



  /** Creates a new ShooterTestCommand. */
  private Timer m_timer = new Timer();
  private StatusSignal<Current> m_current_feedrollers;
  private TalonFX m_shooter;
  private StatusSignal<Current> m_current_shooters;
  private boolean is_active = false;
  


  public ShooterTestCommand(TalonFX feedrollers, TalonFX shooter) {
    this.m_current_feedrollers = feedrollers.getStatorCurrent();
    this.m_shooter = shooter;
    this.m_current_shooters = m_shooter.getStatorCurrent();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_current_feedrollers.refresh();
    Current feedroller_amps = m_current_feedrollers.getValue();
    m_current_shooters.refresh();
    Current shooters_amps = m_current_shooters.getValue();

    if (feedroller_amps.gt(FEEDROLLER_TRIP) && !is_active) {
      m_timer.restart();
      is_active = true;
      return;
    }
    if (shooters_amps.gt(SHOOTERS_TRIP) && is_active) {
      //TODO: REPORT TIME:
      //Time time_to_report = Second.of(m_timer.get());
      m_timer.restart();
      is_active = false;
      return;
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
