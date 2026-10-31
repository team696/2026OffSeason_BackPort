package frc.lib.bases;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.Util;
import java.util.Set;
import java.util.concurrent.CompletableFuture;

/**
 * Base subsystem for any subsystem that uses motors and requires precise position control.
 */
public class ServoMotorSubsystem<IO extends MotorIO> extends MotorSubsystem<IO> {
	protected final Angle epsilonThreshold;

	// Variables for handling homing logic
	protected boolean isHomingSubsystem;
	private ServoHomingConfig homingConfig;
	private boolean mHoming = false;
	private boolean mNeedsToHome = true;
	private DelayedBoolean mHomingDelay;

	// Set by zeroAgainstHardStopCommand once the position has been zeroed against a hard stop
	private boolean hasZeroed = false;
	private String zeroingState = "NOT_STARTED";
	private String zeroingFailure = "";
	private int zeroingAttempts;
	private double lastPreZeroPositionRadians = Double.NaN;
	private long inputSequence;

	/**
	 * Creates a *non-homing* ServoMotorSubsystem with a MotorIO, name for telemetry, and threshold for differences in measurement.
	 *
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for position.
	 */
	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold, boolean tuningMode) {
		super(io, name, tuningMode);
		this.isHomingSubsystem = false;
		this.epsilonThreshold = epsilonThreshold;
	}

	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold) {
		this(io, name, epsilonThreshold, false);
	}

	/**
	 * Creates a *homing* ServoMotorSubsystem with a MotorIO, name for telemetry, and threshold for differences in measurement.
	 *
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for position.
	 * @param config Homing configuration.
	 */
	public ServoMotorSubsystem(
			IO io, String name, Angle epsilonThreshold, boolean tuningMode, ServoHomingConfig config) {
		this(io, name, epsilonThreshold, tuningMode);
		this.isHomingSubsystem = true;
		homingConfig = config;
		mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), homingConfig.kHomingTimeout.in(Units.Seconds));
	}

	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold, ServoHomingConfig config) {
		this(io, name, epsilonThreshold, false, config);
	}

	@Override
	public void periodic() {
		super.periodic();
		inputSequence++;
		if (isHomingSubsystem) {
			if (mNeedsToHome && !mHoming && setpointNearHome() && nearHomingLocation()) {
				mHoming = true;
				useSoftLimits(false);
				mHomingDelay =
						new DelayedBoolean(Timer.getFPGATimestamp(), homingConfig.kHomingTimeout.in(Units.Seconds));
			}
			if (mHoming) {
				io.applySetpoint(Setpoint.withVoltageSetpoint(homingConfig.kHomingVoltage));
				if (mHomingDelay.update(
						Timer.getFPGATimestamp(),
						Math.abs(getVelocity().baseUnitMagnitude()) < homingConfig.kSetHomedVelocity.baseUnitMagnitude()
								&& DriverStation.isEnabled())) {
					setCurrentPosition(homingConfig.kHomePosition);
					applySetpoint(Setpoint.withMotionMagicSetpoint(homingConfig.kHomePosition));
					useSoftLimits(true);
					mNeedsToHome = false;
				}
			}
		}
	}

	/**
	 * Determines whether the currently set setpoint is near subsystem's home position.
	 *
	 * @return True if setpoint is near the home position, false if not.
	 */
	public boolean setpointNearHome() {
		return getSetpoint().mode.isPositionControl()
				&& Util.epsilonEquals(
						getSetpointDoubleInUnits(),
						homingConfig.kHomePosition.in(io.unitType),
						epsilonThreshold.in(io.unitType));
	}

	/**
	 * Determines whether the subsystem is near it's homing position.
	 *
	 * @return True if currently near home position, false if not.
	 */
	public boolean nearHomingLocation() {
		return nearPosition(homingConfig.kHomePosition);
	}

	/**
	 * Determines whether the subsystem is near it's position setpoint.
	 *
	 * @return True if currently near setpoint, false if not. Returns false if not in position control.
	 */
	public boolean nearPositionSetpoint() {
		return getSetpoint().mode.isPositionControl() && nearPosition(Units.Radians.of(getSetpoint().baseUnits));
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		return io.getSetpointDoubleInUnits();
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition) {
		return Util.epsilonEquals(
				getPosition().in(BaseUnits.AngleUnit),
				mechanismPosition.in(BaseUnits.AngleUnit),
				epsilonThreshold.in(BaseUnits.AngleUnit));
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @param epsilon Acceptable error range for position
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition, Angle epsilon) {
		return Util.epsilonEquals(
				getPosition().in(BaseUnits.AngleUnit),
				mechanismPosition.in(BaseUnits.AngleUnit),
				epsilon.in(BaseUnits.AngleUnit));
	}

	@Override
	public void applySetpoint(Setpoint setpoint) {
		super.applySetpoint(setpoint);
		if (isHomingSubsystem) {
			if (mHoming) {
				mHoming = false;
				useSoftLimits(true);
			}
			if (!setpointNearHome()) {
				mNeedsToHome = true;
			}
		}
	}

	/**
	 * Creates a Command that waits until the mechanism is near a given position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A wait command.
	 */
	public Command waitForPositionCommand(Angle mechanismPosition) {
		return Commands.waitUntil(() -> {
			return nearPosition(mechanismPosition);
		});
	}

	/**
	 * Creates a Command that goes to a setpoint and then waits until the mechanism is the setpoint's position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A new Command to apply setpoint and wait.
	 */
	public Command setpointCommandWithWait(Setpoint setpoint) {
		return waitForPositionCommand(BaseUnits.AngleUnit.of(setpoint.baseUnits))
				.deadlineFor(setpointCommand(setpoint));
	}

	/**
	 * Enables or disables soft limits.
	 *
	 * @param enable True to enable, soft to disable.
	 */
	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Sets neutral mode to brake or coast.
	 *
	 * @param wantsBrake True for brake, false for coast.
	 */
	public void setNeutralBrake(boolean wantsBrake) {
		io.setNeutralBrake(wantsBrake);
	}

	/**
	 * Set's the mechanism's current location as a given position.
	 *
	 * @param mechanismPosition the mechanism's position to set location as.
	 */
	public void setCurrentPosition(Angle position) {
		io.setCurrentPosition(position);
	}

	/**
	 * Whether {@link #zeroAgainstHardStopCommand} has successfully zeroed this mechanism since boot.
	 *
	 * @return True once the position has been zeroed against the hard stop.
	 */
	public boolean hasZeroed() {
		return hasZeroed;
	}

	public String getZeroingState() { return zeroingState; }
	public String getZeroingFailure() { return zeroingFailure; }
	public int getZeroingAttempts() { return zeroingAttempts; }
	public double getLastPreZeroPositionRadians() { return lastPreZeroPositionRadians; }

	/**
	 * Creates a Command that drives into a hard stop, zeroes the encoder there, and then applies a setpoint.
	 * If the stop is never found (timeout) or the zero never shows up in the position reading, the motor is
	 * left neutral, a warning is reported, and {@link #hasZeroed()} stays false so it can be retried.
	 *
	 * @param config Voltage, stall detection and timeouts for the zeroing.
	 * @param finishSetpoint Setpoint to apply after a successful zero.
	 * @return A command requiring this subsystem.
	 */
	public Command zeroAgainstHardStopCommand(ZeroConfig config, Setpoint finishSetpoint) {
		return new ZeroCommand(config, finishSetpoint);
	}

	/**
	 * Same as {@link #zeroAgainstHardStopCommand}, but does nothing if this mechanism already zeroed since boot.
	 * The check happens when the command starts, not when it's built.
	 */
	public Command zeroIfNeededCommand(ZeroConfig config, Setpoint finishSetpoint) {
		return Commands.defer(
				() -> hasZeroed ? Commands.none() : zeroAgainstHardStopCommand(config, finishSetpoint),
				Set.of(this));
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		io.initSendable(builder);
		if (isHomingSubsystem) {
			builder.addBooleanProperty("Homing", () -> mHoming, null);
			builder.addBooleanProperty("Needs to Home", () -> mNeedsToHome, null);
		}
	}

	/**
	 * Drives at a fixed voltage until the mechanism stalls against a hard stop, then zeroes the encoder.
	 *
	 * <p>The encoder is only zeroed once the new value shows up in {@link #getPosition()}: the zero is
	 * sent on a background thread, so applying a position setpoint right away could chase the old reading.
	 */
	private class ZeroCommand extends Command {
		private final ZeroConfig config;
		private final Setpoint driveSetpoint;
		private final Setpoint finishSetpoint;
		private final Timer runTimer = new Timer();
		private final Timer stalledTimer = new Timer();
		private final Timer confirmTimer = new Timer();
		private boolean zeroSent;
		private boolean confirmed;
		private boolean failed;
		private CompletableFuture<Boolean> resetAcknowledgement;
		private long acknowledgementInputSequence = -1;

		ZeroCommand(ZeroConfig config, Setpoint finishSetpoint) {
			this.config = config;
			this.finishSetpoint = finishSetpoint;
			this.driveSetpoint = Setpoint.withVoltageSetpoint(config.kVoltage);
			addRequirements(ServoMotorSubsystem.this);
			setName(ServoMotorSubsystem.this.name + " Zero Against Hard Stop");
		}

		@Override
		public void initialize() {
			hasZeroed = false;
			zeroingState = "DRIVING_TO_HOME";
			zeroingFailure = "";
			zeroingAttempts++;
			lastPreZeroPositionRadians = Double.NaN;
			runTimer.restart();
			stalledTimer.stop();
			stalledTimer.reset();
			confirmTimer.stop();
			confirmTimer.reset();
			zeroSent = false;
			confirmed = false;
			failed = false;
			resetAcknowledgement = null;
			acknowledgementInputSequence = -1;
			if (io.homingFeedbackValid(config.kMaxFeedbackAge)) {
				applySetpoint(driveSetpoint);
			} else {
				fail("INVALID_FEEDBACK", "position or velocity feedback is invalid or stale");
			}
		}

		private void fail(String state, String reason) {
			failed = true;
			zeroingState = state;
			zeroingFailure = reason;
			applySetpoint(Setpoint.withNeutralSetpoint());
		}

		@Override
		public void execute() {
			if (failed || confirmed) return;
			if (!io.homingFeedbackValid(config.kMaxFeedbackAge)) {
				fail("INVALID_FEEDBACK", "position or velocity feedback is invalid or stale");
				return;
			}
			if (!zeroSent) {
				if (runTimer.hasElapsed(config.kTimeout.in(Units.Seconds))) {
					fail("HOME_TIMEOUT", "did not reach a plausible home before timeout; encoder was not zeroed");
					return;
				}
				applySetpoint(driveSetpoint);
				// the mechanism starts at rest, so don't call that a stall until it's had time to get moving
				boolean stopped = runTimer.hasElapsed(config.kStartupGrace.in(Units.Seconds))
						&& Math.abs(getVelocity().baseUnitMagnitude()) < config.kStalledVelocity.baseUnitMagnitude();
				if (stopped) {
					stalledTimer.start();
				} else {
					stalledTimer.stop();
					stalledTimer.reset();
					zeroingState = "DRIVING_TO_HOME";
				}

				if (stalledTimer.hasElapsed(config.kStalledTime.in(Units.Seconds))) {
					lastPreZeroPositionRadians = getPosition().baseUnitMagnitude();
					// This is a plausibility guard against a known far-from-home stall, not proof of
					// physical contact. A relative encoder can already have an incorrect reference.
					if (config.kMaxPreZeroPositionError != null
							&& !nearPosition(config.kZeroPosition, config.kMaxPreZeroPositionError)) {
						// Bumper contact can pause the rack before the real hard stop. Keep the
						// same bounded homing effort instead of ending the button's one attempt.
						zeroingState = "BLOCKED_AWAY_FROM_HOME";
						return;
					}
					applySetpoint(Setpoint.withNeutralSetpoint());
					resetAcknowledgement = io.setCurrentPositionForHoming(config.kZeroPosition);
					zeroSent = true;
					zeroingState = "WAITING_FOR_ZERO_ACK";
					confirmTimer.restart();
				}
			} else {
				if (resetAcknowledgement.isCompletedExceptionally()
						|| (resetAcknowledgement.isDone() && !resetAcknowledgement.getNow(false))) {
					fail("ZERO_RESET_FAILED", "device did not acknowledge the encoder reset");
					return;
				}
				if (resetAcknowledgement.isDone()) {
					if (acknowledgementInputSequence < 0) {
						acknowledgementInputSequence = inputSequence;
						zeroingState = "WAITING_FOR_ZERO_FEEDBACK";
					} else if (inputSequence > acknowledgementInputSequence
							&& nearPosition(config.kZeroPosition, config.kConfirmTolerance)) {
						confirmed = true;
					}
				}
				if (!confirmed && confirmTimer.hasElapsed(config.kConfirmTimeout.in(Units.Seconds))) {
					fail("ZERO_CONFIRM_TIMEOUT", "encoder reset was not acknowledged and confirmed before timeout");
				}
			}
		}

		@Override
		public boolean isFinished() {
			return confirmed || failed;
		}

		@Override
		public void end(boolean interrupted) {
			if (confirmed && !interrupted) {
				hasZeroed = true;
				zeroingState = "HOMED";
				applySetpoint(finishSetpoint);
				return;
			}
			// never leave the voltage request behind: it would start pushing into the stop again on re-enable
			hasZeroed = false;
			if (resetAcknowledgement != null && !resetAcknowledgement.isDone()) {
				resetAcknowledgement.cancel(false);
			}
			if (interrupted) {
				zeroingState = "INTERRUPTED";
				zeroingFailure = "homing was interrupted; position remains unverified";
			}
			applySetpoint(Setpoint.withNeutralSetpoint());
			if (failed) {
				DriverStation.reportWarning(
						ServoMotorSubsystem.this.name + " could not zero against its hard stop ("
								+ zeroingFailure
								+ "); position is still unknown",
						false);
			}
		}
	}

	/**
	 * Configuration for zeroing a ServoMotorSubsystem against a hard stop.
	 */
	public static class ZeroConfig {
		/** Voltage to drive at; the sign must point toward the hard stop. */
		public Voltage kVoltage;
		/** Position the mechanism is at when it is sitting on the hard stop. */
		public Angle kZeroPosition;
		/** How close the position reading must get to kZeroPosition to count the zero as applied. */
		public Angle kConfirmTolerance;
		/** Below this speed the mechanism counts as stopped. */
		public AngularVelocity kStalledVelocity;
		/** Optional bound on the old encoder reading before accepting a stall as home. Not an absolute sensor. */
		public Angle kMaxPreZeroPositionError;
		/** Optional maximum hardware feedback age; IO always rejects non-finite feedback. */
		public Time kMaxFeedbackAge;
		/** Time to ignore stalls after starting, while the mechanism gets moving. */
		public Time kStartupGrace;
		/** How long the mechanism must stay stopped to count as against the hard stop. */
		public Time kStalledTime;
		/** Give up if the stop hasn't been found by then. */
		public Time kTimeout;
		/** Give up if the zero hasn't shown up in the position reading by then. */
		public Time kConfirmTimeout;
	}

	/**
	 * Configuration to make a homing ServoMotorSubsystem
	 */
	public static class ServoHomingConfig {
		public Angle kHomePosition;
		public Voltage kHomingVoltage;
		public Time kHomingTimeout;
		public AngularVelocity kSetHomedVelocity;
	}
}
