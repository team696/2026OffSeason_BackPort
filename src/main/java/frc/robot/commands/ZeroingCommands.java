package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;

/**
 * Zeroes the hood and the intake rack against their hard stops. The encoders only count from wherever
 * they were at boot, so this runs once on the first enable and then the mechanisms hold their targets.
 */
public class ZeroingCommands {

    /** True while at least one of the mechanisms still hasn't been zeroed since boot. */
    public static boolean needed() {
        return !Hood.get().hasZeroed() || !Intake.RackAndPinion.get().hasZeroed();
    }

    /**
     * Zeros whichever of the hood and rack haven't been zeroed yet, both at the same time. Button presses that
     * need those subsystems are ignored until it finishes (about a second) instead of cancelling it halfway.
     * Returns a new Command every call, so it can be scheduled on every enable.
     */
    public static Command zeroIfNeeded() {
        return Commands.parallel(
                Hood.get().zeroCommand(),
                Intake.RackAndPinion.get().zeroCommand())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("Zero Hood and Rack");
    }
}
