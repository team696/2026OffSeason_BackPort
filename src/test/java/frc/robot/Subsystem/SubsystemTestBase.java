package frc.robot.Subsystem;

import static org.junit.jupiter.api.Assertions.fail;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;

/**
 * Shared setup for subsystem tests. Starts the HAL in sim, enables the robot, and gives helpers to
 * run the command scheduler in real time (the Phoenix sim and the lib's sim Notifier both run on the
 * wall clock, so we have to actually wait).
 */
public abstract class SubsystemTestBase {
    protected static final double kLoopSeconds = 0.02;

    @BeforeAll
    static void startSim() {
        HAL.initialize(500, 0);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void cancelCommands() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Creates the subsystems and lets them sit for a second. The lib applies TalonFX configs on a
     * background thread and the Phoenix sim devices take a moment to boot, so a setpoint sent right
     * after construction can run with no PID gains. Call this from each test class's @BeforeAll.
     */
    protected static void warmUp(Runnable createSubsystems) {
        createSubsystems.run();
        runFor(1.0);
    }

    /** Runs one robot loop: keeps Phoenix enabled and runs the scheduler (which calls periodic()). */
    protected static void step() {
        Unmanaged.feedEnable(100);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().run();
        try {
            Thread.sleep((long) (kLoopSeconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** Runs robot loops for a number of seconds. */
    protected static void runFor(double seconds) {
        int loops = (int) Math.ceil(seconds / kLoopSeconds);
        for (int i = 0; i < loops; i++) {
            step();
        }
    }

    /** Runs robot loops until the condition is true, failing the test if it takes longer than the timeout. */
    protected static void runUntil(BooleanSupplier condition, double timeoutSeconds, String message) {
        int loops = (int) Math.ceil(timeoutSeconds / kLoopSeconds);
        for (int i = 0; i < loops; i++) {
            step();
            if (condition.getAsBoolean()) {
                return;
            }
        }
        fail(message + " (timed out after " + timeoutSeconds + " s)");
    }

    /** Schedules a command and runs one loop so it starts. */
    protected static void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
        step();
    }
}
