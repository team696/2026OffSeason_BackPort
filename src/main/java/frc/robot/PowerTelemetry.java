package frc.robot;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

/** Records power samples directly to WPILOG, independently of dashboard subscriptions. */
public final class PowerTelemetry {
    private static final String ROOT = "/Power/PDH/";
    private final PowerDistribution pdh;
    private final DoubleLogEntry[] channels;
    private final DoubleArrayLogEntry channelCurrents;
    private final DoubleLogEntry channelSum;
    private final DoubleLogEntry totalCurrent;
    private final DoubleLogEntry voltage;
    private final BooleanArrayLogEntry breakerFaults;
    private final BooleanArrayLogEntry stickyBreakerFaults;
    private final BooleanLogEntry brownout;
    private final BooleanLogEntry canWarning;
    private final BooleanLogEntry hardwareFault;
    private final BooleanLogEntry stickyBrownout;
    private final BooleanLogEntry stickyCanWarning;
    private final BooleanLogEntry stickyCanBusOff;
    private final BooleanLogEntry stickyHasReset;
    private final DoubleLogEntry rioVoltage;
    private final BooleanLogEntry rioBrownout;
    private final DoubleLogEntry canUtilization;
    private final IntegerLogEntry canReceiveErrors;
    private final IntegerLogEntry canTransmitErrors;
    private final IntegerLogEntry canBusOffCount;

    public PowerTelemetry(PowerDistribution pdh, DataLog log) {
        this.pdh = pdh;
        channels = new DoubleLogEntry[pdh.getNumChannels()];
        for (int i = 0; i < channels.length; i++) {
            channels[i] = new DoubleLogEntry(log, ROOT + "Channel" + i + "/CurrentAmps");
        }
        channelCurrents = new DoubleArrayLogEntry(log, ROOT + "ChannelCurrentAmps");
        channelSum = new DoubleLogEntry(log, ROOT + "ChannelSumCurrentAmps");
        totalCurrent = new DoubleLogEntry(log, ROOT + "TotalCurrentAmps");
        voltage = new DoubleLogEntry(log, ROOT + "VoltageVolts");
        breakerFaults = new BooleanArrayLogEntry(log, ROOT + "BreakerFaults");
        stickyBreakerFaults = new BooleanArrayLogEntry(log, ROOT + "StickyBreakerFaults");
        brownout = new BooleanLogEntry(log, ROOT + "Brownout");
        canWarning = new BooleanLogEntry(log, ROOT + "CANWarning");
        hardwareFault = new BooleanLogEntry(log, ROOT + "HardwareFault");
        stickyBrownout = new BooleanLogEntry(log, ROOT + "StickyBrownout");
        stickyCanWarning = new BooleanLogEntry(log, ROOT + "StickyCANWarning");
        stickyCanBusOff = new BooleanLogEntry(log, ROOT + "StickyCANBusOff");
        stickyHasReset = new BooleanLogEntry(log, ROOT + "StickyHasReset");
        rioVoltage = new DoubleLogEntry(log, "/Power/RoboRIO/InputVoltageVolts");
        rioBrownout = new BooleanLogEntry(log, "/Power/RoboRIO/BrownedOut");
        canUtilization = new DoubleLogEntry(log, "/Power/RoboRIO/CANUtilization");
        canReceiveErrors = new IntegerLogEntry(log, "/Power/RoboRIO/CANReceiveErrors");
        canTransmitErrors = new IntegerLogEntry(log, "/Power/RoboRIO/CANTransmitErrors");
        canBusOffCount = new IntegerLogEntry(log, "/Power/RoboRIO/CANBusOffCount");
        new IntegerLogEntry(log, ROOT + "Module").append(pdh.getModule());
        new StringLogEntry(log, ROOT + "ModuleType").append(pdh.getType().name());
        new IntegerLogEntry(log, ROOT + "ChannelCount").append(channels.length);
    }

    /** Called each 20 ms robot loop, including disabled; does not change any outputs. */
    public void record() {
        long timestamp = RobotController.getFPGATime();
        double[] currents = pdh.getAllCurrents();
        double sum = 0.0;
        for (int i = 0; i < channels.length; i++) {
            channels[i].append(currents[i], timestamp);
            sum += currents[i];
        }
        channelCurrents.append(currents, timestamp);
        channelSum.append(sum, timestamp);
        totalCurrent.append(pdh.getTotalCurrent(), timestamp);
        voltage.append(pdh.getVoltage(), timestamp);

        PowerDistributionFaults faults = pdh.getFaults();
        PowerDistributionStickyFaults stickyFaults = pdh.getStickyFaults();
        boolean[] breakers = new boolean[channels.length];
        boolean[] stickyBreakers = new boolean[channels.length];
        for (int i = 0; i < channels.length; i++) {
            breakers[i] = faults.getBreakerFault(i);
            stickyBreakers[i] = stickyFaults.getBreakerFault(i);
        }
        breakerFaults.append(breakers, timestamp);
        stickyBreakerFaults.append(stickyBreakers, timestamp);
        brownout.append(faults.Brownout, timestamp);
        canWarning.append(faults.CanWarning, timestamp);
        hardwareFault.append(faults.HardwareFault, timestamp);
        stickyBrownout.append(stickyFaults.Brownout, timestamp);
        stickyCanWarning.append(stickyFaults.CanWarning, timestamp);
        stickyCanBusOff.append(stickyFaults.CanBusOff, timestamp);
        stickyHasReset.append(stickyFaults.HasReset, timestamp);
        rioVoltage.append(RobotController.getInputVoltage(), timestamp);
        rioBrownout.append(RobotController.isBrownedOut(), timestamp);
        var canStatus = RobotController.getCANStatus();
        canUtilization.append(canStatus.percentBusUtilization, timestamp);
        canReceiveErrors.append(canStatus.receiveErrorCount, timestamp);
        canTransmitErrors.append(canStatus.transmitErrorCount, timestamp);
        canBusOffCount.append(canStatus.busOffCount, timestamp);
    }
}
