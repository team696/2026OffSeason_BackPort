package frc.robot;

import java.lang.management.ManagementFactory;
import java.lang.management.OperatingSystemMXBean;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Log {
	public static boolean initialized = false;

	private static Notifier systemInformationThread;

	public static SEVERITY severity;

	/*
	 * Actually starts logging into a file, otherwise only NT values will be updated
	 */
	public static final void initialize(SEVERITY severity) {
		if (initialized == true)
			return;

		Log.severity = severity;

		if (severity == SEVERITY.NOTHING)
			return;

		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());

		if (Log.severity.level >= SEVERITY.DEBUG.level) {
			systemInformationThread = new Notifier(Log::logSystemInformation);
			systemInformationThread.setName("System Information");
			systemInformationThread.startPeriodic(0.02);
		}

		initialized = true;
	}

	public static void data(String name, Sendable sendable, SEVERITY severity) {
		if (severity.level > Log.severity.level)
			return;
		SmartDashboard.putData(name, sendable);
	}

	public static <T> void value(String name, T value, SEVERITY severity) {
		if (severity.level > Log.severity.level)
			return;
		if (value.getClass() == int.class || value.getClass() == double.class || value.getClass() == float.class
				|| value.getClass() == long.class) {
			SmartDashboard.putString(name, value.toString());
		} else {
			SmartDashboard.putString(name, value.toString());
		}
	}

	public static <T> void value(String name, T value) {
		if (severity.level > Log.severity.level)
			return;
		value(name, value, SEVERITY.DEBUG);
	}

	public static final CANStatus status = new CANStatus();
	public static final PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
	public static final OperatingSystemMXBean osBean = ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);

	public static final void logSystemInformation() {
    value("SystemStats/BrownedOut", RobotController.isBrownedOut());
    value("SystemStats/BatteryVoltage", pdh.getVoltage()); //switched to pdh to see the total voltage(12v) instead of the 5v inputed into the rio
    value("SystemStats/BatteryCurrent", pdh.getAllCurrents());//switched to the pdh to see the total current of the pdh, not rio
    
    // 3.3V Rail
    value("SystemStats/3v3Rail/Voltage", RobotController.getVoltage3V3());
    value("SystemStats/3v3Rail/Current", RobotController.getCurrent3V3());
    value("SystemStats/3v3Rail/Active", RobotController.getEnabled3V3());
    value("SystemStats/3v3Rail/CurrentFaults", RobotController.getFaultCount3V3());
    

	// Commented out cuz i think there is no 5v or 6v voltage rails on the new Systemcore
    // // 5V Rail
    // value("SystemStats/5vRail/Voltage", RobotController.getVoltage5V());
    // value("SystemStats/5vRail/Current", RobotController.getCurrent5V());
    // value("SystemStats/5vRail/Active", RobotController.getEnabled5V());
    // value("SystemStats/5vRail/CurrentFaults", RobotController.getFaultCount5V());
    
    // // 6V Rail
    // value("SystemStats/6vRail/Voltage", RobotController.getVoltage6V());
    // value("SystemStats/6vRail/Current", RobotController.getCurrent6V());
    // value("SystemStats/6vRail/Active", RobotController.getEnabled6V());
    // value("SystemStats/6vRail/CurrentFaults", RobotController.getFaultCount6V());
    
    var canStatus = RobotController.getCANStatus();
    value("SystemStats/CANBus/Utilization", canStatus.percentBusUtilization);
    value("SystemStats/CANBus/OffCount", canStatus.busOffCount);
    value("SystemStats/CANBus/TxFullCount", canStatus.txFullCount);
    value("SystemStats/CANBus/ReceiveErrorCount", canStatus.receiveErrorCount);
    value("SystemStats/CANBus/TransmitErrorCount", canStatus.transmitErrorCount);
    
    // The rest of your PDH and CPU code remains the same
    value("SystemStats/EpochTimeMicros", RobotController.getTime());
    value("SystemStats/PowerDistribution/Temperature", pdh.getTemperature());
    value("SystemStats/PowerDistribution/Voltage", pdh.getVoltage());
    SmartDashboard.putNumberArray("SystemStats/PowerDistribution/ChannelCurrent", pdh.getAllCurrents());
    value("SystemStats/PowerDistribution/TotalCurrent", pdh.getTotalCurrent());
    value("SystemStats/PowerDistribution/TotalPower", pdh.getTotalPower());
    value("SystemStats/PowerDistribution/TotalEnergy", pdh.getTotalEnergy());
    value("SystemStats/PowerDistribution/ChannelCount", pdh.getNumChannels());
    
    value("SystemStats/RAM/FreeMemoryMB", Runtime.getRuntime().freeMemory() / Math.pow(2, 20));
    value("SystemStats/RAM/UsedMemoryMB", (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / Math.pow(2, 20));
    value("SystemStats/CPU/Load", (osBean.getSystemLoadAverage() / osBean.getAvailableProcessors()) * 100);
    value("SystemStats/CPU/TemperatureC", RobotController.getCPUTemp());
}
	public static enum SEVERITY {
		NOTHING(0),
		MINIMAL(2),
		DEBUG(3),
		EVERYTHING(10);

		public final int level;

		SEVERITY(int l) {
			level = l;
		}
	}

}
