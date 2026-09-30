package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(9, RobotConstants.RIOBUS),
	INTAKE_ROLLERS_FOLLOWER(10, RobotConstants.RIOBUS),
	INTAKE_RACK_AND_PINION(11, RobotConstants.RIOBUS),
	HOPPER_ROLLERS(12, RobotConstants.RIOBUS),
	HOPPER_PIVOT(13, RobotConstants.RIOBUS),
	FEEDER_ROLLERS(14, RobotConstants.RIOBUS),
	SHOOTER_MAIN(15, RobotConstants.RIOBUS),
	SHOOTER_FOLLOWER(16, RobotConstants.RIOBUS),
	HOOD(17, RobotConstants.RIOBUS),
	CANDLE(18 ,RobotConstants.RIOBUS),
	FEED_ROLLERS_FOLLOWER(19,RobotConstants.RIOBUS);
	// HOPPER_UPWARDS_MAIN(19, RobotConstants.RIOBUS),
	// HOPPER_UPWARDS_FOLLOWER(20, RobotConstants.RIOBUS);
	


	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}