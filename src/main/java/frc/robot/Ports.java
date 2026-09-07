package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(9, RobotConstants.RIOBUS),
	INTAKE_ROLLERS_FOLLOWER(10, RobotConstants.RIOBUS),
	INTAKE_RACK_AND_PINION(11, RobotConstants.RIOBUS),
	HOPPER_ROLLERS(12, RobotConstants.RIOBUS),
	FEEDER_ROLLERS(13, RobotConstants.RIOBUS),
	TUNNEL_ROLLERS_MAIN(14, RobotConstants.RIOBUS),
	TUNNEL_ROLLERS_FOLLOWER(15, RobotConstants.RIOBUS),
	SHOOTER_MAIN(16, RobotConstants.RIOBUS),
	SHOOTER_FOLLOWER(17, RobotConstants.RIOBUS),
	HOOD(18, RobotConstants.RIOBUS),
	CANDLE(19 ,RobotConstants.RIOBUS),
	HOPPER_CANRANGE(20, RobotConstants.RIOBUS),
	TUNNEL_CANRANGE(21, RobotConstants.RIOBUS),
	HOPPER_UPWARDS_MAIN(22, RobotConstants.RIOBUS),
	HOPPER_UPWARDS_FOLLOWER(23, RobotConstants.RIOBUS);

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}