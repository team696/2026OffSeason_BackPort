package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(9, RobotConstants.RIOBUS),
	INTAKE_ROLLERS_FOLLOWER(10, RobotConstants.RIOBUS),
	INTAKE_RACK_AND_PINION(11, RobotConstants.RIOBUS),
	HOPPER_ROLLERS(12, RobotConstants.CANIVORE_BUS),
	FEEDER_ROLLERS(13, RobotConstants.CANIVORE_BUS),
	TUNNEL_ROLLERS_MAIN(14, RobotConstants.CANIVORE_BUS),
	TUNNEL_ROLLERS_FOLLOWER(15, RobotConstants.CANIVORE_BUS),
	SHOOTER_MAIN(16, RobotConstants.CANIVORE_BUS),
	SHOOTER_FOLLOWER(17, RobotConstants.CANIVORE_BUS),
	HOOD(18, RobotConstants.CANIVORE_BUS),
	CANDLE(19 ,RobotConstants.CANIVORE_BUS),
	HOPPER_CANRANGE(20, RobotConstants.CANIVORE_BUS),
	TUNNEL_CANRANGE(21, RobotConstants.CANIVORE_BUS),
	HOPPER_UPWARDS_MAIN(22, RobotConstants.CANIVORE_BUS),
	HOPPER_UPWARDS_FOLLOWER(23, RobotConstants.CANIVORE_BUS);

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}