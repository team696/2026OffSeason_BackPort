package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(26, RobotConstants.RIOBUS),
	INTAKE_ROLLERS_FOLLOWER(9, RobotConstants.RIOBUS),
	INTAKE_RACK_AND_PINION(10, RobotConstants.CANIVORE_BUS),
	HOPPER_ROLLERS(11, RobotConstants.CANIVORE_BUS),
	FEEDER_ROLLERS(14, RobotConstants.CANIVORE_BUS),
	TUNNEL_ROLLERS_MAIN(15, RobotConstants.CANIVORE_BUS),
	TUNNEL_ROLLERS_FOLLOWER(16, RobotConstants.CANIVORE_BUS),
	SHOOTER_MAIN(17, RobotConstants.CANIVORE_BUS),
	SHOOTER_FOLLOWER(18, RobotConstants.CANIVORE_BUS),
	HOOD(20, RobotConstants.CANIVORE_BUS),
	CANDLE(21 ,RobotConstants.CANIVORE_BUS),
	HOPPER_CANRANGE(22, RobotConstants.CANIVORE_BUS),
	TUNNEL_CANRANGE(23, RobotConstants.CANIVORE_BUS),
	HOPPER_UPWARDS_MAIN(24, RobotConstants.CANIVORE_BUS),
	HOPPER_UPWARDS_FOLLOWER(25, RobotConstants.CANIVORE_BUS);

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}