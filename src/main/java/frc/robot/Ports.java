package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(8, RobotConstants.rio),
	INTAKE_ROLLERS_FOLLOWER(9, RobotConstants.rio),
	INTAKE_PIVOT(10, RobotConstants.canivore1),
	HOPPER_ROLLERS(11, RobotConstants.canivore1),
	FEEDER_ROLLERS(14, RobotConstants.canivore1),
	TUNNEL_ROLLERS_MAIN(15, RobotConstants.canivore1),
	TUNNEL_ROLLERS_FOLLOWER(16, RobotConstants.canivore1),
	SHOOTER_MAIN(17, RobotConstants.canivore1),
	SHOOTER_FOLLOWER(18, RobotConstants.canivore1),
	HOOD(20, RobotConstants.canivore1),
	CANDLE(21 ,RobotConstants.canivore1),
	HOPPER_CANRANGE(22, RobotConstants.canivore1),
	TUNNEL_CANRANGE(23, RobotConstants.canivore1),
	HOPPER_UPWARDS_MAIN(24, RobotConstants.canivore1),
	HOPPER_UPWARDS_FOLLOWER(25, RobotConstants.canivore1);

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}