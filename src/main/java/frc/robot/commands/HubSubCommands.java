package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hotDog.HotDog;
import frc.robot.subsystems.shooter.CatchupSubsystem;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.PositionSubsystem;

public class HubSubCommands {
    
    public Command shootCommand(HotDog hotDog, PositionSubsystem position, CatchupSubsystem catchup, DrumstickSubsystem drumstick, HoodSubsystem hood){
        return Commands.sequence(drumstick.setVelocitySetpoint(position.getShooterRPM()), 
        hood.setTargetPosition(position.getHoodPosition()),
        (catchup.setVelocitySetpoint(40).alongWith(hotDog.setVelocitySetpoint(40)).onlyIf(()->position.readyToShoot(drumstick, hood)))
        );
    }

}
