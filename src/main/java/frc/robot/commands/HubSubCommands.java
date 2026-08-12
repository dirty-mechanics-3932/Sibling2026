package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.utilities.Util.logf;
import frc.robot.subsystems.hotDog.HotDog;
import frc.robot.subsystems.shooter.CatchupSubsystem;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.PositionSubsystem;

public class HubSubCommands {
    
    public Command shootCommand(HotDog hotDog, PositionSubsystem position, CatchupSubsystem catchup, DrumstickSubsystem drumstick, HoodSubsystem hood){
        return Commands.runOnce(()->logf("*****Starting shooter command")).andThen(
        drumstick.shootCommand(position.getShooterRPM())).andThen( 
        Commands.runOnce(()->logf("*****Continuing shooter command"))).andThen(
        hood.setHoodPosition(position.getHoodPosition())).andThen(
        Commands.waitUntil(()->position.readyToShoot(drumstick, hood))).andThen(
        catchup.setCatchupSetpoint(40).alongWith(hotDog.setVelocitySetpoint(40))).andThen(
        Commands.runOnce(()->logf("*****Stopping shooter command")));
    }

}
