package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.utilities.Util.logf;

import frc.robot.subsystems.Pose.PositionSubsystem;
import frc.robot.subsystems.shooter.CatchupSubsystem;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HotDog;

public class HubSubCommands {

    public Command shootBall(DrumstickSubsystem drumstickSubsystem, CatchupSubsystem catchupSubsystem, HotDog hotDog, PositionSubsystem positionSubsystem, HoodSubsystem hood) {
        return new SequentialCommandGroup(
            myLogf("Start shoot ball command %.3f, Hood pos %.3f, Distance to targ %.3f", positionSubsystem.getShooterRPM(), positionSubsystem.getHoodPosition(), positionSubsystem.getDistanceV2()),
            drumstickSubsystem.runToSpeed(positionSubsystem.getShooterRPM()),
            hood.setHoodPosition(positionSubsystem.getHoodPosition()),
            myLogf("Shoot speed OK"),
            hotDog.setVelocitySetpointCmd(3000),
            Commands.waitSeconds(.1),
            catchupSubsystem.setCatchupSetpointCmd(positionSubsystem.getShooterRPM()),
            myLogf("shoot ball complete"));
    }

    public Command stopShootBall(DrumstickSubsystem drumstickSubsystem, CatchupSubsystem catchupSubsystem, HotDog hotDog) {
        return new SequentialCommandGroup(
            drumstickSubsystem.stopShooter(),
            hotDog.stopHotDog(),
            catchupSubsystem.stopCatchup(),
            myLogf("Stop shoot ball command complete"));
    }

    // Old shoot command, doesn't work:

    // public Command shootCommand(HotDog hotDog, PositionSubsystem position, CatchupSubsystem catchup, DrumstickSubsystem drumstick, HoodSubsystem hood){
    //     return Commands.runOnce(()->logf("*****Starting shooter command")).andThen(
    //     drumstick.runToSpeed(position.getShooterRPM())).andThen( 
    //     Commands.runOnce(()->logf("*****Continuing shooter command"))).andThen(
    //     hood.setHoodPosition(position.getHoodPosition())).andThen(
    //     Commands.waitUntil(()->position.readyToShoot(drumstick, hood))).andThen(
    //     catchup.setCatchupSetpoint(40).alongWith(hotDog.setVelocitySetpoint(40))).andThen(
    //     Commands.runOnce(()->logf("*****Stopping shooter command")));
    // }
    
    public Command myLogf(String pattern, Object... args) {
        return new InstantCommand(() -> logf(pattern, args));
    }
}
