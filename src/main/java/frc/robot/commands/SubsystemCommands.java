package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.VerticalFeeder;
import frc.robot.tannersCommands.TannersPassingCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Hopper hopper;
    private final VerticalFeeder verticalfeeder;
    private final Launcher launcher;
    private final Hood hood;
    private final Climber climber;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Hopper hopper,
        VerticalFeeder verticalfeeder,
        Launcher launcher,
        Hood hood,
        Climber climber,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.hopper = hopper;
        this.verticalfeeder = verticalfeeder;
        this.launcher = launcher;
        this.hood = hood;
        this.climber = climber;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Hopper hopper,
        VerticalFeeder verticalfeeder,
        Launcher launcher,
        Hood hood,
        Climber climber
    ) {
        this(
            swerve,
            intake,
            hopper,
            verticalfeeder,
            launcher,
            hood,
            climber,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(launcher, hood, () -> swerve.getState().Pose);
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
            //Commands.waitSeconds(5)
                .andThen(feed())
        );
    }

    public Command pass() {
            final TannersPassingCommand passCommand = new TannersPassingCommand(launcher, hood);
            return Commands.parallel(
                passCommand,
                Commands.waitUntil(() -> passCommand.isReadyToShoot())
                    .andThen(feed())
        );
    }

    public Command shootManually() {
        return launcher.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> launcher.stop());
    }

    public Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                verticalfeeder.feedCommand(),
                Commands.waitSeconds(1.5)
                    .andThen(hopper.feedCommand().alongWith(intake.agitateCommand()))
            )
        );
    }
}