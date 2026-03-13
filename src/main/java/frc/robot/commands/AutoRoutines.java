// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.Floorballs_backup;
import static frc.robot.generated.ChoreoTraj.Start_to_floorballs;
import static frc.robot.generated.ChoreoTraj.backup_to_shoot;
import static frc.robot.generated.ChoreoTraj.over;


import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VerticalFeeder;

public final class AutoRoutines {
    private final Swerve swerve;
    private final Intake intake;
    private final Hopper hopper;
    private final VerticalFeeder verticalfeeder;
    private final Launcher launcher;
    private final Hood hood;
    private final Climber climber;
    private final Limelight limelight;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        Swerve swerve,
        Intake intake,
        Hopper hopper,
        VerticalFeeder verticalfeeder,
        Launcher launcher,
        Hood hood,
        Climber climber,
        Limelight limelight
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.hopper = hopper;
        this.verticalfeeder = verticalfeeder;
        this.launcher = launcher;
        this.hood = hood;
        this.climber = climber;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, hopper, verticalfeeder, launcher, hood, climber);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("DepoAuto", this::outpostAndDepotRoutine);
        autoChooser.addRoutine("over", this::overTheThing);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("DepoAuto");
        final AutoTrajectory startToOutpost = Start_to_floorballs.asAutoTraj(routine);
        final AutoTrajectory outpostBackup = Floorballs_backup.asAutoTraj(routine);
        final AutoTrajectory backupToShoot = backup_to_shoot.asAutoTraj(routine);

        


        routine.active().onTrue(
            Commands.sequence(
                
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );
        startToOutpost.done().onTrue(
            Commands.sequence(
                intake.intakeCommand().withTimeout(2),
                intake.stowCommand(),
                outpostBackup.resetOdometry(),
                outpostBackup.cmd()

            )
        );
        outpostBackup.done().onTrue(
            Commands.sequence(
                backupToShoot.resetOdometry(),
                backupToShoot.cmd(),
                subsystemCommands.aimAndShoot().withTimeout(15)
            )
        );
        //auto commands start here
     
        return routine;
    }

    private AutoRoutine overTheThing() {
        final AutoRoutine routine = autoFactory.newRoutine("over");
        final AutoTrajectory centerBalls = over.asAutoTraj(routine);

            routine.active().onTrue(
            Commands.sequence(
                
                centerBalls.resetOdometry(),
                centerBalls.cmd()
            )
        );
        /*centerBalls.done().onTrue(
            Commands.parallel(
                intake.intakeCommand().withTimeout(7.5),
                grabCenterBalls.resetOdometry
                grabCenterBalls.cmd()
            )
         );
        */
             return routine;
    }
}
