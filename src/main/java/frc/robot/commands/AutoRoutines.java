// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.Floorballs_backup;
import static frc.robot.generated.ChoreoTraj.Start_to_floorballs;
import static frc.robot.generated.ChoreoTraj.backup_to_shoot;
import static frc.robot.generated.ChoreoTraj.over;
import static frc.robot.generated.ChoreoTraj.gather_centerballs;
import static frc.robot.generated.ChoreoTraj.centerballs_back_to_hub;
import static frc.robot.generated.ChoreoTraj.start_to_skyballs;
import static frc.robot.generated.ChoreoTraj.skyballs_to_shoot;
import static frc.robot.generated.ChoreoTraj.backup_to_shoot_side;


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
        autoChooser.addRoutine("Outpost", this::outpost);
        autoChooser.addRoutine("DepoSideShoot", this::depoSideShoot);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine depoSideShoot() {
        final AutoRoutine routine = autoFactory.newRoutine("DepoSideShoot");
        final AutoTrajectory startToDepo = Start_to_floorballs.asAutoTraj(routine);
        final AutoTrajectory depoBackup = Floorballs_backup.asAutoTraj(routine);
        final AutoTrajectory backupToShootSide = backup_to_shoot_side.asAutoTraj(routine);

        


        routine.active().onTrue(
            Commands.sequence(
                
                startToDepo.resetOdometry(),
                startToDepo.cmd()
            )
        );
        startToDepo.done().onTrue(
            Commands.sequence(
                intake.intakeCommand().withTimeout(2),
                intake.stowCommand(),
                depoBackup.resetOdometry(),
                depoBackup.cmd()

            )
        );
        depoBackup.done().onTrue(
            Commands.sequence(
                backupToShootSide.resetOdometry(),
                backupToShootSide.cmd(),
                subsystemCommands.aimAndShoot().withTimeout(15)
            )
        );
        //auto commands start here
     
        return routine;
    }



    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("DepoAuto");
        final AutoTrajectory startToDepo = Start_to_floorballs.asAutoTraj(routine);
        final AutoTrajectory depoBackup = Floorballs_backup.asAutoTraj(routine);
        final AutoTrajectory backupToShoot = backup_to_shoot.asAutoTraj(routine);

        


        routine.active().onTrue(
            Commands.sequence(
                
                startToDepo.resetOdometry(),
                startToDepo.cmd()
            )
        );
        startToDepo.done().onTrue(
            Commands.sequence(
                intake.intakeCommand().withTimeout(2),
                intake.stowCommand(),
                depoBackup.resetOdometry(),
                depoBackup.cmd()

            )
        );
        depoBackup.done().onTrue(
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
        final AutoTrajectory grabCenterBalls = gather_centerballs.asAutoTraj(routine);
        final AutoTrajectory centerBallsToHub = centerballs_back_to_hub.asAutoTraj(routine);

            routine.active().onTrue(
            Commands.sequence(
                
                centerBalls.resetOdometry(),
                centerBalls.cmd()
            )
        );
        /*centerBalls.done().onTrue(
            Commands.parallel(
                intake.intakeCommand().withTimeout(7.5),
                grabCenterBalls.resetOdometry(),
                grabCenterBalls.cmd()
                    .andThen(intake.stowCommand())
            )
         );
        grabCenterBalls.done().onTrue(
            Commands.sequence(
                centerBallsToHub.resetOdometry(),
                centerBallsToHub.cmd()
            )
        );
        centerBallsToHub.done().onTrue(
            subsystemCommands.aimAndShoot()
        );
        */
             return routine;
    }

    private AutoRoutine outpost() {
        final AutoRoutine routine = autoFactory.newRoutine("outpost");
        final AutoTrajectory startToOutpost = start_to_skyballs.asAutoTraj(routine);
        final AutoTrajectory outpostToShoot = skyballs_to_shoot.asAutoTraj(routine);

            routine.active().onTrue(
            Commands.sequence(
                
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );
        startToOutpost.done().onTrue(
            Commands.sequence(
                intake.intakeCommand().withTimeout(3),
                outpostToShoot.resetOdometry(),
                startToOutpost.cmd()
            )
        );
        startToOutpost.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
            )  
        );


    return routine;
    }
}
