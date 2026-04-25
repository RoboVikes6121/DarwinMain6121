// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.Floorballs_backup_getMore;
import static frc.robot.generated.ChoreoTraj.Start_to_floorballs;
import static frc.robot.generated.ChoreoTraj.Start_to_floorballs_also_know_as_outhouse_from_center;
import static frc.robot.generated.ChoreoTraj.backup_to_shoot;
import static frc.robot.generated.ChoreoTraj.over_left;
import static frc.robot.generated.ChoreoTraj.over_right;
import static frc.robot.generated.ChoreoTraj.Left_Center_Auton;
import static frc.robot.generated.ChoreoTraj.Left_Auton_Grab_More_Balls;
import static frc.robot.generated.ChoreoTraj.Left2_Auton_Step1;
import static frc.robot.generated.ChoreoTraj.Left2_Auton_Step2;
import static frc.robot.generated.ChoreoTraj.Right2_Auton_Step1;
import static frc.robot.generated.ChoreoTraj.Right2_Auton_Step2;
import static frc.robot.generated.ChoreoTraj.gather_centerballs_right;
import static frc.robot.generated.ChoreoTraj.centerballs_back_to_hub_left;
import static frc.robot.generated.ChoreoTraj.centerballs_back_to_hub_right;
import static frc.robot.generated.ChoreoTraj.start_to_skyballs;
import static frc.robot.generated.ChoreoTraj.skyballs_to_shoot;
import static frc.robot.generated.ChoreoTraj.start_to_skyballs_from_bump;
import static frc.robot.generated.ChoreoTraj.outpost_to_depo;


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
        autoChooser.addRoutine("DepoAuto", this::depoAuton);
        autoChooser.addRoutine("centerDepoAuton", this::depoAutonFromCenter);
        autoChooser.addRoutine("leftCenterAuton", this::leftCenterAuton);
        autoChooser.addRoutine("rightCenterAuton", this::rightCenterAuton);
        autoChooser.addRoutine("DoubleLeftAuton", this::doubleLeftAuton);
        autoChooser.addRoutine("DoubRightAuton", this::doubleRightAuton);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine doubleRightAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("DoubleLeftAuton");
        final AutoTrajectory rightDoubleAuton1 = Right2_Auton_Step1.asAutoTraj(routine);
        final AutoTrajectory rightDoubleAuton2 = Right2_Auton_Step2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.parallel(
                rightDoubleAuton1.resetOdometry().andThen(rightDoubleAuton1.cmd()),
                Commands.waitSeconds(1.5).andThen(intake.intakeCommand().withTimeout(6.75))
            )
        );
        rightDoubleAuton1.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot().withTimeout(3.5),
                Commands.parallel(
                    rightDoubleAuton2.resetOdometry().andThen(rightDoubleAuton2.cmd()),
                    Commands.waitSeconds(1).andThen(intake.intakeCommand().withTimeout(7.75))
                )
            )
            
        ); 
        rightDoubleAuton2.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot().withTimeout(10)
            )
        );

        return routine;
    }

    private AutoRoutine doubleLeftAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("DoubleLeftAuton");
        final AutoTrajectory leftDoubleAuton1 = Left2_Auton_Step1.asAutoTraj(routine);
        final AutoTrajectory leftDoubleAuton2 = Left2_Auton_Step2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.parallel(
                leftDoubleAuton1.resetOdometry().andThen(leftDoubleAuton1.cmd()),
                Commands.waitSeconds(1).andThen(intake.intakeCommand().withTimeout(5.5))
            )
        );
        leftDoubleAuton1.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot().withTimeout(3.5),
                Commands.parallel(
                    leftDoubleAuton2.resetOdometry().andThen(leftDoubleAuton2.cmd()),
                    Commands.waitSeconds(1).andThen(intake.intakeCommand().withTimeout(7.75))
                )
            )
        ); 
        leftDoubleAuton2.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot().withTimeout(10)
            )
        );

        return routine;
    }

    private AutoRoutine depoAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("DepoAuto");
        final AutoTrajectory startToDepo = Start_to_floorballs.asAutoTraj(routine);
        final AutoTrajectory depoGetMore = Floorballs_backup_getMore.asAutoTraj(routine);
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
                depoGetMore.resetOdometry(),
                depoGetMore.cmd()

            )
        );
        depoGetMore.done().onTrue(
            Commands.sequence(
                intake.intakeCommand().withTimeout(2),
                backupToShoot.resetOdometry(),
                backupToShoot.cmd()
            )
        );
        backupToShoot.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot().withTimeout(15)
            )
        );

        //auto commands start here
     
        return routine;
    }

    private AutoRoutine depoAutonFromCenter() {
        final AutoRoutine routine = autoFactory.newRoutine("DepoAuto");
        final AutoTrajectory startToDepo = Start_to_floorballs_also_know_as_outhouse_from_center.asAutoTraj(routine);
        final AutoTrajectory depoGetMore = Floorballs_backup_getMore.asAutoTraj(routine);
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
                depoGetMore.resetOdometry(),
                depoGetMore.cmd()

            )
        );
        depoGetMore.done().onTrue(
            Commands.sequence(
                intake.intakeCommand().withTimeout(2),
                backupToShoot.resetOdometry(),
                backupToShoot.cmd()
            )
        );
        backupToShoot.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot().withTimeout(15)
            )
        );

        //auto commands start here
     
        return routine;
    }


    private AutoRoutine rightCenterAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("rightCenterAuton");
        final AutoTrajectory centerBallsRight = over_right.asAutoTraj(routine);
        final AutoTrajectory grabCenterBallsRight = gather_centerballs_right.asAutoTraj(routine);
        final AutoTrajectory centerBallsToHubRight = centerballs_back_to_hub_right.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                
                centerBallsRight.resetOdometry(),
                centerBallsRight.cmd()
            )
        );
        centerBallsRight.done().onTrue(
            Commands.parallel(
                grabCenterBallsRight.resetOdometry().andThen(grabCenterBallsRight.cmd()),
                intake.intakeCommand().withTimeout(7.5)
            )
         );
        grabCenterBallsRight.done().onTrue(
            Commands.sequence(
                centerBallsToHubRight.resetOdometry(),
                centerBallsToHubRight.cmd()
            )
        );
        centerBallsToHubRight.done().onTrue(
            subsystemCommands.aimAndShoot()
        );

        return routine;
    }

    private AutoRoutine leftCenterAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("leftCenterAuton");
        final AutoTrajectory leftAuton = Left_Center_Auton.asAutoTraj(routine);

            routine.active().onTrue(
            Commands.parallel(
                leftAuton.resetOdometry().andThen(leftAuton.cmd()),
                Commands.waitSeconds(1.5).andThen(intake.intakeCommand().withTimeout(5.5))
                
            )
        );
    leftAuton.done().onTrue(
                Commands.sequence(
                    subsystemCommands.aimAndShoot().withTimeout(6)
                )
            );
        
             return routine;
    }
}
