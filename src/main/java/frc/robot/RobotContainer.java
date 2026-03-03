// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.VerticalFeeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.util.SwerveTelemetry;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();
    private final VerticalFeeder verticalfeeder = new VerticalFeeder();
    private final Launcher launcher = new Launcher();
    private final Hood hood = new Hood();
    private final Climber climber = new Climber();
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final AimAndDriveCommand aimAndDriveCommands = new AimAndDriveCommand(
        swerve,
        forwardInput,
        leftInput
     );
     
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        hopper,
        verticalfeeder,
        launcher,
        hood,
        climber,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      
       // Subsystem initialization
        swerve = new Swerve();
        intake = new Intake();
        

      

        // Register Named Commands
        NamedCommands.registerCommand("intake", intake.intakeCommand());
        NamedCommands.registerCommand("aimAndShoot", subsystemCommands.aimAndDriveCommand());
        NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

        configureBindings();
       //  autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())

           .onTrue(intake.homingCommand());
           //.onTrue(climber.homingCommand());

       // driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        operator.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        operator.rightBumper().whileTrue(subsystemCommands.shootManually());

        operator.leftTrigger().whileTrue(intake.intakeCommand());
        operator.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        //operator.povUp().onTrue(climber.positionCommand(Climber.Position.HANGING));
        //operator.povDown().onTrue(climber.positionCommand(Climber.Position.HUNG));
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.povRight().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
}