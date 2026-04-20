package frc.robot.tannersCommands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Launcher;

public class TannersDefaultSpeedCommand extends Command {

    private final Launcher launcher;


    public TannersDefaultSpeedCommand(Launcher launcher) {
        this.launcher = launcher;
    
        
        addRequirements(launcher);
    }



    @Override
    public void execute() {
        launcher.defaultSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}