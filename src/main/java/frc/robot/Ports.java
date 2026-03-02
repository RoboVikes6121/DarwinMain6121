package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kCANBus = new CANBus("CANivore6121", "./logs/example.hoot");

    // Talon FX IDs
    public static final int rollerMotor = 31;
    public static final int pivotMotor = 30;
    public static final int hopperMotor = 32;
    public static final int verticalFeederModer = 33;
    public static final int leftLauncherMotor = 40;
    public static final int centerLauncherMotor = 41;
    public static final int rightLauncherMotor = 42;
    public static final int climberMotor = 60;

    // PWM Ports
    public static final int LeftServo = 1;
    public static final int RightServo = 0;
}