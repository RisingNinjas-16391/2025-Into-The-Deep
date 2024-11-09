package org.firstinspires.ftc.teamcode.subsystems.slides.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.BetterElevatorFeedforward;
import org.firstinspires.ftc.teamcode.helpers.MathUtil;
import org.firstinspires.ftc.teamcode.helpers.Motor;
import org.firstinspires.ftc.teamcode.helpers.TrapezoidalSubsystemBase;

@Config
public class ElevatorSubsystem extends TrapezoidalSubsystemBase {
    private final Motor m_leftMotor;
    private final Motor m_rightMotor;
    private final PIDFController m_feedbackController = new PIDFController(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD, 0);
    private final BetterElevatorFeedforward m_feedforwardController = new BetterElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    public ElevatorSubsystem(HardwareMap hardwareMap){
        super(ElevatorConstants.TRAPEZOIDAL_CONSTRAINTS, ElevatorConstants.ERROR_MARGIN);

        m_leftMotor = new Motor(hardwareMap, "liftTop");
        m_rightMotor = new Motor(hardwareMap, "liftBottom");

        m_leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);

        m_leftMotor.stopAndResetEncoder();
        m_rightMotor.stopAndResetEncoder();

        m_leftMotor.setRunMode(Motor.RunMode.RawPower);
        m_rightMotor.setRunMode(Motor.RunMode.RawPower);

    }
    @Override
    public void periodic(){
        super.periodic();

        m_feedbackController.setPIDF(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kF);
        m_feedforwardController.setGains(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    }

    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry, "Elevator");
        telemetry.addData("Left Motor Ticks:", m_leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Ticks:", m_rightMotor.getCurrentPosition());
        telemetry.addData("Position Centimeters:", getExtensionMeters());
        telemetry.addData("Velocity Centimeters Per Second:", getVelocity());
    }

    /**
     * Uses trapezoidal motion-profiling to implement pseudo-positional control
     * Using this method disables velocity control
     *
     * @param extension the desired extension
     * @return boolean state to determine whether the input extension is safe
     */
    public boolean setGoal(double extension) {
        double adjustedExtension = MathUtil.clip(ElevatorConstants.LOWER_BOUND, extension, ElevatorConstants.UPPER_BOUND);
        super.setGoal(new TrapezoidProfile.State(adjustedExtension, 0));
        return adjustedExtension == extension;
    }

    public void forceGoal(double extension) {
        super.setGoal(new TrapezoidProfile.State(extension, 0));
    }

    /**
     * Set velocity of the extension using PID and feedforward control
     * If used externally, call disable() before using this method
     * Make sure to call enable() to resume positional control
     *
     * @param desiredVelocity in inches per second
     */
    public void setVelocity(double desiredVelocity) {
        super.setVelocity(desiredVelocity);
    }

    /**
     * Velocity measured in meters per second
    */
    public double getVelocity() {
        return m_leftMotor.getCorrectedVelocity() * ElevatorConstants.CENTIMETERS_PER_TICK;
    }

    public double getExtensionMeters() {
        return m_leftMotor.getCurrentPosition() * ElevatorConstants.CENTIMETERS_PER_TICK;
    }

    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getExtensionMeters(), getVelocity());
    }

    @Override
    protected double calculateVoltage(double velocity, double position) {
        return m_feedbackController.calculate(getExtensionMeters(), position) +
                m_feedforwardController.calculate(velocity, 0);
    }

    /**
     * Sets the voltage of the pivot motors
     *
     * @param voltage
     */
    @Override
    protected void setVoltage(double voltage) {
        m_leftMotor.set(voltage/12);
        m_rightMotor.set(voltage/12);
    }

    public void resetPosition() {
        m_leftMotor.stopAndResetEncoder();
        m_rightMotor.stopAndResetEncoder();
    }

    public double getCurrent() {
        return m_leftMotor.getCurrent(CurrentUnit.AMPS) + m_rightMotor.getCurrent(CurrentUnit.AMPS);
    }
}
