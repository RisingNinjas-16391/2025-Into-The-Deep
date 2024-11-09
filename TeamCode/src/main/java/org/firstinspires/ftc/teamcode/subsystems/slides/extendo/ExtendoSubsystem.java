package org.firstinspires.ftc.teamcode.subsystems.slides.extendo;

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
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorConstants;

@Config
public class ExtendoSubsystem extends TrapezoidalSubsystemBase {
    private final Motor m_motor;
    private final PIDFController m_feedbackController = new PIDFController(ExtendoConstants.kP,ExtendoConstants.kI,ExtendoConstants.kD, 0);
    private final BetterElevatorFeedforward m_feedforwardController = new BetterElevatorFeedforward(ExtendoConstants.kS, ExtendoConstants.kG, ExtendoConstants.kV, ExtendoConstants.kA);

    public ExtendoSubsystem(HardwareMap hardwareMap){
        super(ExtendoConstants.TRAPEZOIDAL_CONSTRAINTS, ExtendoConstants.ERROR_MARGIN);

        m_motor = new Motor(hardwareMap, "extension");

        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        m_motor.setInverted(true);

        m_motor.stopAndResetEncoder();

        m_motor.setRunMode(Motor.RunMode.RawPower);

    }
    @Override
    public void periodic(){
        super.periodic();

        m_feedbackController.setPIDF(ExtendoConstants.kP, ExtendoConstants.kI, ExtendoConstants.kD, ExtendoConstants.kF);
        m_feedforwardController.setGains(ExtendoConstants.kS, ExtendoConstants.kG, ExtendoConstants.kV, ExtendoConstants.kA);

    }

    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry, "Extendo");
        telemetry.addData("Motor Ticks:", m_motor.getCurrentPosition());
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
        double adjustedExtension = MathUtil.clip(ExtendoConstants.LOWER_BOUND, extension, ExtendoConstants.UPPER_BOUND);
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
        return m_motor.getCorrectedVelocity() * ExtendoConstants.CENTIMETERS_PER_TICK;
    }

    public double getExtensionMeters() {
        return m_motor.getCurrentPosition() * ExtendoConstants.CENTIMETERS_PER_TICK;
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
        m_motor.set(voltage/12);
    }

    public void resetPosition() {
        m_motor.stopAndResetEncoder();
    }

    public double getCurrent() {
        return m_motor.getCurrent(CurrentUnit.AMPS);
    }
}
