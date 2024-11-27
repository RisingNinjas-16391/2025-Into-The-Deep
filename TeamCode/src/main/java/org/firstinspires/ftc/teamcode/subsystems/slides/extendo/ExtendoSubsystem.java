package org.firstinspires.ftc.teamcode.subsystems.slides.extendo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.BetterElevatorFeedforward;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    private final MotorEx m_motor;
    private final PIDController m_feedbackController = new PIDController(ExtendoConstants.kP,ExtendoConstants.kI,ExtendoConstants.kD);
    private final BetterElevatorFeedforward m_feedforwardController = new BetterElevatorFeedforward(ExtendoConstants.kS, ExtendoConstants.kG, ExtendoConstants.kV, ExtendoConstants.kA);

    private final Telemetry m_telemetry;

    private final TrapezoidProfile.Constraints constraints = ExtendoConstants.TRAPEZOIDAL_CONSTRAINTS;

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State forwardGoal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ExtendoSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_motor = new MotorEx(hardwareMap, "extension");

        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        m_motor.setInverted(true);

        m_motor.stopAndResetEncoder();

        m_motor.setRunMode(Motor.RunMode.RawPower);

        m_telemetry = telemetry;

        goal = new TrapezoidProfile.State(getExtensionMeters(), 0);
        forwardGoal = goal;
        setpoint = goal;
    }
    @Override
    public void periodic(){
        setVoltage(calculateVoltage(goal.velocity, goal.position));

        m_feedbackController.setPID(ExtendoConstants.kP, ExtendoConstants.kI, ExtendoConstants.kD);
        m_feedforwardController.setGains(ExtendoConstants.kS, ExtendoConstants.kG, ExtendoConstants.kV, ExtendoConstants.kA);

        m_telemetry.addLine("Extendo");
        m_telemetry.addData("Motor Ticks:", m_motor.getCurrentPosition());
        m_telemetry.addData("Position Centimeters:", getExtensionMeters());
        m_telemetry.addData("Velocity Centimeters Per Second:", getVelocity());
    }

    public void setPosition(double extension) {
        double adjustedExtension = MathUtil.clamp(ExtendoConstants.LOWER_BOUND, extension, ExtendoConstants.UPPER_BOUND);
        goal = new TrapezoidProfile.State(extension, 0);
    }

    public void incrementPosition(double deltaMeters) {
        goal.position += deltaMeters;
    }

    public double getVelocity() {
        return m_motor.getCorrectedVelocity() * ExtendoConstants.CENTIMETERS_PER_TICK;
    }

    public double getExtensionMeters() {
        return m_motor.getCurrentPosition() * ExtendoConstants.CENTIMETERS_PER_TICK;
    }

    private TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getExtensionMeters(), getVelocity());
    }

    private double calculateVoltage(double velocity, double position) {
        return m_feedbackController.calculate(getExtensionMeters(), position) +
                m_feedforwardController.calculate(velocity, 0);
    }

    private void setVoltage(double voltage) {
        m_motor.set(voltage/12);
    }

    public void resetPosition() {
        m_motor.stopAndResetEncoder();
    }

    public boolean isFinished() {
        return Math.abs(goal.position - getState().position) < 2;
    }
}
