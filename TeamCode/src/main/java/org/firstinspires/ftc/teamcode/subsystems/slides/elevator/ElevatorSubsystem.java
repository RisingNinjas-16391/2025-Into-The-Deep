package org.firstinspires.ftc.teamcode.subsystems.slides.elevator;

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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Config
public class ElevatorSubsystem extends SubsystemBase {
    NetworkTable elev;
    private final MotorEx m_leftMotor;
    private final MotorEx m_rightMotor;
    private final PIDController m_feedbackController = new PIDController(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD);
    private final BetterElevatorFeedforward m_feedforwardController = new BetterElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    private final Telemetry m_telemetry;

    private final TrapezoidProfile.Constraints constraints = ElevatorConstants.TRAPEZOIDAL_CONSTRAINTS;

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State forwardGoal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ElevatorSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_leftMotor = new MotorEx(hardwareMap, "liftTop");
        m_rightMotor = new MotorEx(hardwareMap, "liftBottom");

        m_leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);

        m_leftMotor.stopAndResetEncoder();
        m_rightMotor.stopAndResetEncoder();

        m_leftMotor.setRunMode(Motor.RunMode.RawPower);
        m_rightMotor.setRunMode(Motor.RunMode.RawPower);

        m_telemetry = telemetry;

        goal = new TrapezoidProfile.State(getExtensionMeters(), 0);
        forwardGoal = goal;
        setpoint = goal;

        elev = NetworkTableInstance.getDefault().getTable("Elevator");
    }
    @Override
    public void periodic(){
        setVoltage(calculateVoltage(goal.velocity, goal.position));

        m_feedbackController.setPID(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        m_feedforwardController.setGains(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

        m_telemetry.addLine("Elevator");
        m_telemetry.addData("Left Motor Ticks:", m_leftMotor.getCurrentPosition());
        m_telemetry.addData("Right Motor Ticks:", m_rightMotor.getCurrentPosition());
        m_telemetry.addData("Position Meters:", getExtensionMeters());
        m_telemetry.addData("Velocity Meters Per Second:", getVelocity());

        elev.getEntry("Position Meters").setDouble(getExtensionMeters());
        elev.getEntry("Velocity Meters Per Second").setDouble(getVelocity());
        elev.getEntry("Goal Meters").setDouble(goal.position);
        elev.getEntry("Goal Velocity").setDouble(goal.velocity);

//        super.updateTelemetry(telemetry, "Extendo");
//        telemetry.addData("Motor Ticks:", m_motor.getCurrentPosition());
//        telemetry.addData("Position Centimeters:", getExtensionMeters());
//        telemetry.addData("Velocity Centimeters Per Second:", getVelocity());
    }

    public void setPosition(double extension) {
        double adjustedExtension = MathUtil.clamp(ElevatorConstants.LOWER_BOUND, extension, ElevatorConstants.UPPER_BOUND);
        goal = new TrapezoidProfile.State(extension, 0);
    }

    public void incrementPosition(double deltaMeters) {
        goal.position += deltaMeters;
    }

    public double getVelocity() {
        return m_leftMotor.getCorrectedVelocity() * ElevatorConstants.CENTIMETERS_PER_TICK;
    }

    public double getExtensionMeters() {
        return m_leftMotor.getCurrentPosition() * ElevatorConstants.CENTIMETERS_PER_TICK;
    }

    private TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getExtensionMeters(), getVelocity());
    }

    private double calculateVoltage(double velocity, double position) {
        return m_feedbackController.calculate(getExtensionMeters(), position) +
                m_feedforwardController.calculate(velocity, 0);
    }

    private void setVoltage(double voltage) {
        m_leftMotor.set(voltage/12);
        m_rightMotor.set(voltage/12);
    }

    public void resetPosition() {
        m_leftMotor.stopAndResetEncoder();
        m_rightMotor.stopAndResetEncoder();
    }

    public boolean isFinished() {
        return Math.abs(goal.position - getState().position) < 5.0;
    }
}
