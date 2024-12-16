package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ColorSubsystem extends SubsystemBase {
    private final ColorSensor m_colorSensor;
    private final Telemetry m_telemetry;

    public ColorSubsystem(@NonNull HardwareMap hardwareMap, Telemetry telemetry){
        m_colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        m_telemetry = telemetry;
    }

    @Override
    public void periodic() {
        m_telemetry.addData("red Value",m_colorSensor.red());
        m_telemetry.addData("blue Value",m_colorSensor.blue());
        m_telemetry.addData("green Value",m_colorSensor.green());
    }

    public boolean hasCorrectColor() {
        if (OperatorPresets.IsRed == true) {
            return m_colorSensor.red() > m_colorSensor.blue();
                    //||m_ColorSensor.red()+m_ColorSensor.green() > m_ColorSensor.blue()*2;
                    //||m_ColorSensor.green() > m_ColorSensor.red();
        }
        else {
            return m_colorSensor.blue() > m_colorSensor.red()||m_colorSensor.green() > m_colorSensor.blue()*.6+m_colorSensor.red();
            //                    //||m_ColorSensor.green() > m_ColorSensor.red();
        }

        //return m_ColorSensor.blue() > m_ColorSensor.red();
    }

    public boolean sampleDetected() {
        return m_colorSensor.blue() + m_colorSensor.red() + m_colorSensor.green() > 900;
        // Figure this out
    }


    /*
    public static String sampleDetected(int red, int green, int blue) {
        if ((blue + green + red) >= 900) {
            if (blue >= green && blue >= red) {
                return "BLUE";
            } else if (green >= red) {
                return "YELLOW";
            } else {
                return "RED";
            }
        }
        else {
            return "NONE";
        }
    }*/

}