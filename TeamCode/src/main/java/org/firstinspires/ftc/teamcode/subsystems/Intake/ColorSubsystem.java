package org.firstinspires.ftc.teamcode.subsystems.Intake;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ColorSubsystem extends SubsystemBase {
    private final ColorSensor m_ColorSensor;

    public ColorSubsystem(@NonNull HardwareMap hardwareMap){
        m_ColorSensor = hardwareMap.get(ColorSensor.class, "color");

    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("red Value",m_ColorSensor.red());
        telemetry.addData("blue Value",m_ColorSensor.blue());
        telemetry.addData("green Value",m_ColorSensor.green());
    }


    public boolean hasCube() {
        return m_ColorSensor.blue()+m_ColorSensor.red() > 500;
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