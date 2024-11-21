package org.firstinspires.ftc.teamcode.subsystems.Intake;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;


public class ColorSubsystem extends SubsystemBase {
    private final ColorSensor m_ColorSensor;

    public ColorSubsystem(@NonNull HardwareMap hardwareMap){
        m_ColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("red Value",m_ColorSensor.red());
        telemetry.addData("blue Value",m_ColorSensor.blue());
        telemetry.addData("green Value",m_ColorSensor.green());
    }


    public boolean hasCorrectColor() {
        //I might need to make = into ==
        if (OperatorPresets.IsRed = true) {
            return m_ColorSensor.red() > m_ColorSensor.blue();
                    //||m_ColorSensor.red()+m_ColorSensor.green() > m_ColorSensor.blue()*2;
                    //||m_ColorSensor.green() > m_ColorSensor.red();
        }
        else {
            return m_ColorSensor.blue() > m_ColorSensor.red();
                    //||m_ColorSensor.red()+m_ColorSensor.green() > m_ColorSensor.blue()*2;2;
            //                    //||m_ColorSensor.green() > m_ColorSensor.red();
        }

        //return m_ColorSensor.blue() > m_ColorSensor.red();
    }

    public boolean sampleDetected() {
        return m_ColorSensor.blue() + m_ColorSensor.red() + m_ColorSensor.green() > 900;
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