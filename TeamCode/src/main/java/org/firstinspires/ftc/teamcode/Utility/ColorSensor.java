package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensor {




    public enum DetectedColor {
        RED,
        BLUE,
        UNKNOWN;
    }

    public float normRed;
    public float normGreen;
    public float normBlue;



    public DetectedColor getDetectedColor(NormalizedColorSensor colorSensor){
        NormalizedRGBA colors =colorSensor.getNormalizedColors();
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        if (normRed > normBlue){
            return DetectedColor.RED;
        }
        else return DetectedColor.BLUE;

    }


}
