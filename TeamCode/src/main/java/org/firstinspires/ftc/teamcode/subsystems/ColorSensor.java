package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.vision.opencv.ColorRange;

public class ColorSensor extends SubsystemBase {
    private final ColorRange redRange = ColorRange.RED;//new ColorRange(ColorSpace.HSV, new Scalar(276, ));
    private final ColorRange yellowRange = ColorRange.YELLOW;
    private final ColorRange blueRange = ColorRange.BLUE;
    private final NormalizedColorSensor sensor;
    public ColorSensor(HardwareMap hardwareMap, String hardwareName) {
        this(hardwareMap, hardwareName, 2);
    }
    public ColorSensor(HardwareMap hardwareMap, String hardwareName, float gain) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, hardwareName);
        sensor.setGain(gain);
        NormalizedRGBA returnValue = sensor.getNormalizedColors();

    }
    public NormalizedRGBA getColor(){
        return sensor.getNormalizedColors();
    }
    public boolean objectNear() {
        return sensor.getNormalizedColors().alpha > 0.5;
    }
    public boolean isBlue() {
        return false;
    }
}
