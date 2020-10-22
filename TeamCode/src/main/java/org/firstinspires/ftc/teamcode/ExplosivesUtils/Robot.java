package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashMap;

public class Robot {

    DcMotor fleft, fright, bright, bleft;

    HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap=hardwareMap;
        init();
    }

    /*
        Create all the hardware objects and set them to their respective variables. Called upon initialization of the class.
     */
    private void init() {
        // Initialize all the variables
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        bright = hardwareMap.get(DcMotor.class, "bright");

    }

    /*
        Returns an ArrayList of SensorData objects that should represent every used sensor on the robot.
     */
    public ArrayList<SensorData> getSensorData() {
        ArrayList<SensorData> list = new ArrayList<>();
        list.add(new SensorData("fleft encoder", fleft.getCurrentPosition()));
        list.add(new SensorData("fright encoder", fright.getCurrentPosition()));
        list.add(new SensorData("bleft encoder", bleft.getCurrentPosition()));
        list.add(new SensorData("bright encoder", bright.getCurrentPosition()));
        return list;
    }

}
