package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class ExplosiveTele extends LinearOpMode {

    public Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        createRobot();
        initialize();
        while(opModeIsActive()) { looping(); }
    }

    /*
        Populates the robot object.
     */
    private void createRobot() {
        robot = new Robot(hardwareMap);
    }

    /*
        Used to initialize any motors or servos to their proper positions, as well as activate objects such as cameras or other sensors that are not created in the Robot class. The robot will already be created and initialized upon calling.
     */
    protected abstract void initialize();

    /*
        The method called every tick of the TeleOp period after the Start button has been pressed.
     */
    protected abstract void looping();


    public void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
            showError("Could not sleep Thread!");
        }
    }

    /*
        Halt progress on the program and show an error in the telemetry
     */
    public void showError(String error) {
        telemetry.addLine("Error: " + error);
        telemetry.update();
        while(opModeIsActive()) {
            // Wait here
        }
    }

}
