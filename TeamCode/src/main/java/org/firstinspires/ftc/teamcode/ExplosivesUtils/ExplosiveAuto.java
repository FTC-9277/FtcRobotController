package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ExplosiveAuto extends LinearOpMode {

    public Robot robot;

    long startTime = System.currentTimeMillis();

    // The number of milliseconds the program must run for at least before ending
    final int MINIMUM_RUNNING_MILLIS = 5000;

    @Override
    public void runOpMode() throws InterruptedException {
        createRobot();
        initialize();
        waitForStart();
        begin();

        // Wait until program has run for the minimum millis until exiting
        while(!hasRunForMinimumTime()) {}
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
        The method called once the Start button has been pressed and the 30 second Autonomous period has begun.
     */
    protected abstract void begin();


    public boolean hasRunForMinimumTime() {
        return System.currentTimeMillis() >= startTime+MINIMUM_RUNNING_MILLIS;
    }


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
