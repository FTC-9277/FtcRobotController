package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveAuto;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.Sampler;

@Autonomous(name = "PSAuto")
public class PowerShotAuto extends ExplosiveAuto {

    Camera camera;
    Sampler sampler;

    @Override
    protected void initialize() {
        log("Starting.");
        camera = new Camera(hardwareMap,false);
        sampler = new Sampler(camera);
        log("Initialized!");
    }

    @Override
    protected void begin() throws InterruptedException {

        log("cycling");
        camera.cycle();
        log("end");
        log("sampling");
        double[] value = sampler.sample();
        log(("Sample: " + value[0] + ", TotalW: " + value[1] + ", HeightUp: " + value[2] + ", HeightDown: " + value[3] + ", W: " + value[4] + ", H: " + value[5] + ", center: (" + value[6] + "," + value[7] + ")"));

        robot.driveForwardEncoders(0.35,1500);
        robot.turnToPS(Robot.PowerShotPos.RIGHT);

        waitMillis(250);

        robot.shoot(Robot.ShooterSpeed.AUTO_PS);
        waitMillis(1250);
        robot.conveyor(-1.0);
        waitMillis(350);
        robot.conveyor(0.0);

        robot.turnToPS(Robot.PowerShotPos.MID);
        robot.conveyor(-1.0);
        waitMillis(550);
        robot.conveyor(0.0);

        robot.turnToPS(Robot.PowerShotPos.LEFT);
        robot.intake();
        robot.conveyor(-1.0);
        waitMillis(1200);
        robot.conveyor(0.0);

        waitMillis(250);
        robot.stopIntake();
        robot.stopShooter();

        robot.turnToAngle(0);

        //Now on depends on vision

        if(value[0] == 0) {
            robot.driveForwardEncoders(0.3,600);
            robot.strafeEncoders(0.3, 500, Robot.Direction.LEFT);
            robot.turnToAngle(90);
            robot.dropWobbler();
            waitMillis(1250);

            robot.openGrabber();
            waitMillis(250);

            robot.liftWobbler();

            robot.turnToAngle(0);

            robot.dropWobbler();

            robot.strafeEncoders(1.0,350, Robot.Direction.LEFT);

            robot.driveBackwardEncoders(0.5,1050);

            robot.closeGrabber();
            waitMillis(500);
            robot.liftWobbler();

            waitMillis(500);

            robot.driveForwardEncoders(0.7,750);

            robot.turnToAngle(90);

            robot.driveBackwardEncoders(1.0,500);

            robot.satisfyingWobblerDrop();


        } else if (value[0]==1) {
            robot.driveForwardEncoders(0.3,750);
            robot.strafeEncoders(0.3, 400, Robot.Direction.LEFT);
            robot.turnToAngle(180);
            robot.dropWobbler();
            waitMillis(1250);

            robot.openGrabber();
            waitMillis(250);

            robot.liftWobbler();

            robot.turnToAngle(0);

            robot.closeGrabber();

            robot.intake();
            robot.driveBackwardEncoders(0.5,1500);

            robot.conveyor(-1.0);
            robot.shoot(Robot.ShooterSpeed.AUTO_PS);

            robot.driveForwardEncoders(0.6,1000);
            robot.turnToAngle(0);

            waitMillis(1500);

            robot.stopShooter();
            robot.stopIntake();
            robot.conveyor(0.0);

            robot.driveForwardEncoders(1.0,250);
        } else if (value[0] == 4) {
            robot.driveForwardEncoders(1.0,1600);
            robot.strafeEncoders(0.5, 550, Robot.Direction.LEFT);
            robot.turnToAngle(90);
            robot.dropWobbler();
            waitMillis(1250);

            robot.openGrabber();
            waitMillis(250);

            robot.liftWobbler();

            robot.turnToAngle(0);

            robot.closeGrabber();

            robot.intake();
            robot.driveBackwardEncoders(0.5,2750);

            robot.conveyor(-1.0);
            robot.shoot(Robot.ShooterSpeed.AUTO_TOWERGOAL);

            robot.driveForwardEncoders(0.6,750);
            robot.turnToAngle(0);

            waitMillis(1500);

            robot.stopShooter();
            robot.stopIntake();
            robot.conveyor(0.0);

            robot.driveForwardEncoders(1.0,250);
        }

    }

    public void shootPS() {
    }
}
