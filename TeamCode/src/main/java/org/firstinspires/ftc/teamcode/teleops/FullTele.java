package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.Robot;

@TeleOp(name = "FullTele")
public class FullTele extends ExplosiveTele {

    @Override
    protected void initialize() {

    }

    boolean locked = false;

    final double LEFT_PSHOT=-10;
    final double CENTER_PSHOT=0;
    final double RIGHT_PSHOT=-10;

    @Override
    protected void looping() {

        if(Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {
            robot.fright.setPower(0.6*((gamepad1.left_stick_y+gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.bright.setPower(0.6*((gamepad1.left_stick_y-gamepad1.left_stick_x) + gamepad1.right_stick_x));
            robot.fleft.setPower(0.6*((gamepad1.left_stick_y-gamepad1.left_stick_x) - gamepad1.right_stick_x));
            robot.bleft.setPower(0.6*((gamepad1.left_stick_y+gamepad1.left_stick_x) - gamepad1.right_stick_x));
        } else {
            robot.stop();
        }

        if(Math.abs(gamepad2.right_trigger)>0.2) {
            robot.shootL.setVelocity(-1950);
            robot.shootR.setVelocity(-1850);
            telemetry.addLine("Lvelocity: " + robot.shootL.getVelocity());
            telemetry.addLine("Rvelocity: " + robot.shootR.getVelocity());
            telemetry.update();
        } else if(Math.abs(gamepad2.left_trigger)>0.2) {
            robot.shootL.setVelocity(2000);
            robot.shootR.setVelocity(2000);
            telemetry.addLine("Lvelocity: " + robot.shootL.getVelocity());
            telemetry.addLine("Rvelocity: " + robot.shootR.getVelocity());
            telemetry.update();
        } else {
//            robot.shootL.setVelocity(-robot.shootL.getVelocity()/8);
//            robot.shootR.setVelocity(-robot.shootR.getVelocity()/8);
            robot.shootL.setVelocity(0);
            robot.shootR.setVelocity(0);
        }

        if(gamepad2.a) {
            robot.intake();
        } else if (gamepad2.y) {
            robot.outtake();
        } else {
            robot.stopIntake();
        }

        if(gamepad2.dpad_up) {
            robot.conveyor(-1);
        } else if(gamepad2.dpad_down) {
            robot.conveyor(1);
        } else {
            robot.conveyor(0);
        }

        if(gamepad1.y && !locked) {
            locked=true;
            robot.turnToAngle(CENTER_PSHOT);
        } else {
            locked=false;
        }

        if(gamepad1.b && !locked) {
            locked=true;
            robot.turnToAngle(RIGHT_PSHOT);
        } else {
            locked=false;
        }

        if(gamepad1.x && !locked) {
            locked=true;
            robot.turnToAngle(LEFT_PSHOT);
        } else {
            locked=false;
        }

        if(gamepad1.a && !locked) {
            locked=true;
            robot.strafe(0.5, Robot.Direction.LEFT);
            waitMillis(500);
            robot.strafe(0.6, Robot.Direction.RIGHT);
            waitMillis(750);
            robot.drive(-0.5);
            waitMillis(150);
            robot.turnToAngle(0);
        } else {
            locked=false;
        }

        if(gamepad2.x) {
            robot.closeGrabber();
        } else if(gamepad2.b && robot.wobbler.getPosition()>0.6) {
            robot.openGrabber();
        }

//        if(gamepad2.dpad_left) {
//            robot.dropWobbler();
//        } else if(gamepad2.dpad_right) {
//            robot.liftWobbler();
//        }

        if(gamepad2.left_bumper && !wobblerToggleLock) {
            wobblerToggleLock=true;
            if(wobblerUp) {
                robot.dropWobbler();
                wobblerUp=false;
            } else {
                wobblerUp=true;
                robot.liftWobbler();
            }
        } else if (!gamepad2.left_bumper) {
            wobblerToggleLock=false;
        }

        if(gamepad2.right_bumper && !wobblerLock) {
            wobblerLock=true;
            robot.stop();
            robot.satisfyingWobblerDrop();
        } else {
            wobblerLock=false;
        }

    }

    public boolean wobblerUp=true;
    public boolean wobblerLock=false;
    public boolean wobblerToggleLock=false;

}
