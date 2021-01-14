package org.firstinspires.ftc.teamcode.ExplosivesUtils;

import android.graphics.drawable.DrawableContainer;
import android.graphics.drawable.GradientDrawable;
import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.HashMap;

public class Robot {

    public DcMotor fleft, fright, bright, bleft, shootL, shootR, intake, conveyor;

    BNO055IMU imu;

    OpMode opMode;

    HardwareMap hardwareMap;

    public enum Direction {
        LEFT, RIGHT
    }

    public Robot(HardwareMap hardwareMap, OpMode opMode) {
        this.hardwareMap=hardwareMap;
        this.opMode=opMode;
        init();
    }

    /*
        Create all the hardware objects and set them to their respective variables. Called upon initialization of the class.
     */
    private void init() {

        initGyro();

        // Initialize all the variables
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        bright = hardwareMap.get(DcMotor.class, "bright");

        shootL = hardwareMap.get(DcMotor.class, "shootL");
        shootR = hardwareMap.get(DcMotor.class, "shootR");

        intake = hardwareMap.get(DcMotor.class, "intake");

        conveyor = hardwareMap.get(DcMotor.class, "conveyor");

        bright.setDirection(DcMotorSimple.Direction.REVERSE);
//        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /*
        Returns an ArrayList of SensorData objects that should represent every used sensor on the robot.
     */
    public ArrayList<SensorData> getSensorData() {
        ArrayList<SensorData> list = new ArrayList<>();
        list.add(new SensorData("gyro heading", getHeading()));
        list.add(new SensorData("fleft encoder", fleft.getCurrentPosition()));
        list.add(new SensorData("fright encoder", fright.getCurrentPosition()));
        list.add(new SensorData("bleft encoder", bleft.getCurrentPosition()));
        list.add(new SensorData("bright encoder", bright.getCurrentPosition()));
        return list;
    }

    public void shoot(double speed) {
        shootL.setPower(speed);
        shootR.setPower(speed);
    }

    public void intake() {
        intake.setPower(0.7);
    }

    public void outtake() {
        intake.setPower(-0.7);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void stop() {
        drive(0);
        shoot(0);
        stopIntake();
    }

    public void conveyor(double speed) {
        conveyor.setPower(speed);
    }

    public void drive(double speed) {

        speed = -speed;

        if(speed < -1 || speed > 1) { return; }

        fleft.setPower(speed);
        fright.setPower(speed);
        bleft.setPower(speed);
        bright.setPower(speed);

    }

    public void turn(double speed, Direction direction) {
        speed = Math.abs(speed);
        if(direction == Direction.LEFT) {
            rightSide(speed);
            leftSide(-speed);
        } else {
            rightSide(-speed);
            leftSide(speed);
        }
    }

    public void turn(double speed) {
        rightSide(-speed);
        leftSide(speed);
    }

    public void turn(int degrees, double speed) {

        long startTime = System.currentTimeMillis();

        double initialAngle = getHeading();
        double targetAngle = getHeading()+degrees;

        // If the degrees is positive, sign is 1; else -1
        int sign = degrees>0 ? 1 : -1;

        double diff = getHeading()-targetAngle;
        double calcSpeed = speed;

        while(diff >= 2 || System.currentTimeMillis()-startTime>TURN_TIMEOUT) {

            // Calculate speed that each side should go at
            if(diff>25) {
                calcSpeed=speed;
            } else if (diff>15) {
                calcSpeed=speed*0.6;
            } else if (diff>5) {
                calcSpeed=speed*0.4;
            } else {
                calcSpeed=speed*0.2;
            }

            if(calcSpeed<=MIN_TURN_SPEED) {
                calcSpeed=MIN_TURN_SPEED;
            }

            leftSide(sign*calcSpeed);
            rightSide(-sign*calcSpeed);

            diff = getHeading()-targetAngle;
        }

    }


    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()) {
            //Wait
        }

    }

    public void leftSide(double speed) {
        bleft.setPower(speed);
        fleft.setPower(speed);
    }

    public void rightSide(double speed) {
        bright.setPower(speed);
        fright.setPower(speed);
    }

    public Orientation getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getHeading() {
        return getAngle().firstAngle;
    }

    public void driveStraight(double speed, int ticks) {
        resetEncoders();
        long startTime = System.currentTimeMillis();
        double leftMult = 1;
        double rightMult = 1;
        while(System.currentTimeMillis()-startTime<=15000) {

            // If the difference between the two is less than 10 ticks, have them both move at the same speed
            if(Math.abs(leftEncoder()-rightEncoder())<10) {
                leftMult=1;
                rightMult=1;
            } else {

                double avg = (leftEncoder()+rightEncoder())/2;

                double leftDiff = Math.abs(avg-leftEncoder());
                double rightDiff = Math.abs(avg-rightEncoder());

                if(leftEncoder()>rightEncoder()) {
                    rightMult=1.1;
                    leftMult=0.9;
                } else {
                    leftMult=1.1;
                    rightMult=0.9;
                }

            }

            if(Math.abs(ticks-leftEncoder())<10) {
                leftMult=0;
            }

            if(Math.abs(ticks-rightEncoder())<10) {
                rightMult=0;
            }

            leftSide(speed*leftMult);
            rightSide(speed*rightMult);

            if(leftMult==0 && rightMult==0) {
                return;
            }

        }
    }

    public void strafe(double speed, Direction direction) {
        if(direction == Direction.LEFT) {
            fright.setPower(-speed);
            bright.setPower(speed);
            fleft.setPower(speed);
            bleft.setPower(-speed);
        } else {
            fright.setPower(speed);
            bright.setPower(-speed);
            fleft.setPower(-speed);
            bleft.setPower(speed);
        }
    }

    public void resetEncoders() {
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double leftEncoder() {
        return fleft.getCurrentPosition();
//        return (bleft.getCurrentPosition()+fleft.getCurrentPosition())/2;
    }

    public double rightEncoder() {
        return fright.getCurrentPosition();
//        return (bright.getCurrentPosition()+fright.getCurrentPosition())/2;
    }

    final double MIN_TURN_SPEED = 0.2;
    final int TURN_TIMEOUT = 4000;
    final static double MAX_TURN_DIFF = 2;

    public void turnToAngle(double angle) {
        long startTime = System.currentTimeMillis();

        double initial = stripAngle(getHeading());
        double target = stripAngle(angle);

        double diff = target-initial;

        opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target),null);
        opMode.telemetry.update();

        double speed = 0.0;

        while(System.currentTimeMillis()<startTime+TURN_TIMEOUT &&  Math.abs(diff)>MAX_TURN_DIFF) {
            opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target),null);
            opMode.telemetry.update();
            speed = 0.8*motorPowerFunction(diff);
            turn(speed);
            diff = target-stripAngle(getHeading());
        }

        turn(0);

        waitMillis(100);

        diff = target-stripAngle(getHeading());

        while(System.currentTimeMillis()<startTime+TURN_TIMEOUT/2 && Math.abs(diff)>MAX_TURN_DIFF/2) {
            opMode.telemetry.addData(("Head: " + getHeading() + " Diff: " + diff + " Target: " + target),null);
            opMode.telemetry.update();
            speed = 0.6*motorPowerFunction(diff);
            turn(speed);
            diff = target-stripAngle(getHeading());
        }

        turn(0);

    }

    // Custom function found on Desmos to control the power of the motors based off of the given angle
    public double motorPowerFunction(double angle) {
        if(angle>0) {
            return Math.sqrt(angle/600)+0.2;
        } else {
            return -Math.sqrt(-angle/600)-0.2;
        }
    }

    // Manipulates a given angle so that it is between 0º and 360º
    public double stripAngle(double angle) {
        if(angle>360) {
            angle-=Math.floor(angle/360)*360;
        } else if (angle<-360) {
            angle+=(Math.floor(angle/360)*-360)-360;
        }
        return angle;
    }

    public void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }

}
