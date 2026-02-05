package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class driveMech {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(-(backLeftPower)); // moves in wrong direction
        backRightMotor.setPower(backRightPower);
    }

    public double getPowerFrontLeft() {
        return frontLeftMotor.getPower();
    }
    public double getPowerFrontRight() {
        return frontRightMotor.getPower();
    }
    public double getPowerBackLeft() {
        return backLeftMotor.getPower();
    }
    public double getPowerBackRight() {
        return backRightMotor.getPower();
    }

    public void drive(double theta, double power, double turn) {

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeft = (power * sin/max - turn) / .9;
        double frontRight = (power * cos/max + turn) / .9;
        double backLeft = (power * cos/max - turn) / .9;
        double backRight = (power * sin/max + turn) / .9;

        if((power + Math.abs(turn)) > 1) {
            frontLeft /= power + turn;
            frontRight /= power + turn;
            backLeft /= power + turn;
            backRight /= power + turn;
        }
        setPower(frontLeft, frontRight, backLeft, backRight);
    }
}
