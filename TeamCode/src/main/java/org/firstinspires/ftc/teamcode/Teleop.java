package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Manjesh on 12/4/2018.
 */
@TeleOp(name = "Trinity Competition Tele-Op")
public class Teleop extends LinearOpMode
{
    //Motor Count -- 7 / 8
    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;
    public DcMotor leftArmMotor;
    public DcMotor rightArmMotor;
    public DcMotor liftMotor;

    //Servo Count -- 7 / 12
    public CRServo sweepServo;
    public CRServo rightExtendServo;
    public CRServo leftExtendServo;
    public CRServo rightRotateServo;
    public CRServo leftRotateServo;
    public Servo flickServo;
    public Servo liftpushServo;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        leftArmMotor = hardwareMap.dcMotor.get("leftArmMotor");
        rightArmMotor = hardwareMap.dcMotor.get("rightArmMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        sweepServo = hardwareMap.crservo.get("sweepServo");
        rightExtendServo = hardwareMap.crservo.get("rightExtendServo");
        leftExtendServo = hardwareMap.crservo.get("leftExtendServo");
        rightRotateServo = hardwareMap.crservo.get("rightExtendServo");
        leftRotateServo = hardwareMap.crservo.get("leftExtendServo");
        flickServo = hardwareMap.servo.get("flickServo");
        liftpushServo = hardwareMap.servo.get("liftpushServo");
        leftMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        while (opModeIsActive())
        {
            //Introduce variables
            double drive;   // Power for forward and back motion
            double strafe;  // Power for left and right motion
            double rotate;  // Power for rotating the robot
            double lift;
            double arm;

            //Gamepad 1 Portion
            //-------------------------------------------------------------------------
            drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
            strafe = -gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            //Set the values for the drive to be only -1 <-> 1
            drive = Range.clip(drive, -1, 1);
            strafe = Range.clip(strafe, -1, 1);
            rotate = Range.clip(rotate, -1, 1);

            //Set the variables drive component to work with our custom method
            drive = (float) scaleInput(drive);
            strafe = (float) scaleInput(strafe);
            rotate = (float) scaleInput(rotate);

            if (gamepad1.left_trigger > 0.25)
            {
                drive /= 3;
                strafe /= 3;
                rotate /= 3;
            }

            if (gamepad1.x)
            {
                drive = 0;
                strafe = 0;
                rotate = 0;
            }

            if (gamepad1.a)
            {
                liftpushServo.setPosition(Servo.MIN_POSITION);
            }

            if (gamepad1.b)
            {
                liftpushServo.setPosition(Servo.MAX_POSITION);
            }

            if (gamepad1.dpad_up){
                extendServo.setPower(1);
            }
            else if (gamepad1.dpad_down)
            {
                extendServo.setPower(-1);
            }
            else if (gamepad1.dpad_right){
                extendServo.setPower(0);
            }

            //Set the power for the wheels
            leftMotorBack.setPower(drive - strafe + rotate);
            leftMotorFront.setPower(drive + strafe + rotate);
            rightMotorBack.setPower(drive + strafe - rotate);
            rightMotorFront.setPower(drive - strafe - rotate);

            //Gamepad 2 Portion
            //-------------------------------------------------------------------------

            lift = gamepad2.right_stick_y;

            lift = Range.clip(lift, -1, 1);

            lift = (float) scaleInput(lift);

            liftMotor.setPower(lift);


            arm = gamepad2.left_stick_y;

            arm = Range.clip(arm, -1,1);

            arm = (float) scaleInput(arm);

            if (gamepad2.right_trigger > 0.25)
            {
                arm /= 3;
            }

            if (gamepad2.left_trigger > 0.25)
            {
                arm /= 2;
            }

            armMotor.setPower(arm);

            if (gamepad2.a)
            {
                collectServo.setPosition(Servo.MIN_POSITION);
            }

            if (gamepad2.b)
            {
                collectServo.setPosition(Servo.MAX_POSITION);
            }

            //Fail safe actions
            //-------------------------------------------------------------------------
            if (gamepad2.x)
            {
                flickServo.setPosition(Servo.MAX_POSITION);
            }
            idle();
        }
    }

    double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
}