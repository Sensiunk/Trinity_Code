package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Manjesh on 2/21/2018.
 */
//@TeleOp(name = "Trinity First Competition Tele-Op")
public class Teleop extends LinearOpMode
{
    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;
    public DcMotor liftMotor;
    public DcMotor armMotor;
    public Servo flickServo;
    public Servo extendServo;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        extendServo = hardwareMap.servo.get("extendServo");
        flickServo = hardwareMap.servo.get("flickServo");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive())
        {
            //Introduce variables
            double drive;   // Power for forward and back motion
            double strafe;  // Power for left and right motion
            double rotate;  // Power for rotating the robot
            double arm;

            //Gamepad 1 Portion
            //-------------------------------------------------------------------------
            drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
            strafe = -gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

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
            if (gamepad1.right_trigger > 0.25)
            {
                drive = 0;
                strafe = 0;
                rotate = 0;
            }

            //Set the power for the wheels
            leftMotorBack.setPower(drive - strafe + rotate);
            leftMotorFront.setPower(drive + strafe + rotate);
            rightMotorBack.setPower(drive + strafe - rotate);
            rightMotorFront.setPower(drive - strafe - rotate);

            //Gamepad 2 Portion
            //-------------------------------------------------------------------------
            arm = -gamepad2.left_stick_y;

            arm = Range.clip(arm, -1, 1);

            arm = (float) scaleInput(arm);

//            liftMotor.setPower(arm);
//
//            if (gamepad2.x)
//            {
//                flickServo.setPosition(Servo.MIN_POSITION);
//            }
//
//            if (gamepad2.a)
//            {
//                //Block gripper opens
//                rightServo.setPosition(Servo.MAX_POSITION);
//                leftServo.setPosition(Servo.MIN_POSITION);
//            }
//            if (gamepad2.b)
//            {
//                //Block gripper closes
//                rightServo.setPosition(Servo.MIN_POSITION);
//                leftServo.setPosition(Servo.MAX_POSITION);
//            }


            //Fail safe actions
            //-------------------------------------------------------------------------

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