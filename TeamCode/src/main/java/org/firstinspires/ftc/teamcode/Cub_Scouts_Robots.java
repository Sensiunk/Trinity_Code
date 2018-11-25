package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Manjesh on 11/09/2018.
 */
@TeleOp(name = "Cub Scouts Robot")
public class Cub_Scouts_Robots extends LinearOpMode
{
    public DcMotor rightMotor;
    public DcMotor leftMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive())
        {
            //Introduce variables
            double drive;   // Power for forward and back motion
            double rotate;  // Power for rotating the robot

            //Gamepad 1 Portion
            //-------------------------------------------------------------------------
            drive = gamepad1.left_stick_y;  // Negative because the gamepad is weird
            rotate = -gamepad1.right_stick_x;

            //Set the values for the drive to be only -1 <-> 1
            drive = Range.clip(drive, -1, 1);
            rotate = Range.clip(rotate, -1, 1);

            //Set the variables drive component to work with our custom method
            drive = (float) scaleInput(drive);
            rotate = (float) scaleInput(rotate);

            //Set the power for the wheels
            rightMotor.setPower(drive - rotate);
            leftMotor.setPower(drive + rotate);

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