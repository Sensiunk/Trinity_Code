package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Manjesh on 2/21/2018.
 */
@TeleOp(name = "Lift Test")
public class LiftTest extends LinearOpMode
{
    public DcMotor lift;

    @Override
    public void runOpMode() throws InterruptedException
    {
        lift = hardwareMap.dcMotor.get("lift");

        waitForStart();
        while (opModeIsActive())
        {
            //Introduce variables
            double drive;   // Power for forward and back motion

            //Gamepad 1 Portion
            //-------------------------------------------------------------------------
            drive = -gamepad1.left_stick_y;
            //Set the values for the drive to be only -1 <-> 1
            drive = Range.clip(drive, -1, 1);
            //Set the variables drive component to work with our custom method
            drive = (float) scaleInput(drive);

            if (gamepad1.left_trigger > 0.25)
            {
                drive /= 3;
            }
            if (gamepad1.right_trigger > 0.25)
            {
                drive /= 2;
            }
            //Streaks

            lift.setPower(drive);



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