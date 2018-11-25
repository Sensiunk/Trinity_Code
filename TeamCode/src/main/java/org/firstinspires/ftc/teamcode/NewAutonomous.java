package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="RoverDEPOT", group="Linear Opmode")
public class NewAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Declare all variables
    final float CIRCUMFERENCE = (float)(4.00 * Math.PI);
    final int ENCODERTICKS = 1680;
    final double GEARRATIO = 0.6;
    final double COUNTS_PER_INCH = (ENCODERTICKS * GEARRATIO) / (CIRCUMFERENCE);
    final double DRIVE_SPEED             = 0.9;     // Nominal speed for better accuracy.
    final double TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.
    final double HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    final double P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    final double P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //Declare all motors
    DcMotor rightMotorFront;
    DcMotor leftMotorFront;
    DcMotor rightMotorBack;
    DcMotor leftMotorBack;
    DcMotor armMotor;
    DcMotor liftMotor;
    Servo flickServo;
    BNO055IMU imu;
    Orientation angles;

    private GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Looking for the names of the motors in the config
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        flickServo = hardwareMap.servo.get("flickServo");
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");
        telemetry.addData("Gyro Calibration Status", imu.getCalibrationStatus().toString());
        telemetry.update();

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Good Luck Drivers");

            detector = new GoldAlignDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();

            // Optional Tuning
            //detector.alignsize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
            detector.maxAreaScorer.weight = 0.005;

            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0;

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(4000);
            liftMotor.setPower(1);
            sleep(5000);
            liftMotor.setPower(-4000);
            liftMotor.setPower(1);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            gyroDrive(DRIVE_SPEED, 5, 0);

            detector.enable();
            //Left
            if (detector.getXPosition() > 390) {
                strafe(DRIVE_SPEED, 40, 0);
                gyroDrive(DRIVE_SPEED, 5, 0);
                gyroDrive(DRIVE_SPEED, -5, 0);
                strafe(DRIVE_SPEED, -40, 0);
            }
            //Center
            else if (detector.getXPosition() < 390 && detector.getXPosition() > 100) {
                gyroDrive(DRIVE_SPEED, 5, 0);
                gyroDrive(DRIVE_SPEED, -5, 0);
            }
            //Right
            else if (detector.getXPosition() < 99) {
                strafe(DRIVE_SPEED, -40, 0);
                gyroDrive(DRIVE_SPEED,5, 0);
                gyroDrive(DRIVE_SPEED, -5, 0);
                strafe(DRIVE_SPEED, 40, 0);
            }
            detector.disable();

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setTargetPosition(-400);
            armMotor.setPower(1);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sleep(3000);
            flickServo.setPosition(Servo.MAX_POSITION);
            gyroDrive(DRIVE_SPEED, 20.0, 0.0);    // Drive FWD 15 inches
            gyroTurn(TURN_SPEED, -90.0);     // Turn  CCW to 90 Degrees
            gyroHold(TURN_SPEED, -90.0, 0.5);    // Hold -90 Deg heading for a 1/2 second
            gyroDrive(DRIVE_SPEED, -35.0, -90.0);    // Drive FWD 41 inches with heading -85 Deg
            gyroTurn(TURN_SPEED, -45.0);      // Turn CW to Origin angle
            gyroHold(TURN_SPEED, -45.0, 0.5);     // Hold 0 Deg heading for a 1/2 second
            gyroDrive(DRIVE_SPEED, -56.0, -45.0);
            sleep(1000);
            flickServo.setPosition(Servo.MIN_POSITION);
            sleep(1000);
            flickServo.setPosition(Servo.MAX_POSITION);
            sleep(1000);
            flickServo.setPosition(Servo.MIN_POSITION);
            sleep(1000);
            flickServo.setPosition(Servo.MAX_POSITION);
            sleep(1000);
            gyroDrive(DRIVE_SPEED, 63.0, -45.0);
            gyroTurn(TURN_SPEED, -10.0);
            gyroHold(TURN_SPEED, -10.0, 0.5);
            gyroDrive(DRIVE_SPEED, 45.0, -10.0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
    public  void strafe (double speed, double distance, double angle){

        int     newLeftTargetFront;
        int     newLeftTargetBack;
        int     newRightTargetFront;
        int     newRightTargetBack;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTargetFront = leftMotorFront.getCurrentPosition() + moveCounts;
            newLeftTargetBack = leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTargetFront = rightMotorFront.getCurrentPosition() + moveCounts;
            newRightTargetBack = rightMotorBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            rightMotorFront.setTargetPosition(newRightTargetFront);
            rightMotorBack.setTargetPosition(newRightTargetBack);
            leftMotorFront.setTargetPosition(newLeftTargetFront);
            leftMotorBack.setTargetPosition(newLeftTargetBack);

            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            rightMotorFront.setPower(speed);
            rightMotorBack.setPower(speed);
            leftMotorFront.setPower(speed);
            leftMotorBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (rightMotorFront.isBusy() && rightMotorBack.isBusy() && leftMotorFront.isBusy() && leftMotorBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotorFront.setPower(leftSpeed);
                leftMotorBack.setPower(leftSpeed);
                rightMotorFront.setPower(rightSpeed);
                rightMotorBack.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newRightTargetFront, newRightTargetBack, newLeftTargetFront, newLeftTargetBack);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotorFront.getCurrentPosition(), leftMotorBack.getCurrentPosition(), rightMotorFront.getCurrentPosition(), rightMotorBack.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftMotorFront.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorFront.setPower(0);
            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroDrive ( double speed, double distance, double angle) {

        int     newLeftTargetFront;
        int     newLeftTargetBack;
        int     newRightTargetFront;
        int     newRightTargetBack;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTargetFront = leftMotorFront.getCurrentPosition() + moveCounts;
            newLeftTargetBack = leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTargetFront = rightMotorFront.getCurrentPosition() + moveCounts;
            newRightTargetBack = rightMotorBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            rightMotorFront.setTargetPosition(newRightTargetFront);
            rightMotorBack.setTargetPosition(newRightTargetBack);
            leftMotorFront.setTargetPosition(newLeftTargetFront);
            leftMotorBack.setTargetPosition(newLeftTargetBack);

            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            rightMotorFront.setPower(speed);
            rightMotorBack.setPower(speed);
            leftMotorFront.setPower(speed);
            leftMotorBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (rightMotorFront.isBusy() && rightMotorBack.isBusy() && leftMotorFront.isBusy() && leftMotorBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotorFront.setPower(leftSpeed);
                leftMotorBack.setPower(leftSpeed);
                rightMotorFront.setPower(rightSpeed);
                rightMotorBack.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newRightTargetFront, newRightTargetBack, newLeftTargetFront, newLeftTargetBack);
                telemetry.addData("Actual",  "%7d:%7d",      leftMotorFront.getCurrentPosition(), leftMotorBack.getCurrentPosition(), rightMotorFront.getCurrentPosition(), rightMotorBack.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftMotorFront.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorFront.setPower(0);
            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorFront.setPower(0);
        rightMotorBack.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftMotorFront.setPower(leftSpeed);
        leftMotorBack.setPower(leftSpeed);
        rightMotorFront.setPower(rightSpeed);
        rightMotorBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}