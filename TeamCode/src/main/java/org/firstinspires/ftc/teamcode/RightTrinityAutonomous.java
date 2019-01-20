package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Manjesh on 12/4/2018.
 */
@Autonomous(name="Right Side Trinity Competition Autonomous",group = "British Columbia Competition")
public class RightTrinityAutonomous extends LinearOpMode {

    //Declare all variables
    final float CIRCUMFERENCE = (float)(3.93701 * Math.PI);
    final int ENCODERTICKS = 1680;
    final double GEARRATIO = 0.67;
    final double COUNTS_PER_INCH = (ENCODERTICKS * GEARRATIO) / (CIRCUMFERENCE);
    final double DRIVE_SPEED             = 1.0;     // Nominal speed for better accuracy.
    final double TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.
    final double HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    final double P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    final double P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    final int value = 19500;



    //Declare all motors
    public DcMotor rightMotorFront;
    public DcMotor leftMotorFront;
    public DcMotor rightMotorBack;
    public DcMotor leftMotorBack;
    public DcMotor liftMotor;
    public DcMotor armMotor;
    public Servo flickServo;
    public CRServo extendServo;
    public Servo liftpushServo;
    public Servo collectServo;
    public ModernRoboticsI2cGyro gyro;
    private GoldAlignDetector detector;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    WebcamName webcamName;
    Dogeforia vuforia;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        flickServo = hardwareMap.servo.get("flickServo");
        extendServo = hardwareMap.crservo.get("extendServo");
        liftpushServo = hardwareMap.servo.get("liftpushServo");
        collectServo = hardwareMap.servo.get("collectServo");
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.get("gyro");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            while (!isStarted()) {
                telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                telemetry.update();
            }

            telemetry.addData("Status", "Good Luck Drivers");

            // Set up parameters for Vuforia
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            // Vuforia licence key
            parameters.vuforiaLicenseKey = "AX40NPb/////AAABmWzlHnKJi0nuq3wajaHxlcCMF0ZLxRs4AEtkt1PP4v6QjOwoH4CxiTATq674CdaLumg2ZnH7395KGAYmbp24PKYaSQ+plebKP7GiXVMaNWYplDCTKJEIirTBOqpX3C2pVRhFwzGmDs2MtDlNMa1hxHZ8jStPwHRFR/nCkLQ96QGn3BLfDvZtT5Dbsdh5/Tsegpsyky++xnpC8zIhPNgdO1kDl+mq/JXbbaeJFkQpD5Tokcwlg+GeN0tTdgALRkAbBzx13LWm8spVQFIN5eB/U+lxyPGFykMQXcS4UejkN1uooE8FvA6qFQMaddFqvAMEgY4Q43EV/+PjTaNqep/4ZnPAr3ySSAewf3ClIzoAnzgK";
            parameters.fillCameraMonitorViewParent = true;

            // Set camera name for Vuforia config
            parameters.cameraName = webcamName;

            // Create Dogeforia object
            vuforia = new Dogeforia(parameters);
            vuforia.enableConvertFrameToBitmap();

            //Setup trackables
            VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
            // For convenience, gather together all the trackable objects in one easily-iterable collection */

            final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
            final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
            final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

            for (VuforiaTrackable trackable : allTrackables)
            {
                ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            }

            // Activate the targets
            targetsRoverRuckus.activate();

            // Initialize the detector
            detector = new GoldAlignDetector();
            detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
            detector.useDefaults();
            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
            detector.downscale = 0.8;

            // Set the detector
            vuforia.setDogeCVDetector(detector);
            vuforia.enableDogeCV();
            vuforia.showDebug();
            vuforia.start();
            sleep(1500);

            gyro.resetZAxisIntegrator();

            //Center
            if (detector.getXPosition() > 180 && detector.getXPosition() < 420)
            {
                vuforia.stop();
                Lift(DRIVE_SPEED, value);
                gyroTurn(DRIVE_SPEED, 45);
                gyroHold(DRIVE_SPEED, 45, 0.5);
                gyroDrive(DRIVE_SPEED, 3, 45);
                gyroTurn(DRIVE_SPEED, 0);
                gyroHold(DRIVE_SPEED, 0, 0.5);
                gyroDrive(DRIVE_SPEED, 15, 0);
                gyroDrive(DRIVE_SPEED, 10, 0);
                gyroDrive(DRIVE_SPEED, -10, 0);
                gyroTurn(DRIVE_SPEED, 90);
                gyroHold(DRIVE_SPEED, 90, 0.5);
                gyroDrive(DRIVE_SPEED, 54, 90);
                gyroTurn(DRIVE_SPEED, 125);
                gyroHold(DRIVE_SPEED, 125, 0.5);
                gyroDrive(DRIVE_SPEED, 40, 125);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroDrive(DRIVE_SPEED, -75, 125);
            }

            //Right
            else if (detector.getXPosition() > 420)
            {
                vuforia.stop();
                Lift(DRIVE_SPEED, value);
                gyroTurn(DRIVE_SPEED, 45);
                gyroHold(DRIVE_SPEED, 45, 0.5);
                gyroDrive(DRIVE_SPEED, 3, 45);
                gyroTurn(DRIVE_SPEED, -45);
                gyroHold(DRIVE_SPEED, -45, 0.5);
                gyroDrive(DRIVE_SPEED, 25, -45);
                gyroTurn(DRIVE_SPEED, 0);
                gyroHold(DRIVE_SPEED, 0, 0.5);
                gyroDrive(DRIVE_SPEED, 8, 0);
                gyroDrive(DRIVE_SPEED, -8, 0);
                gyroTurn(DRIVE_SPEED, 90);
                gyroHold(DRIVE_SPEED, 90, 0.5);
                gyroDrive(DRIVE_SPEED, 73, 90);
                gyroTurn(DRIVE_SPEED, 125);
                gyroHold(DRIVE_SPEED, 125, 0.5);
                gyroDrive(DRIVE_SPEED, 40, 125);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroDrive(DRIVE_SPEED, -75, 125);
            }

            //Left
            else if (detector.getXPosition() < 180)
            {
                vuforia.stop();
                Lift(DRIVE_SPEED,value);
                gyroTurn(DRIVE_SPEED, 45);
                gyroHold(DRIVE_SPEED, 45, 0.5);
                gyroDrive(DRIVE_SPEED, 3, 45);
                gyroTurn(DRIVE_SPEED, 45);
                gyroHold(DRIVE_SPEED, 45, 0.5);
                gyroDrive(DRIVE_SPEED, 20, 45);
                gyroTurn(DRIVE_SPEED, 0);
                gyroHold(DRIVE_SPEED, 0, 0.5);
                gyroDrive(DRIVE_SPEED, 15, 0);
                gyroDrive(DRIVE_SPEED, -15, 0);
                gyroTurn(DRIVE_SPEED, 90);
                gyroHold(DRIVE_SPEED, 90, 0.5);
                gyroDrive(DRIVE_SPEED, 35, 90);
                gyroTurn(DRIVE_SPEED, 125);
                gyroHold(DRIVE_SPEED, 125, 0.5);
                gyroDrive(DRIVE_SPEED, 40, 125);
                flickServo.setPosition(Servo.MIN_POSITION);
                sleep(1000);
                gyroDrive(DRIVE_SPEED, -75, 125);
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();

            StopDriving();
        }
    }

    public void StopDriving ()
    {
        leftMotorFront.setPower(0);
        rightMotorBack.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        liftMotor.setPower(0);
    }

    public void Lift (double power, int distance)
    {
        int moveNumber;
        int move;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            moveNumber = (int)(distance);
            move = liftMotor.getCurrentPosition() + moveNumber;

            // Set Target and Turn On RUN_TO_POSITION
            liftMotor.setTargetPosition(move);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            power = Range.clip(Math.abs(power), 0.0, 1.0);
            liftMotor.setPower(power);

            while (opModeIsActive() && liftMotor.getCurrentPosition() < liftMotor.getTargetPosition()) {
                telemetry.addData("Current Value", liftMotor.getCurrentPosition());
                telemetry.addData("Target Value", liftMotor.getTargetPosition());
                telemetry.update();
                idle();
            }

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
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}