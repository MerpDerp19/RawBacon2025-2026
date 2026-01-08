package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.Utility.AprilTagVision;
import org.firstinspires.ftc.teamcode.Utility.OdometryGlobalCoordinatePosition;

import java.util.List;


@Autonomous(name = "MotifAutoRed")
public class MotifAutoRed extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 336.87;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontright", rbName = "backright", lfName = "frontleft", lbName = "backleft";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    int color;
    int motif = 0;
    private AprilTagVision vision = new AprilTagVision();

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }
    private MotifAutoRed.LaunchState launchState;

    private enum AutoState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        COMPLETE;
    }
    private MotifAutoRed.AutoState autoState;
    /*private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }
    private AutonomousState autonomousState;
*/


    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    org.firstinspires.ftc.teamcode.Utility.OdometryGlobalCoordinatePosition globalPositionUpdate;


    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime launcherTimer = new ElapsedTime();

    final double LAUNCHER_TARGET_VELOCITY = 50;
    final double LAUNCH_TIME = 2;
    final double FEED_TIME = 0.20;
    final double TIME_BETWEEN_SHOTS = 2;
    int shotsToFire = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        vision.init(hardwareMap);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        // autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;



        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 50);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));



        // A goToPosition method to copy and paste from
        //goToPosition(0*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 0, 0.5*COUNTS_PER_INCH);

        // V START WRITING YOUR AUTO HERE!!!! V

        goToPosition(0 * COUNTS_PER_INCH, -36 * COUNTS_PER_INCH, 0.5, -45, 0.5 * COUNTS_PER_INCH);

        AprilTagDetection tag = null;
        //april tag vision
        List<AprilTagDetection> detections = vision.getDetections();
        if (!detections.isEmpty()) {
            tag = detections.get(0);
            telemetry.addData("Tag ID", tag.id);
            int motif = tag.id;
            if (tag.ftcPose != null) {
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("yaw ", tag.ftcPose.yaw);

            }
        } else {
            telemetry.addLine("No tags detected");
            int motif = 0;
        }

        sleep(1000);

        goToPosition(0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);

        if (motif == 21) {
            launcher.setVelocity(24);
            sleep(4000);
            leftFeeder.setPower(-1);
            rightFeeder.setPower(1);
            sleep(150);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        } else if (motif == 23) {
            leftFeeder.setPower(-1);
            rightFeeder.setPower(1);
            sleep(150);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            sleep(150);
            leftFeeder.setPower(-1);
            rightFeeder.setPower(1);
            sleep(150);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }

        launcher.setVelocity(47);
        sleep(4000);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(1500);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(1500);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(1500);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(3000);

        launcher.setVelocity(50);
        sleep(4000);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(1500);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(1500);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(1500);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(150);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        sleep(3000);

        //sleep(30000);



        goToPosition(18 * COUNTS_PER_INCH,-20 * COUNTS_PER_INCH, 0.5, 0, 0.5 * COUNTS_PER_INCH);

        sleep(30000);


        //launches artifacts
//        launch(true);
//
//        //code for launching 3
//        if(launch(false)) {
//            shotsToFire -= 1;
//            if(shotsToFire > 0) {
//                launch(true);
//            } else {
////                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                launcher.setVelocity(0);
//
//            }
//        }












        while (opModeIsActive()) {

            switch (autoState) {
                case LAUNCH:
                    //feef
                    if(launch(false)) {
                        shotsToFire -= 1;
                        if(shotsToFire > 0) {
                            launch(true);
                        } else {
//                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            launcher.setVelocity(0);

                        }
                    }
                    if(shotsToFire == 0) {
                        autoState = AutoState.WAIT_FOR_LAUNCH;
                        shotsToFire = 3;
                    }
                    break;
                case WAIT_FOR_LAUNCH:
                    //feen
                    break;
                case DRIVING_AWAY_FROM_GOAL:
                    //spaghetti
                    break;
                case COMPLETE:
                    //mario
                    break;
            }

















            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();

        }

        //Stop the thread
        globalPositionUpdate.stop();

    }



    //still need to use last 3 variables to set power to motors
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();


        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() & distance > allowableDistanceError) {
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            //double distancein = (distance / COUNTS_PER_INCH) / 20;
            double math = Math.cbrt((Math.abs(distance / (COUNTS_PER_INCH)) / 20) + 0.2) - 0.37;
            double powervalue = Math.min(math, allowableDistanceError);
            //double math = 2;
            // double powervalue = Math.min(math, 1);


            //if (distance < 3 * COUNTS_PER_INCH) powervalue = 0.4;


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation()) * 0.004;


            double x_rotated = (robot_movement_x_component * Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation())) - robot_movement_y_component * Math.sin(Math.toRadians(globalPositionUpdate.returnOrientation()))) * 1.3;
            double y_rotated = robot_movement_x_component * Math.sin(Math.toRadians(globalPositionUpdate.returnOrientation())) + robot_movement_y_component * Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation()));

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());





            double denominator = Math.max(Math.abs(y_rotated) + Math.abs(x_rotated) + Math.abs(pivotCorrection), 1);
            // if not working take Math.abs of powervalue
            left_back.setPower((pivotCorrection) + ((y_rotated - x_rotated) / denominator) * powervalue);
            left_front.setPower((pivotCorrection) + ((y_rotated + x_rotated) / denominator) * powervalue);
            right_back.setPower((-pivotCorrection) + ((y_rotated + x_rotated) / denominator) * powervalue);
            right_front.setPower((-pivotCorrection) + ((y_rotated - x_rotated) / denominator) * powervalue);

            telemetry.update();
        }


        double disterr = allowableDistanceError / (COUNTS_PER_INCH * 2);
        while (opModeIsActive() & (globalPositionUpdate.returnOrientation() < (desiredRobotOrientation - disterr) || globalPositionUpdate.returnOrientation() > (desiredRobotOrientation + disterr))) {

            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation());


            double math1 = 1;
            double power = 1;

            if (pivotCorrection < 0) {
                math1 = Math.cbrt((pivotCorrection / 50) - 0.4) + 0.67;
                power = Math.max(math1, -0.5);
            } else {
                math1 = Math.cbrt((pivotCorrection / 50) + 0.4) - 0.67;
                power = Math.min(math1, 0.5);
            }






            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());


            left_back.setPower(power);
            left_front.setPower(power);
            right_back.setPower(-power);
            right_front.setPower(-power);


            telemetry.update();

        }
        left_back.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        droneLauncher = hardwareMap.get(Servo.class, "launcher");
//        droneLauncher.setPosition(0);
//
//        Grabber = hardwareMap.get(Servo.class, "rightgrabber");
//        Grabber2 = hardwareMap.get(Servo.class, "leftgrabber");
//        GrabberPivot = hardwareMap.get(Servo.class, "grabberPivot");
//
//        Grabber.setPosition(0.14);
//        Grabber2.setPosition(0.32);
//        GrabberPivot.setPosition(0.77);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = MotifAutoRed.LaunchState.PREPARE;
                    shotTimer.reset();
                    launcherTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcherTimer.seconds() > LAUNCH_TIME){
                    launchState = MotifAutoRed.LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = MotifAutoRed.LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }
}