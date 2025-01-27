package pedroPathing.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Auton Left Sample", group = "Examples")
public class SampleAutoM3 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, sleepTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private Servo swap, Wrist, Rotation, Sample;
    private DcMotor Lift, Lift2, rig1, rig2;
    private int pathState;
    double rotCor = 0, liftPow = 0.9;
    double delayScore = 0.5, delayPickUp = 1.5, delayGrab = 0.25;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(17, 129, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(31.5, 123.25, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(31.5, 133, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(38, 133.5, Math.toRadians(45));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(63, 98, Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(75, 120, Math.toRadians(270));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private double rotPos = 0.17, rotPick = 0.34, rotDelta = 0.05, rotWait = 3*rotDelta;
    private double wristScore = 0, wristPick = 1, wristSpecial = (wristScore*3+wristPick)/4.0;
    private double closeClaw = 0, openClaw = 0.3;
    private double grabDelay = 0.75, scoreDelay = 0.75;

    private int waitLiftPos = 2000;

    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        telemetry.addData("Current Path State: ", pathState);
        switch (pathState) {
            case 0:
                Rotation.setPosition(rotPos);
                Wrist.setPosition(wristPick);
                score();
                follower.followPath(scorePreload);
                setPathState(1);
                //sleep(5);
                break;
            case 1:
                if (!Lift.isBusy() && !Lift2.isBusy() && Lift.getCurrentPosition()>2900 && !follower.isBusy()) {
                    Wrist.setPosition(wristScore);
                    setPathState(2);
                }
                break;
            case 2:
                boolean condition = Wrist.getPosition()<wristScore+0.2;
                if (pathTimer.getElapsedTimeSeconds()>scoreDelay) {
                    Sample.setPosition(openClaw);
                    Rotation.setPosition(rotPick-rotDelta);
                    Wrist.setPosition(wristPick);
                    comeBack();
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && !Lift.isBusy() && !Lift2.isBusy() && Rotation.getPosition()>rotPick-rotWait && Lift.getCurrentPosition()<500) {
                    Rotation.setPosition(rotPick);
                    setPathState(13);
                }
                //sleep(5);
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds()>grabDelay) {
                    Sample.setPosition(closeClaw);
                    Rotation.setPosition(rotPos);
                    score();
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy() && !Lift.isBusy() && !Lift2.isBusy() && Rotation.getPosition()<rotPos+rotWait && Lift.getCurrentPosition()>2500) {
                    Wrist.setPosition(wristScore);
                    setPathState(5);
                }
                break;
            case 5:
                condition = Wrist.getPosition()<wristScore+0.2;
                if (pathTimer.getElapsedTimeSeconds()>scoreDelay) {
                    Sample.setPosition(openClaw);
                    Rotation.setPosition(rotPick-rotDelta);
                    Wrist.setPosition(wristPick);
                    comeBack();
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && !Lift.isBusy() && !Lift2.isBusy() && Rotation.getPosition()>rotPick-rotWait && Lift.getCurrentPosition()<500) {
                    Rotation.setPosition(rotPick);
                    setPathState(14);
                }
                //sleep(5);
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds()>grabDelay) {
                    Sample.setPosition(closeClaw);
                    Rotation.setPosition(rotPos);
                    score();
                    follower.followPath(scorePickup2,true);
                    setPathState(7);
                }
            case 7:
                if (!follower.isBusy() && !Lift.isBusy() && !Lift2.isBusy() && Rotation.getPosition()<rotPos+rotWait && Lift.getCurrentPosition()>2500) {
                    Wrist.setPosition(wristScore);
                    setPathState(8);
                }
                break;
            case 8:
                condition = Wrist.getPosition()<wristScore+0.2;
                if (pathTimer.getElapsedTimeSeconds()>scoreDelay) {
                    Sample.setPosition(openClaw);
                    Rotation.setPosition(rotPick-rotDelta);
                    Wrist.setPosition(wristPick);
                    comeBack();
                    swap.setPosition(0.25);
                    setPathState(50);
                }
                break;
            case 50:
                if (!follower.isBusy() && !Lift.isBusy() && !Lift2.isBusy() && Rotation.getPosition()>rotPick-rotWait && Lift.getCurrentPosition()<500) {
                    follower.followPath(grabPickup3,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && !Lift.isBusy() && !Lift2.isBusy() && Rotation.getPosition()>rotPick-rotWait && Lift.getCurrentPosition()<500) {
                    Rotation.setPosition(rotPick);
                    setPathState(15);
                }
                //sleep(5);
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds()>grabDelay) {
                    Sample.setPosition(closeClaw);
                    Rotation.setPosition(rotPos);
                    score();
                    swap.setPosition(0.4);
                    follower.followPath(scorePickup3,true);
                    setPathState(10);
                }
            case 10:
                if (!follower.isBusy() && !Lift.isBusy() && !Lift2.isBusy() && Rotation.getPosition()<rotPos+rotWait && Lift.getCurrentPosition()>2500) {
                    Wrist.setPosition(wristScore);
                    setPathState(11);
                }
                break;
            case 11:
                condition = Wrist.getPosition()<wristScore+0.2;
                if (pathTimer.getElapsedTimeSeconds()>scoreDelay) {
                    Sample.setPosition(openClaw);
                    Rotation.setPosition(rotPos+0.035);
                    Wrist.setPosition(wristSpecial);
                    comeBack();
                    follower.followPath(park, false);
                    setPathState(12);
                }
                break;
        }
        updateTelemetry(telemetry);
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        sleepTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);
        buildPaths();
        rig1 = hardwareMap.get(DcMotor.class, "rig1");
        rig2 = hardwareMap.get(DcMotor.class, "rig2");
        Lift = hardwareMap.get(DcMotor.class, "lift");
        Lift2 = hardwareMap.get(DcMotor.class, "lift2");
        Sample = hardwareMap.get(Servo.class, "sample");
        Rotation = hardwareMap.get(Servo.class, "rotate");
        rig1 = hardwareMap.get(DcMotor.class, "rig1");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        rig1.setDirection(DcMotorSimple.Direction.FORWARD);
        //rig1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rig2.setDirection(DcMotorSimple.Direction.REVERSE);
        //rig2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wrist = hardwareMap.get(Servo.class, "wrist");
        swap = hardwareMap.get(Servo.class, "switch");
        Rotation.setPosition(rotPos);
        Wrist.setPosition(wristPick);
        Sample.setPosition(closeClaw);
        swap.setPosition(0.4);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
    public void score() {
        Lift.setTargetPosition(3080);//To be updated, Belt is loose
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(3080);//To be updated, Belt is loose
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(liftPow);
        //while (Lift.isBusy() && Lift2.isBusy()) {
        telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
        telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
        telemetry.addData("Current Position Lift 2:", Lift2.getCurrentPosition());
        telemetry.addData("Target Position Lift 2:", Lift2.getTargetPosition());
        telemetry.update();
        //}

    }
    public void comeBack() {
        Sample.setPosition(0.3);
        Wrist.setPosition(1);
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(0);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(liftPow);
        //while (Lift.isBusy() && Lift2.isBusy()) {
        telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
        telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
        telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
        telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
        telemetry.update();
        //}
    }
    public void sleep(double x) {
        sleepTimer.resetTimer();
        while (sleepTimer.getElapsedTimeSeconds()<x) x=x;
    }
}
