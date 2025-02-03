package pedroPathing.opmodes;

import static java.lang.Thread.sleep;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aditi Aryamane - 21386 Tx-Rx
 * @version 2.0, 11/28/2024
 * txrx 1/19/2025
 */

//Cases tutorial that i put up here becuz it took too much space
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
//if (!follower.isBusy()) {
/* Score Preload */



//}

@Autonomous(name = "Auton Right - Specimen", group = "Autonomous")
public class PedroSpecimenAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, sleepTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private Servo swap, Wrist, Rotation, Sample;
    private DcMotor Lift, Lift2, rig1, rig2;
    private int pathState;
    private double rotPos = 0.17, rotPick = 0.42, rotDelta = 0.05, rotWait = 3*rotDelta;
    private double wristScore = 0, wristPick = 0.9, wristSpecial = (wristScore*3+wristPick)/4.0, wristPick4 = 0.75;
    private double closeClaw = 0, openClaw = 0.3;
    private int liftScore = 1500;
    private double grabDelay = 0.75;
    double rotCor = 0.0075, liftPow = 0.8, rotScore = rotPos+0.067, rotBack = 0.21, rotSpec=0.3022;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 60, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(38, 70, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickSamplePose = new Pose(28, 26.2, Math.toRadians(0));//pickup1pose
    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickSpecimenPose = new Pose(21, 24, Math.toRadians(0));//pickup1pose

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose dragPose = new Pose(60, 30, Math.toRadians(180));//pickup2pose

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose dragPose2 = new Pose(13, 13, Math.toRadians(180)); //pickup3pose

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose pickPose = new Pose(24, 23.2, Math.toRadians(180));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose scorePose2 = new Pose(45, 68, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(45, 65, Math.toRadians(180));
    private final Pose midwayPose = new Pose(37.5/2, 0, Math.toRadians(0));
    private final Pose parkPose = new Pose(6, 6, Math.toRadians(0));

    private final Pose parkControlPose = new Pose(6, 6, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain prepToPick, goToMidwayPose2, goToPickPose, dragSample, pickPickup2, scorePickup2, pickPickup3, scorePickup3;

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
        prepToPick = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickSamplePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickSamplePose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToPickPose = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSamplePose), new Point(pickSpecimenPose)))
                .setLinearHeadingInterpolation(pickSamplePose.getHeading(), pickSpecimenPose.getHeading())
                .build();

        /* This is our dragSample PathChain. We are using a single path with a BezierLine, which is a straight line. */
        dragSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragPose), new Point(dragPose2)))
                .setLinearHeadingInterpolation(dragPose.getHeading(), dragPose2.getHeading())
                .build();

        /* This is our pickPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragPose2), new Point(pickPose)))
                .setLinearHeadingInterpolation(dragPose2.getHeading(), pickPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickPose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickPose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickPose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickPose.getHeading(), scorePose3.getHeading())
                .build();


        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose3), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        telemetry.addData("Current Path State: ", pathState);
        double scoreDelay = 0.75;
        switch (pathState) {
            case 0: //TODO: move to scorePreload and after 1 second release specimen
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
               /* if (pathTimer.getElapsedTimeSeconds()>scoreDelay) {
                    Sample.setPosition(openClaw);
                    setPathState(14);
                }

                */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>3) {
                    score();
                    setPathState(25);
                }
                break;
            case 25:
                if (!Lift.isBusy() && !Lift2.isBusy() && Lift.getCurrentPosition()>liftScore-100) {
                    Sample.setPosition(openClaw);
                    comeBack();
                    follower.followPath(prepToPick);
                    setPathState(2);
                }
                /*if(pathTimer.getElapsedTimeSeconds()>scoreDelay+1) {
                    score();
                    //sleep(1000);
                    Sample.setPosition(openClaw);
                    setPathState(14);
                    //sleep(5);
                }*/
                break;
            case 2: //TODO: drag sample to observation zone
                if (!follower.isBusy()) {
                    Rotation.setPosition(rotPick);
                    Wrist.setPosition(0);
                    setPathState(50);
                }
                break;
            case 50:
                if (Sample.getPosition()==closeClaw) {
                    Rotation.setPosition(rotSpec-0.02);
                    Wrist.setPosition(0.25);
                    setPathState(3);
                    //setPathState(3);
                }
                else if (Rotation.getPosition()>rotPick-0.01) {
                    Sample.setPosition(closeClaw);
                }
                break;
            case 3: //TODO: move to pickup specimen
                if (Rotation.getPosition()<rotSpec) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    Sample.setPosition(openClaw);
                    //setPathState(4);
                }
                //sleep(5);
                break;
            case 4: //TODO: pickup specimen
                if(pathTimer.getElapsedTimeSeconds()>1) {
                    Sample.setPosition(closeClaw);
                    sleep(500);
                    Rotation.setPosition(rotPos+0.06);
                    swap.setPosition(0.4);
                    Lift.setTargetPosition(350);
                    Lift2.setTargetPosition(350);
                    Lift.setPower(liftPow);
                    Lift2.setPower(liftPow);
                    Wrist.setPosition(wristPick);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds()>0.5){
                follower.followPath(scorePickup2, true);
                setPathState(15);
            }
            case 15:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds()>2){
                        score();
                        sleep(500);
                        Sample.setPosition(openClaw);
                        setPathState(5);
                    }
                }
                break;
            case 5: //todo: set rotation to scoring pose
                if (pathTimer.getElapsedTimeSeconds()>3) {
                    //Sample.setPosition(0);
                    Rotation.setPosition(rotBack);
                    Wrist.setPosition(0);
                    sleep(500);
                    setPathState(6);
                }
                break;
            case 6: //todo: go to scoring pose 2
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2,true);
                    setPathState(7);
                }
                //sleep(5);
                break;
            case 7:
                if (!follower.isBusy()){
                    score();
                    sleep(500);
                    Sample.setPosition(0.3);
                    setPathState(8);
                }
                break;
            case 8: //todo: go to pick pose
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds()>1) {
                    comeBack();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pickPickup3,true);
                    setPathState(12);
                }
                //sleep(5);
                break;
            case 12:// todo: pick up 3rd specimen
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    Rotation.setPosition(rotSpec);
                    Wrist.setPosition(0.5);
                    sleep(1000);
                    Sample.setPosition(0);
                    setPathState(5);
                }
                break;
            case 13:// todo: pick up 3rd specimen
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pickPickup3,true);
                    setPathState(9);
                }
                //sleep(5);
                break;
            case 9: //todo: set to score pose
                /* This case checks the robot's position and will wait until the robot positzzion is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    Rotation.setPosition(rotBack);
                    Wrist.setPosition(0);
                    Sample.setPosition(0);
                    setPathState(10);
                }
                break;
            case 10: //todo: score 3rd specimen
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(pathTimer.getElapsedTimeSeconds()>2) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(11);
                }
                break;
            case 11: //todo: go to park pose
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds()>1) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(-824636);
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
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

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
        follower.setMaxPower(0.8);
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
        Rotation.setPosition(0.16+rotCor); //0.1675
        Wrist.setPosition(1);
        Sample.setPosition(0);
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
        Sample.setPosition(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
    public void score() {
        Rotation.setPosition(rotScore);
        Lift.setTargetPosition(liftScore);//To be updated, Belt is loose
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(liftScore);//To be updated, Belt is loose
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
    public void score2() {
        Rotation.setPosition(rotBack);
        Lift.setTargetPosition(1500);//To be updated, Belt is loose
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(1500);//To be updated, Belt is loose
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
       //Sample.setPosition(0.3);
        Rotation.setPosition(0.16+rotCor);
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
}

