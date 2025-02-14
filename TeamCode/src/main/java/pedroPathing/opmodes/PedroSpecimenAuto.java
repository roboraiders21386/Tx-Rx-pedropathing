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
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.constants.RobotConstants;

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

@Autonomous(name = "Pick Specimen", group = "Autonomous")
public class PedroSpecimenAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, sleepTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private Servo swap, Wrist, Rotation, Sample;
    private DcMotor Lift, Lift2, rig1, rig2;
    private int pathState;
    private double rotPos = 0.17, rotPick = 0.42, rotDelta = 0.01, rotWait = 3*rotDelta;
    private double wristScore = 0, wristPick = 0.75, wristSpecial = (wristScore*3+wristPick)/4.0, wristPick4 = 0.75, wristSpecPick = 0;
    private double closeClaw = 0, openClaw = 0.3;
    private int liftScore = 1800;
    private double grabDelay = 2;
    double rotCor = 0.0075, liftPow = 0.65, rotScore = rotPos+0.08, rotBack = 0.21, rotSpec=0.12+0.015;


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
    private final Pose scorePose = new Pose(40, 73, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickSamplePose = new Pose(26.65, 27, Math.toRadians(0));//pickup1pose
    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickSamplePose2 = new Pose(26.3, 17.5, Math.toRadians(0));//pickup1pose
    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickSpecimenPose = new Pose(17, 17, Math.toRadians(0));//pickup1pose
    private final Pose pickotherSpecimenPose = new Pose(19, 30, Math.toRadians(0));//pickup1pose

    /** Park Pose for our robot, after we do all of the scoring. */
    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose scorePose2 = new Pose(40, 72, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(40, 69, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(40, 70.5, Math.toRadians(0));
    private final Pose midwayPose = new Pose(25, 20, Math.toRadians(0));
    private final Pose parkPose = new Pose(9, 26, Math.toRadians(0));
    private final Pose pickMidwayPose = new Pose(35, 31, Math.toRadians(0));
    private final Pose pickMidwayPose2 = new Pose(35, 70, Math.toRadians(0));
    private final Pose pickMidwayPose3 = new Pose(30, 62.5, Math.toRadians(0));
    private final Pose parkControlPose = new Pose(6, 6, Math.toRadians(0));
    private final Pose dragFirst1 = new Pose(26.65, 37, Math.toRadians(0));
    private final Pose dragFirst2 = new Pose(57.5, 37, Math.toRadians(0));
    private final Pose dragFirst3 = new Pose(57.5, 27, Math.toRadians(0));
    private final Pose dragFirst4 = new Pose(13.5, 27, Math.toRadians(0));
    private final Pose dragSecond1 = new Pose(57.5, 17.5, Math.toRadians(0));
    private final Pose dragSecond2 = new Pose(17, 17.5, Math.toRadians(0));
    private final Pose dragThird0 = new Pose(57.5, 22.5, Math.toRadians(0));
    private final Pose dragThird1 = new Pose(57.5, 15, Math.toRadians(0));
    private final Pose dragThird2 = new Pose(17, 15, Math.toRadians(0));
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain prepToPick, goToMidwayPose2, goToPickSample1, goToPickSample2, pickPickup2, scorePickup2, pickPickup3, scorePickup3, pickPickup4, scorePickup4;
    private PathChain dragFirstOne,dragFirstTwo, dragFirstThree, dragFirstFour, dragSecondOne, dragSecondTwo, dragSecondThree, dragThirdOne, dragThirdTwo, dragThirdThree;

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
        goToPickSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickSamplePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickSamplePose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goToPickSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSamplePose), new Point(pickSamplePose2)))
                .setLinearHeadingInterpolation(pickSamplePose.getHeading(), pickSamplePose2.getHeading())
                .build();

        /* This is our pickPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSamplePose2), new Point(pickSpecimenPose)))
                .setLinearHeadingInterpolation(pickSamplePose2.getHeading(), pickSpecimenPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimenPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickSpecimenPose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose2), new Point(pickMidwayPose2), new Point(pickMidwayPose), new Point(pickotherSpecimenPose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickotherSpecimenPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickotherSpecimenPose), new Point(pickMidwayPose3), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickotherSpecimenPose.getHeading(), scorePose3.getHeading())
                .build();
        pickPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose3), new Point(pickMidwayPose3), new Point(pickMidwayPose), new Point(pickotherSpecimenPose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickotherSpecimenPose.getHeading())
                .build();
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickotherSpecimenPose), new Point(pickMidwayPose3), new Point(scorePose4)))
                .setLinearHeadingInterpolation(pickotherSpecimenPose.getHeading(), scorePose4.getHeading())
                .build();
        dragFirstOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(dragFirst1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), dragFirst1.getHeading())
                .build();
        dragFirstTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragFirst1), new Point(dragFirst2)))
                .setLinearHeadingInterpolation(dragFirst1.getHeading(), dragFirst2.getHeading())
                .build();
        dragFirstThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragFirst2), new Point(dragFirst3)))
                .setLinearHeadingInterpolation(dragFirst2.getHeading(), dragFirst3.getHeading())
                .build();
        dragFirstFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragFirst3), new Point(dragFirst4)))
                .setLinearHeadingInterpolation(dragFirst3.getHeading(), dragFirst4.getHeading())
                .build();
        dragSecondOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragFirst4), new Point(dragFirst3)))
                .setLinearHeadingInterpolation(dragFirst4.getHeading(), dragFirst3.getHeading())
                .build();
        dragSecondTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragFirst3), new Point(dragSecond1)))
                .setLinearHeadingInterpolation(dragFirst3.getHeading(), dragSecond1.getHeading())
                .build();
        dragSecondThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragSecond1), new Point(dragSecond2)))
                .setLinearHeadingInterpolation(dragSecond1.getHeading(), dragSecond2.getHeading())
                .build();
        dragThirdOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragSecond2), new Point(dragThird0)))
                .setLinearHeadingInterpolation(dragSecond2.getHeading(), dragThird0.getHeading())
                .build();
        dragThirdTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragSecond1), new Point(dragThird1)))
                .setLinearHeadingInterpolation(dragSecond1.getHeading(), dragThird1.getHeading())
                .build();
        dragThirdThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dragThird1), new Point(dragThird2)))
                .setLinearHeadingInterpolation(dragThird1.getHeading(), dragThird2.getHeading())
                .build();
        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose3), new Point(pickMidwayPose3), new Point(parkPose)));
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
                liftPow = 0.8;
                follower.followPath(scorePreload);
                Rotation.setPosition(0.17125+rotCor);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>1.5) {
                    Rotation.setPosition(rotScore);
                    score();
                    setPathState(251);
                }
                break;
            case 251:
                if (pathTimer.getElapsedTimeSeconds()>1.5 || !Lift.isBusy() && !Lift2.isBusy()) {
                    liftPow = 0.65;
                    Sample.setPosition(openClaw);
                    comeBack();
                    follower.setMaxPower(1);
                    follower.followPath(dragFirstOne);
                    setPathState(252);
                }
                break;
            case 252:
                if (follower.getPose().getY()<37.75) {
                    follower.followPath(dragFirstTwo);
                    setPathState(253);
                }
                break;
            case 253:
                if (follower.getPose().getX()>55) {
                    follower.followPath(dragFirstThree);
                    setPathState(254);
                }
                break;
            case 254:
                if (follower.getPose().getY()<30) {
                    follower.followPath(dragFirstFour);
                    setPathState(255);
                }
                break;
            case 255:
                if (follower.getPose().getX()<20) {
                    follower.followPath(dragSecondOne);
                    setPathState(256);
                }
                break;
            case 256:
                if (follower.getPose().getX()>55) {
                    follower.followPath(dragSecondTwo);
                    setPathState(257);
                }
                break;
            case 257:
                if (follower.getPose().getY()<20) {
                    follower.followPath(dragSecondThree);
                    setPathState(258);
                }
                break;
            case 258:
                if (follower.getPose().getX()<20) {
                    follower.followPath(dragThirdOne);
                    setPathState(259);
                }
                break;
            case 259:
                if (!follower.isBusy()) {
                    follower.followPath(dragThirdTwo);
                    setPathState(260);
                }
                break;
            case 260:
                if (!follower.isBusy()) {
                    follower.followPath(dragThirdThree);
                    setPathState(261);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds()>1.5 || !Lift.isBusy() && !Lift2.isBusy()) {
                    liftPow = 0.65;
                    Sample.setPosition(openClaw);
                    comeBack();
                    follower.followPath(goToPickSample1);
                    setPathState(2);
                }
                break;
            case 2: //TODO: drag sample to observation zone
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    Rotation.setPosition(rotPick-rotDelta);
                    Wrist.setPosition(wristPick);
                    setPathState(55);
                }
                break;
            case 55:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.85 || Rotation.getPosition()>rotPick-rotWait) {
                    Rotation.setPosition(rotPick);
                    setPathState(56);
                }
                break;
            case 56:
                if (pathTimer.getElapsedTimeSeconds()>1.4275) {
                    Sample.setPosition(closeClaw);
                    setPathState(50);
                }
                break;
            case 50: //go to
                if (pathTimer.getElapsedTimeSeconds()>0.25) {
                    Rotation.setPosition(rotSpec-0.02);
                    //Sample.setPosition(closeClaw);
                    setPathState(3);
                    //setPathState(3);
                }
                break;
            case 3: //TODO: move to pickup specimen
                if (pathTimer.getElapsedTimeSeconds()>2.75) {
                    Wrist.setPosition(wristSpecPick);
                    //Sample.setPosition(openClaw);
                    setPathState(74);
                }
                //sleep(5);
                break;
            case 74:
                if(pathTimer.getElapsedTimeSeconds()>0.1){
                    Sample.setPosition(openClaw);
                    setPathState(57);
                }
                break;
            case 57:
                if (pathTimer.getElapsedTimeSeconds()>0.25) {
                    Wrist.setPosition(wristPick);
                    follower.followPath(goToPickSample2);
                    Rotation.setPosition(rotPick);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds()>2.55){
                    Sample.setPosition(closeClaw);
                    Rotation.setPosition(rotSpec-0.02);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds()>2.75) {
                    Wrist.setPosition(wristSpecPick);
                    setPathState(59);
                }
                break;
            case 59:
                if(pathTimer.getElapsedTimeSeconds()>0.15){
                    Sample.setPosition(openClaw);
                    Rotation.setPosition(rotSpec);
                    follower.followPath(pickPickup2);
                    setPathState(100);
                }
                break;
            case 100:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>0.75) {
                    Sample.setPosition(closeClaw);
                    setPathState(200);
                }

                break;
            case 200:
                if (pathTimer.getElapsedTimeSeconds()>0.1) {
                    Rotation.setPosition(0.1925+rotCor);
                    Wrist.setPosition(wristPick);
                    setPathState(6);
                }
                break;
            case 6: //todo: go to scoring pose 2
                if(pathTimer.getElapsedTimeSeconds()>0.25) {
                    follower.setMaxPower(0.95);
                    follower.followPath(scorePickup2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>2){
                    Rotation.setPosition(rotScore);
                    score();
                    setPathState(245);
                }
                break;
            case 245:
                if (pathTimer.getElapsedTimeSeconds()>1.25 || !Lift.isBusy() && !Lift2.isBusy()) {
                    Sample.setPosition(openClaw);
                    comeBack();
                    setPathState(12);
                }
                break;
            case 12:// todo: pick up 3rd specimen
                if (pathTimer.getElapsedTimeSeconds()>0.25/*0.25*/) {
                    Rotation.setPosition(rotSpec);
                    Wrist.setPosition(wristSpecPick);

                    setPathState(13);
                }
                break;
            case 13:// todo: pick up 3rd specimen
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>0.75/*.75*/) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(pickPickup3,true);
                    setPathState(101);
                }
                //sleep(5);
                break;
            case 101:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>3) {
                    Sample.setPosition(closeClaw);
                    setPathState(201);
                }

                break;
            case 201:
                if (pathTimer.getElapsedTimeSeconds()>0.1) {
                    Rotation.setPosition(0.1925+rotCor);
                    Wrist.setPosition(wristPick);
                    setPathState(61);
                }
                break;
            case 61: //todo: go to scoring pose 2
                if(pathTimer.getElapsedTimeSeconds()>0.5) {
                    follower.followPath(scorePickup3);
                    setPathState(71);
                }
                break;
            case 71:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>2.75){
                    Rotation.setPosition(rotScore);
                    score();
                    setPathState(2451);
                }
                break;
            case 2451:
                if (pathTimer.getElapsedTimeSeconds()>1.25 || !Lift.isBusy() && !Lift2.isBusy()) {
                    Sample.setPosition(openClaw);
                    comeBack();
                    setPathState(111);
                }
                break;
            case 111:// todo: pick up 3rd specimen
                if (pathTimer.getElapsedTimeSeconds()>0.5/*1*/) {
                    Rotation.setPosition(rotSpec);
                    Wrist.setPosition(wristSpecPick);

                    setPathState(131);
                }
                break;
            case 131:// todo: pick up 3rd specimen
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>0.25/*0.5*/) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(pickPickup4,true);
                    setPathState(1011);
                }
                //sleep(5);
                break;
            case 1011:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.75) {
                    Sample.setPosition(closeClaw);
                    setPathState(2011);
                }

                break;
            case 2011:
                if (pathTimer.getElapsedTimeSeconds()>0.1) {
                    Rotation.setPosition(0.1925+rotCor);
                    Wrist.setPosition(wristPick);
                    setPathState(611);
                }
                break;
            case 611: //todo: go to scoring pose 2
                if(pathTimer.getElapsedTimeSeconds()>0.25) {
                    follower.followPath(scorePickup4);
                    setPathState(711);
                }
                break;
            case 711:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds()>2.75){
                    Rotation.setPosition(rotScore);
                    score();
                    setPathState(24511);
                }
                break;
            case 24511:
                if (pathTimer.getElapsedTimeSeconds()>1.25 || !Lift.isBusy() && !Lift2.isBusy()) {
                    Sample.setPosition(openClaw);
                    comeBack();
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
        follower.setMaxPower(0.92);
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
        Sample.setPosition(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
    public void score() {
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
