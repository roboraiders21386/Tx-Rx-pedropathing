package pedroPathing.constants;

public class RobotConstants {
    static {
        double rotSpecScore = 0.21,
                rotSampScore = 0.15,
                rotPick = 0.42,
                rotDelta = 0.01,
                rotSpecPick = 0.085,
                rotLeft = 0.395;
        int liftZero = 0, liftSampScore = 3080, liftSpecScore = 1800, liftSampLowScore = 1300;
        double clawOpen = 0.3, clawClose = 0;
        int rigSet = 0, rig = 0, rigInit = 0;
        double swapH = 0.4, swapV = 0.05;
        double wristPick = 0.8, wristSpecScore = wristPick, wristSampScore = 0;
    }
}