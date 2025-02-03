package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class DriveEncoderLConstants {
    static {

        DriveEncoderConstants.forwardTicksToInches = 0.028;
        DriveEncoderConstants.strafeTicksToInches = 1;
        DriveEncoderConstants.turnTicksToInches = 1;

        DriveEncoderConstants.robot_Width = 17.5;
        DriveEncoderConstants.robot_Length = 16;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
        /*
        TwoWheelConstants.forwardTicksToInches = 0.002;
        TwoWheelConstants.strafeTicksToInches = 0.002;
        TwoWheelConstants.forwardY = 11.0/16;
        TwoWheelConstants.strafeX = 1;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "RF"; //Expansion Port 0 - Blue
        TwoWheelConstants.strafeEncoder_HardwareMapName = "RB"; //Expansion Port 1 - Yellow
        //Port 0 - RF, Port 1 - RB
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

         */
    }
}