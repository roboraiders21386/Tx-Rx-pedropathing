package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0005;
        TwoWheelConstants.strafeTicksToInches = 0.0005;
        TwoWheelConstants.forwardY = 7;//11.0/16;
        TwoWheelConstants.strafeX = 3;//1;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "RB"; //Expansion Port 1 - Blue
        TwoWheelConstants.strafeEncoder_HardwareMapName = "RF"; //Expansion Port 0 - Yellow
        //Port 0 - RF, Port 1 - RB
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);


    }
}