package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001;
        ThreeWheelConstants.strafeTicksToInches = .0012;
        ThreeWheelConstants.turnTicksToInches = .001;
        ThreeWheelConstants.leftY = 6.6;
        ThreeWheelConstants.rightY = -6.6;
        ThreeWheelConstants.strafeX = 1.0;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightEncoder";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightRear";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




