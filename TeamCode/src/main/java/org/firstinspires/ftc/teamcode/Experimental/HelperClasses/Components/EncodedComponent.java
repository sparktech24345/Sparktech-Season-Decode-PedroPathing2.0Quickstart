package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;

public abstract class EncodedComponent extends Component {
    protected Encoder componentEncoder;


    public abstract <T extends EncodedComponent> T useWithEncoder(boolean useWithEncoder);

    public Encoder getEncoderInstance() {
        return componentEncoder;
    }

    @Override
    public double getPosition() {
        return componentEncoder.getEncoderPosition();
    }
}
