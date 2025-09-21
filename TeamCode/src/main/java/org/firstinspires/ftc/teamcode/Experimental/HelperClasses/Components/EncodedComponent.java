package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components;

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
