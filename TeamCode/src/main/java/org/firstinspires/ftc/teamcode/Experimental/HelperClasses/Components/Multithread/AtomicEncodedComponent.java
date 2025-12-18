package org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.Multithread;

import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Components.EncodedComponent;
import org.firstinspires.ftc.teamcode.Experimental.HelperClasses.Encoder;

import java.util.concurrent.atomic.AtomicReference;

public abstract class AtomicEncodedComponent extends AtomicComponent {
    protected AtomicReference<Encoder> componentEncoder;


    public abstract <T extends AtomicEncodedComponent> T useWithEncoder(boolean useWithEncoder);

    public Encoder getEncoderInstance() {
        return componentEncoder.get();
    }

    @Override
    public double getPosition() {
        return componentEncoder.get().getEncoderPosition();
    }
}
