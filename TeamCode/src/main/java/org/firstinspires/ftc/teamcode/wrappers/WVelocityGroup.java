package org.firstinspires.ftc.teamcode.wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Velocity-focused actuator group wrapper.
 * Provides PID + Feedforward (kS + kV*v + kA*a) control with optional voltage scaling and smoothing.
 */
public class WVelocityGroup {

    public enum FeedforwardMode {
        NONE,
        SIMPLE,     // kS + kV*v + kA*a
        CONSTANT    // constant feedforward value
    }

    private final Map<String, HardwareDevice> devices = new HashMap<>();
    private PIDController controller;
    private DoubleSupplier voltageSupplier = () -> 12.0; // default 12V
    private final ElapsedTime timer = new ElapsedTime();
    private Supplier<Object> topic; // optional external velocity supplier

    // state
    private double velocity = 0.0;
    private double velocityScaled = 0.0;
    private double targetVelocity = 0.0;
    private double power = 0.0;
    private double lastPower = 0.0;
    private boolean enabled = true;
    private boolean floating = false;

    // feedforward
    private FeedforwardMode ffMode = FeedforwardMode.NONE;
    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;
    private double constantFF = 0.0;

    // scaling
    private double velocityScaling = ((double) 1 / 28) * 60.0 * (4.0 / 3.0);

    // for accel calc
    private double prevVelScaled = 0.0;
    private double acceleration = 0.0;
    private double lastTime = Double.NaN;

    // smoothing / clamping
    private double maxPowerChangePerSec = Double.POSITIVE_INFINITY;
    private double maxPower = 1.0;

    // optional voltage value (for correction)
    private double voltage = 12.0;

    public WVelocityGroup(HardwareDevice... devices) {
        this(null, devices);
    }

    public WVelocityGroup(Supplier<Object> topic, HardwareDevice... devices) {
        this.topic = topic;
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        timer.reset();
    }

    // --------------------
    // READ VELOCITY
    // --------------------
    public void read() {
        if (topic != null) {
            Object v = topic.get();
            if (v instanceof Double) velocity = (Double) v;
            else if (v instanceof Integer) velocity = ((Integer) v).doubleValue();
            velocityScaled = velocity * velocityScaling;
            return;
        }

        for (HardwareDevice device : devices.values()) {
            if (device instanceof WEncoder) {
                velocity = ((WEncoder) device).getRawVelocity();
                velocityScaled = velocity * velocityScaling;
                return;
            }
        }

        velocity = 0.0;
        velocityScaled = 0.0;
    }

    // --------------------
    // PERIODIC LOOP
    // --------------------
    public void periodic() {
        double now = timer.seconds();
        if (Double.isNaN(lastTime)) lastTime = now;
        double dt = Math.max(1e-6, now - lastTime);

        read();

        acceleration = (velocityScaled - prevVelScaled) / dt;

        double pidOutput = 0.0;
        if (controller != null && enabled) {
            pidOutput = controller.calculate(velocityScaled, targetVelocity);
        }

        double ff = 0.0;
        switch (ffMode) {
            case SIMPLE:
                double sign = Math.signum(targetVelocity);
                ff = kS * (sign == 0 ? 1.0 : sign) + kV * targetVelocity + kA * acceleration;
                break;
            case CONSTANT:
                ff = constantFF;
                break;
            default:
                ff = 0.0;
        }

        double volts = ff + pidOutput;
        double battery = voltage != 0 ? voltage : voltageSupplier.getAsDouble();
        double desiredPower = volts / Math.max(0.0001, battery);

        desiredPower = Math.max(-1.0, Math.min(1.0, desiredPower));

        // Rate limit (smoothing)
        if (!Double.isInfinite(maxPowerChangePerSec)) {
            double maxDelta = maxPowerChangePerSec * dt;
            double delta = desiredPower - lastPower;
            if (Math.abs(delta) > maxDelta) {
                desiredPower = lastPower + Math.signum(delta) * maxDelta;
            }
        }

        this.power = desiredPower;
        this.lastPower = desiredPower;

        prevVelScaled = velocityScaled;
        lastTime = now;
    }

    // --------------------
    // WRITE TO MOTORS
    // --------------------
    public void write() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof DcMotor) {
                DcMotor m = (DcMotor) device;
                double correction = 1.0;
                double batt = voltage != 0 ? voltage : voltageSupplier.getAsDouble();
                if (batt != 0) correction = 12.0 / batt;

                if (!floating && enabled)
                    m.setPower((power * maxPower) * correction);
                else
                    m.setPower(0.0);
            }
        }
    }

    // --------------------
    // CONFIGURATION API
    // --------------------
    public WVelocityGroup setPID(double p, double i, double d) {
        if (controller == null) controller = new PIDController(p, i, d);
        else controller.setPID(p, i, d);
        return this;
    }

    public WVelocityGroup setPIDController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    public WVelocityGroup setFeedforwardSimple(double kS, double kV, double kA) {
        this.ffMode = FeedforwardMode.SIMPLE;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public WVelocityGroup setFeedforwardConstant(double constantVolts) {
        this.ffMode = FeedforwardMode.CONSTANT;
        this.constantFF = constantVolts;
        return this;
    }

    public WVelocityGroup setFeedforwardNone() {
        this.ffMode = FeedforwardMode.NONE;
        return this;
    }

    public WVelocityGroup setVelocityScaling(double scaling) {
        this.velocityScaling = scaling;
        return this;
    }

    public WVelocityGroup setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        return this;
    }

    public WVelocityGroup setVoltageSupplier(DoubleSupplier supplier) {
        this.voltageSupplier = supplier;
        return this;
    }

    public WVelocityGroup setVoltage(double voltage) {
        this.voltage = voltage;
        return this;
    }

    public WVelocityGroup setEnabled(boolean enabled) {
        this.enabled = enabled;
        return this;
    }

    public WVelocityGroup setFloat(boolean f) {
        this.floating = f;
        return this;
    }

    public WVelocityGroup setMaxPower(double max) {
        this.maxPower = max;
        return this;
    }

    public void setMaxPowerChangePerSec(double maxChange) {
        this.maxPowerChangePerSec = maxChange;
    }

    // --------------------
    // GETTERS
    // --------------------
    public double getVelocity() { return velocityScaled; }
    public double getRawVelocity() { return velocity; }
    public double getTargetVelocity() { return targetVelocity; }
    public double getPower() { return power; }
    public List<HardwareDevice> getDevices() { return new ArrayList<>(devices.values()); }
    public HardwareDevice getDevice(String name) { return devices.get(name); }
}
