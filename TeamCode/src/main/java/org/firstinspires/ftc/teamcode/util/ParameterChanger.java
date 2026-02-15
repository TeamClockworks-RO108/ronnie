package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

public class ParameterChanger {

    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final EdgeDetector up = new EdgeDetector(false);
    private final EdgeDetector down = new EdgeDetector(false);
    private final EdgeDetector left = new EdgeDetector(false);
    private final EdgeDetector right = new EdgeDetector(false);

    private final EdgeDetector next = new EdgeDetector(false);

    private final EdgeDetector prev = new EdgeDetector(false);

    private final DecimalFormat decimalFormat = new DecimalFormat("0", DecimalFormatSymbols.getInstance(Locale.ENGLISH));

    private boolean enabled = false;
    int currentField;

    double scale = 1;

    private final List<PField> fields = new ArrayList<>();

    public ParameterChanger(Gamepad gamepad, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        left.onPress(() -> scale = scale / 10);
        right.onPress(() -> scale = scale * 10);
        up.onPress(() -> changeField(scale));
        down.onPress(() -> changeField(-scale));
        next.onPress(() -> currentField = (currentField == fields.size() - 1) ? 0 : currentField + 1);
        prev.onPress(() -> currentField = (currentField == 0) ? fields.size() - 1 : currentField - 1);
        decimalFormat.setMaximumFractionDigits(20);
    }

    public void register(String name, Supplier<Double> getter, Consumer<Double> setter) {
        fields.add(new PField(
                setter,
                getter,
                name
        ));
    }

    public void enable(boolean enabled) {
        this.enabled = enabled;
    }

    public void update() {
        if (!enabled) return;

        up.update(gamepad.dpad_up);
        down.update(gamepad.dpad_down);
        left.update(gamepad.dpad_left);
        right.update(gamepad.dpad_right);
        prev.update(gamepad.left_bumper);
        next.update(gamepad.right_bumper);
        if (!fields.isEmpty()) {
            telemetry.addLine("Editing field " + (currentField + 1) + "/" + fields.size() + " " + fields.get(currentField).name);
            telemetry.addLine("Editor value is " + decimalFormat.format(fields.get(currentField).getter.get()));
            telemetry.addLine("Editor scale is " + decimalFormat.format(scale));
        }
    }

    private void changeField(double amount) {
        fields.get(currentField).setter.accept(fields.get(currentField).getter.get() + amount);
    }


    public static class PField {
        private final Consumer<Double> setter;
        private final Supplier<Double> getter;
        private final String name;

        public PField(Consumer<Double> setter, Supplier<Double> getter, String name) {
            this.setter = setter;
            this.getter = getter;
            this.name = name;
        }
    }

}
