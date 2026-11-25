package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.List;

public class PanelsDraw {
    private static final double ROBOT_RADIUS = 9;
    private static final int SAMPLES_PER_PATH = 24;
    private static final double MIN_SECONDS_BETWEEN_DRAWS = 0.1;

    private static final FieldManager field = PanelsField.INSTANCE.getField();
    private static final Style pathStyle = new Style("", "#4CAF50", 0.75);
    private static final Style ghostStyle = new Style("", "#9E9E9E", 0.75);
    private static final Style robotStyle = new Style("", "#3F51B5", 0.75);
    private static long lastDrawNanos = 0;

    static {
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public static void draw(List<Path> paths, List<Pose> keyPoses, Pose robot) {
        long now = System.nanoTime();
        if ((now - lastDrawNanos) / 1e9 < MIN_SECONDS_BETWEEN_DRAWS) return;
        lastDrawNanos = now;
        for (Path path : paths) drawPath(path, pathStyle);
        for (Pose pose : keyPoses) drawRobot(pose, ghostStyle);
        if (robot != null) drawRobot(robot, robotStyle);
        field.update();
    }

    private static void drawPath(Path path, Style style) {
        field.setStyle(style);
        Pose previous = path.get(0.0);
        for (int i = 1; i <= SAMPLES_PER_PATH; i++) {
            Pose next = path.get(i / (double) SAMPLES_PER_PATH);
            field.moveCursor(previous.x(), previous.y());
            field.line(next.x(), next.y());
            previous = next;
        }
    }

    private static void drawRobot(Pose pose, Style style) {
        field.setStyle(style);
        field.moveCursor(pose.x(), pose.y());
        field.circle(ROBOT_RADIUS);
        field.moveCursor(pose.x(), pose.y());
        field.line(pose.x() + Math.cos(pose.heading()) * ROBOT_RADIUS, pose.y() + Math.sin(pose.heading()) * ROBOT_RADIUS);
    }
}