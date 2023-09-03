package org.texastorque.toast.lib.pipelines;

import java.util.ArrayList;
import org.texastorque.toast.lib.DetectedObject;
import com.fasterxml.jackson.databind.JsonNode;

public class ConeDetectionPipeline extends Pipeline {

    ArrayList<DetectedObject> cones = new ArrayList<DetectedObject>();

    public ConeDetectionPipeline() {
        super("cone-detection");
    }

    @Override
    protected void init() {}

    @Override
    protected void deinit() {

    }

    @Override
    protected void update(JsonNode root) {
        root.get("cones").forEach(target -> {
            final JsonNode objectName = target.get("object_name");
            final JsonNode centerX = target.get("center_x");
            final JsonNode confidence = target.get("confidence");

            cones.add(new DetectedObject(objectName.asText(), centerX.asDouble(), confidence.asDouble()));
        });
    }

    public double getBestTargetX() {
        double lowestConfidence = Integer.MAX_VALUE;
        DetectedObject bestTarget = null;
        for (DetectedObject cone : cones) {
            if (cone.getConfidence() < lowestConfidence) {
                lowestConfidence = cone.getConfidence();
                bestTarget = cone;
            }
        }

        return bestTarget.getCenterX();
    }

}
