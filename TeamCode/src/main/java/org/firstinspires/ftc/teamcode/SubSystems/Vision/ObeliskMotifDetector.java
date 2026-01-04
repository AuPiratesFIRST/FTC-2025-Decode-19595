        package org.firstinspires.ftc.teamcode.SubSystems.Vision;
        
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
        import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;
        
        import java.util.HashMap;
        import java.util.List;
        import java.util.Map;
        
        /**
         * ObeliskMotifDetector
         *
         * Detects which OBELISK AprilTag (IDs 21,22,23) is visible and maps it to a 3-color motif.
         * NOTE: The mapping below (21->GPP, 22->PGP, 23->PPG) is an example. Verify event documentation
         * or change the mapping to match your field/event.
         *
         * Usage:
         *   ObeliskMotifDetector detector = new ObeliskMotifDetector(aprilTagNavigator, telemetry);
         *   // call detector.update() repeatedly (e.g. in init loop)
         *   if (detector.getMotif() != null) { // got motif }
         */
        public class ObeliskMotifDetector {
        
            private final AprilTagNavigator april;
            private final Telemetry telemetry;
        
            // Tag ID -> motif mapping (change if your event uses a different mapping)
            private final Map<Integer, ArtifactColor[]> motifByTag = new HashMap<>();
        
            private ArtifactColor[] currentMotif = null;
            private int currentTagId = -1;
        
            public ObeliskMotifDetector(AprilTagNavigator aprilTagNavigator, Telemetry telemetry) {
                this.april = aprilTagNavigator;
                this.telemetry = telemetry;
        
                // Default mapping (example):
                // 21 -> GPP
                // 22 -> PGP
                // 23 -> PPG
                motifByTag.put(21, new ArtifactColor[] { ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE });
                motifByTag.put(22, new ArtifactColor[] { ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE });
                motifByTag.put(23, new ArtifactColor[] { ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN });
            }
        
            /**
             * Call repeatedly (non-blocking). Returns true if the motif was found/updated on this call.
             */
            public boolean update() {
                 List<AprilTagDetection> detections = april.getRawDetections(); 
                if (detections == null || detections.isEmpty()) {
                    telemetry.addData("Obelisk", "No detections");
                    return false;
                }
        
                // Find the nearest obelisk tag (21,22,23) among detections
                AprilTagDetection best = null;
                for (AprilTagDetection d : detections) {
                    if (april.isObeliskTag(d)) {
                        if (best == null || d.ftcPose.range < best.ftcPose.range) {
                            best = d;
                        }
                    }
                }
        
                if (best == null) {
                    telemetry.addData("Obelisk", "No obelisk tag found");
                    return false;
                }
        
                int id = best.id;
                if (id == currentTagId) {
                    telemetry.addData("Obelisk", "Tag %d (same) range=%.1f", id, best.ftcPose.range);
                    return false; // no change
                }
        
                ArtifactColor[] motif = motifByTag.get(id);
                if (motif == null) {
                    telemetry.addData("Obelisk", "Unknown obelisk ID: %d", id);
                    return false;
                }
        
                currentTagId = id;
                currentMotif = motif;
                telemetry.addData("Obelisk", "Tag %d detected -> motif=%s", id, ArtifactColorToString(motif));
                return true;
            }
        
            public ArtifactColor[] getMotif() {
                return currentMotif == null ? null : currentMotif.clone();
            }
        
            public int getMotifTagId() {
                return currentTagId;
            }
        
            private String ArtifactColorToString(ArtifactColor[] motif) {
                if (motif == null) return "";
                StringBuilder sb = new StringBuilder();
                for (ArtifactColor c : motif) sb.append(c.getCharacter());
                return sb.toString();
            }
        
            /**
             * Optional: allow overriding the mapping (if your event provides mapping elsewhere)
             */
            public void setMapping(int tagId, ArtifactColor[] motif) {
                motifByTag.put(tagId, motif.clone());
    }
}