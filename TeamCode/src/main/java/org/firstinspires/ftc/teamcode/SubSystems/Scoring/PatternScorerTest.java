package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

/**
 * Unit tests for PatternScorer and ArtifactColor classes
 * Tests pattern generation, scoring logic, and color enumeration
 */
public class PatternScorerTest {

    public static void main(String[] args) {
        System.out.println("=== PatternScorer and ArtifactColor Test Suite ===\n");
        
        int testsPassed = 0;
        int testsTotal = 0;

        // Test 1: ArtifactColor enum values
        testsTotal++;
        try {
            assert ArtifactColor.GREEN.getCharacter() == 'G';
            assert ArtifactColor.PURPLE.getCharacter() == 'P';
            System.out.println("✓ Test 1: ArtifactColor enum values - PASSED");
            testsPassed++;
        } catch (AssertionError e) {
            System.out.println("✗ Test 1: ArtifactColor enum values - FAILED");
        }

        // Test 2: ArtifactColor.fromChar()
        testsTotal++;
        try {
            assert ArtifactColor.fromChar('G') == ArtifactColor.GREEN;
            assert ArtifactColor.fromChar('g') == ArtifactColor.GREEN;
            assert ArtifactColor.fromChar('P') == ArtifactColor.PURPLE;
            assert ArtifactColor.fromChar('p') == ArtifactColor.PURPLE;
            System.out.println("✓ Test 2: ArtifactColor.fromChar() - PASSED");
            testsPassed++;
        } catch (AssertionError | IllegalArgumentException e) {
            System.out.println("✗ Test 2: ArtifactColor.fromChar() - FAILED: " + e.getMessage());
        }

        // Test 3: ArtifactColor.fromChar() invalid input
        testsTotal++;
        try {
            try {
                ArtifactColor.fromChar('X');
                System.out.println("✗ Test 3: ArtifactColor.fromChar() invalid - FAILED (should throw exception)");
            } catch (IllegalArgumentException e) {
                System.out.println("✓ Test 3: ArtifactColor.fromChar() invalid - PASSED");
                testsPassed++;
            }
        } catch (Exception e) {
            System.out.println("✗ Test 3: ArtifactColor.fromChar() invalid - FAILED: " + e.getMessage());
        }

        // Test 4: PatternScorer pattern generation
        testsTotal++;
        try {
            ArtifactColor[] motif = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            PatternScorer scorer = new PatternScorer(motif);
            ArtifactColor[] pattern = scorer.getPattern();
            
            assert pattern.length == 9;
            assert pattern[0] == ArtifactColor.GREEN;
            assert pattern[1] == ArtifactColor.PURPLE;
            assert pattern[2] == ArtifactColor.PURPLE;
            assert pattern[3] == ArtifactColor.GREEN;
            assert pattern[4] == ArtifactColor.PURPLE;
            assert pattern[5] == ArtifactColor.PURPLE;
            assert pattern[6] == ArtifactColor.GREEN;
            assert pattern[7] == ArtifactColor.PURPLE;
            assert pattern[8] == ArtifactColor.PURPLE;
            
            System.out.println("✓ Test 4: PatternScorer pattern generation - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 4: PatternScorer pattern generation - FAILED: " + e.getMessage());
        }

        // Test 5: PatternScorer invalid motif (null)
        testsTotal++;
        try {
            try {
                new PatternScorer(null);
                System.out.println("✗ Test 5: PatternScorer null motif - FAILED (should throw exception)");
            } catch (IllegalArgumentException e) {
                System.out.println("✓ Test 5: PatternScorer null motif - PASSED");
                testsPassed++;
            }
        } catch (Exception e) {
            System.out.println("✗ Test 5: PatternScorer null motif - FAILED: " + e.getMessage());
        }

        // Test 6: PatternScorer invalid motif (wrong length)
        testsTotal++;
        try {
            try {
                ArtifactColor[] badMotif = {ArtifactColor.GREEN, ArtifactColor.PURPLE};
                new PatternScorer(badMotif);
                System.out.println("✗ Test 6: PatternScorer wrong length motif - FAILED (should throw exception)");
            } catch (IllegalArgumentException e) {
                System.out.println("✓ Test 6: PatternScorer wrong length motif - PASSED");
                testsPassed++;
            }
        } catch (Exception e) {
            System.out.println("✗ Test 6: PatternScorer wrong length motif - FAILED: " + e.getMessage());
        }

        // Test 7: PatternScorer perfect match scoring
        testsTotal++;
        try {
            ArtifactColor[] motif = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            PatternScorer scorer = new PatternScorer(motif);
            ArtifactColor[] detected = {
                ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE,
                ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE,
                ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE
            };
            int score = scorer.scorePattern(detected);
            assert score == 9;
            System.out.println("✓ Test 7: PatternScorer perfect match (9/9) - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 7: PatternScorer perfect match - FAILED: " + e.getMessage());
        }

        // Test 8: PatternScorer partial match scoring
        testsTotal++;
        try {
            ArtifactColor[] motif = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            PatternScorer scorer = new PatternScorer(motif);
            ArtifactColor[] detected = {
                ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE,
                ArtifactColor.GREEN, ArtifactColor.PURPLE
            };
            int score = scorer.scorePattern(detected);
            assert score == 5;
            System.out.println("✓ Test 8: PatternScorer partial match (5/9) - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 8: PatternScorer partial match - FAILED: " + e.getMessage());
        }

        // Test 9: PatternScorer mismatch scoring
        testsTotal++;
        try {
            ArtifactColor[] motif = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            PatternScorer scorer = new PatternScorer(motif);
            ArtifactColor[] detected = {
                ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.GREEN
            };
            int score = scorer.scorePattern(detected);
            assert score == 0;
            System.out.println("✓ Test 9: PatternScorer mismatch (0/9) - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 9: PatternScorer mismatch - FAILED: " + e.getMessage());
        }

        // Test 10: PatternScorer empty/null detected array
        testsTotal++;
        try {
            ArtifactColor[] motif = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            PatternScorer scorer = new PatternScorer(motif);
            int score1 = scorer.scorePattern(null);
            int score2 = scorer.scorePattern(new ArtifactColor[0]);
            assert score1 == 0;
            assert score2 == 0;
            System.out.println("✓ Test 10: PatternScorer empty/null detected - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 10: PatternScorer empty/null detected - FAILED: " + e.getMessage());
        }

        // Test 11: PatternScorer.patternString()
        testsTotal++;
        try {
            ArtifactColor[] pattern = {
                ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE,
                ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE
            };
            String patternStr = PatternScorer.patternString(pattern);
            assert patternStr.equals("GPPGPP");
            System.out.println("✓ Test 11: PatternScorer.patternString() - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 11: PatternScorer.patternString() - FAILED: " + e.getMessage());
        }

        // Test 12: PatternScorer.toString()
        testsTotal++;
        try {
            ArtifactColor[] motif = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN};
            PatternScorer scorer = new PatternScorer(motif);
            String str = scorer.toString();
            assert str.contains("GPG");
            assert str.contains("GPGPGPGPG");
            System.out.println("✓ Test 12: PatternScorer.toString() - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 12: PatternScorer.toString() - FAILED: " + e.getMessage());
        }

        // Test 13: Real-world scenario - SensorColorTelemetry default motif
        testsTotal++;
        try {
            ArtifactColor[] defaultMotif = {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN};
            PatternScorer scorer = new PatternScorer(defaultMotif);
            // Simulate detecting: G, P, G, P, G, P, G
            ArtifactColor[] detected = {
                ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN,
                ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE,
                ArtifactColor.GREEN
            };
            int score = scorer.scorePattern(detected);
            // Expected pattern: GPG GPG GPG
            // Detected: GPG PGP G
            // Match: G, P, G = 3 matches
            assert score == 3;
            System.out.println("✓ Test 13: Real-world scenario (SensorColorTelemetry motif) - PASSED");
            testsPassed++;
        } catch (AssertionError | Exception e) {
            System.out.println("✗ Test 13: Real-world scenario - FAILED: " + e.getMessage());
        }

        // Summary
        System.out.println("\n=== Test Summary ===");
        System.out.println("Tests Passed: " + testsPassed + "/" + testsTotal);
        if (testsPassed == testsTotal) {
            System.out.println("✓ ALL TESTS PASSED!");
        } else {
            System.out.println("✗ Some tests failed. Please review the output above.");
        }
    }
}
