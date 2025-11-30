# Red Alliance Auto Logic Test

## Issues Found and Fixed:

### 1. ✅ **FIXED: Spindexer Position Advancement Logic**
   - **Problem**: The spindexer position was advancing before firing the artifact.
   - **Fix Applied**: Changed sequence to: Fire from current position → Wait for firing delay → Advance spindexer → Wait for spindexer → Complete

### 2. ✅ **FIXED: Preload Scoring Sequence**
   - **Problem**: For preload, we should fire from position 0 (which has the preload), then advance.
   - **Old Flow**: Start shooter → Wait for RPM → Advance spindexer → Fire
   - **New Flow**: Start shooter → Wait for RPM → Fire preload from position 0 → Wait for firing delay → Advance spindexer to position 1
   - **Fix Applied**: Reordered the sequence in `isActionComplete()` to fire first, then advance.

### 3. ✅ **FIXED: Shooter Never Stops**
   - **Problem**: Shooter kept running after scoring.
   - **Fix Applied**: Shooter now stops after spindexer reaches next position (after scoring sequence completes).

### 4. ✅ **FIXED: Spindexer Initialization**
   - **Problem**: Spindexer position wasn't explicitly initialized.
   - **Fix Applied**: Added `spindexer.goToPosition(0)` in `init()` to ensure it starts at position 0 (preload position).

### 5. **State Machine Edge Cases**
   - **Problem**: No timeout protection if subsystems fail (e.g., shooter never reaches RPM).
   - **Status**: Already handled with timeout checks in `isActionComplete()`.

### 6. **Intake Timing**
   - **Problem**: Intake starts when path starts, but we might want to start intake slightly before reaching the intake zone.
   - **Status**: Current implementation is reasonable - intake starts when path to intake zone begins.

## Test Cases:

### Test Case 1: Preload Scoring
1. Robot starts at startPose
2. Spindexer initialized to position 0 (has preload)
3. Follows scorePreload path
4. When path completes:
   - Shooter starts spinning
   - Wait for shooter RPM (or timeout)
   - **Fire preload from position 0** (firing delay)
   - Advance spindexer to position 1
   - Wait for spindexer to reach position 1
   - Shooter stops
   - Move to intake

### Test Case 2: First Set Intake and Score
1. Move to intake position
2. Start intake while following intake path
3. Wait for intake duration
4. Move to scoring position
5. Score artifact from position 1:
   - Shooter starts spinning
   - Wait for shooter RPM
   - **Fire artifact from position 1** (firing delay)
   - Advance spindexer to position 2
   - Wait for spindexer to reach position 2
   - Shooter stops

### Test Case 3: Second Set Intake and Score
1. Move to second intake position
2. Start intake while following intake path
3. Wait for intake duration
4. Move to scoring position
5. Score artifact from position 2:
   - Shooter starts spinning
   - Wait for shooter RPM
   - **Fire artifact from position 2** (firing delay)
   - Advance spindexer to position 0 (wraps around)
   - Wait for spindexer to reach position 0
   - Shooter stops

## Expected State Transitions:

```
State 0 → State 1: Start preload path
State 1 → State 2: Path complete, start scoring preload
State 2 → State 3: Scoring complete, move to intake
State 3 → State 4: Path complete, start intake path and intake
State 4 → State 5: Intake complete, move to score
State 5 → State 6: Path complete, start scoring
State 6 → State 7: Scoring complete, move to second intake
State 7 → State 8: Path complete, start intake path and intake
State 8 → State 9: Intake complete, move to score
State 9 → State 10: Path complete, start scoring
State 10 → State -1: Scoring complete, finish
```

## Spindexer Position Tracking:

- Start: Position 0 (has preload)
- After preload: Position 1
- After first set: Position 2
- After second set: Position 0 (wraps around)

