# Requirements Evaluation Report

## A. Modular PID Controller ⚠️ **PARTIALLY SATISFIED**

### Current Implementation:
- ✅ Basic PID class exists in `modules/controllers.py`
- ✅ Has Kp, Ki, Kd parameters
- ✅ Implements standard PID algorithm

### Missing Features:
- ❌ **No reset functionality** - cannot reset integral/state between runs
- ❌ **No integral windup protection** - integral can grow unbounded
- ❌ **No output limits** - PID output is only clipped externally
- ❌ **No anti-windup mechanism** - could cause overshoot
- ❌ **Limited modularity** - no methods for tuning, logging, or state inspection

### Recommendation:
The PID controller is functional but needs enhancement for production use. Should add:
- `reset()` method
- Output saturation limits
- Integral windup protection
- Better encapsulation

---

## B. Generate Desired Car Velocity ⚠️ **NEEDS INVESTIGATION**

### Current Implementation:
- ✅ Uses `VELOCITY_CONTROL` mode on rear wheel joints
- ✅ Configurable target velocity and force via speed profiles
- ✅ Gradual acceleration ramp-up implemented

### Potential Issues:
- ⚠️ **No verification** that velocity control actually works
- ⚠️ **No debugging output** to confirm wheel velocities match targets
- ⚠️ **Force values seem inconsistent** - SLOW profile has higher force (52382) than NORMAL (7726)
- ⚠️ **No joint damping settings** - wheels might not respond correctly
- ⚠️ **No verification** of actual car velocity vs target

### Recommendation:
**REQUIRES TESTING**: The code structure suggests velocity control should work, but:
1. Need to verify wheel joint types support VELOCITY_CONTROL
2. Need to check if force values are appropriate
3. Need to add debugging to log actual vs target velocities
4. May need to adjust joint damping or use TORQUE_CONTROL instead

---

## C. Constrain Cube Inside Vehicle ❌ **NOT SATISFIED**

### Current Implementation:
- ✅ Cube position is calculated in car's local frame
- ✅ X-axis limit enforced via `np.clip()` (limit_x: 0.25)
- ✅ Cube position updated using `local_to_world()` transformation

### Critical Issues:
- ❌ **No physical constraint** - cube uses `resetBasePositionAndOrientation()` which teleports it
- ❌ **Cube is NOT attached to car** - no joint or constraint created
- ❌ **No Y-axis limits** - cube can move sideways (currently fixed at 0)
- ❌ **No Z-axis limits** - cube Z is hardcoded to 0.2
- ❌ **Cube can escape** - if limits fail, cube could detach from car
- ❌ **No collision with car interior** - cube doesn't interact with car body

### Recommendation:
**CRITICAL FIX NEEDED**: Should implement:
1. Create a **slider joint** or **prismatic constraint** between cube and car
2. Constrain cube to move only along X-axis in car's local frame
3. Set joint limits to enforce bounds
4. Add Y and Z axis bounds checking
5. Ensure cube mass is tuned (currently 30% of car mass - may need adjustment)

---

## D. Implement Cube Position Control Function ⚠️ **PARTIALLY SATISFIED**

### Current Implementation:
- ✅ Cube position is updated relative to car
- ✅ Uses local coordinate system
- ✅ Enforces X-axis limits

### Missing Features:
- ❌ **No dedicated function** - logic is inline in `main.py` (lines 236-251)
- ❌ **No encapsulation** - should be a method of CarJumpEnv or separate class
- ❌ **No validation** of car dimensions - limits don't check if cube fits
- ❌ **Hardcoded Y and Z** - only X is controlled, Y=0, Z=0.2 fixed
- ❌ **No bounds checking** against actual car interior dimensions

### Recommendation:
**SHOULD BE REFACTORED**: Create a dedicated function like:
```python
def set_cube_position(car, cube, local_pos, limits):
    # Validate bounds
    # Transform to world
    # Update position
```

---

## E. Landing Evaluation ❌ **NOT SATISFIED**

### Current Implementation:
- ✅ `hasLanded()` function detects when car contacts ground after launch
- ✅ Tracks task states (ascend, launched, landed)

### Missing Metrics:
- ❌ **No uprightness check** - doesn't measure pitch/roll angles at landing
- ❌ **No velocity measurement** - doesn't record impact velocity
- ❌ **No stability check** - doesn't verify if car remains stable after landing
- ❌ **No success/failure criteria** - only detects landing, doesn't evaluate quality
- ❌ **No metrics logging** - no data saved about landing performance

### Recommendation:
**MAJOR ENHANCEMENT NEEDED**: Should implement:
1. **Uprightness metric**: `abs(pitch) < threshold` and `abs(roll) < threshold`
2. **Impact velocity**: Measure velocity magnitude at landing moment
3. **Stability check**: Verify car doesn't tip over after landing (e.g., for 1-2 seconds)
4. **Success criteria**: Define what constitutes a "good" landing
5. **Metrics logging**: Save landing data to file for analysis

Example metrics:
- Landing pitch angle (degrees)
- Landing roll angle (degrees)
- Impact velocity (m/s)
- Stability duration (seconds)
- Success/failure boolean

---

## Summary

| Requirement | Status | Priority |
|------------|--------|----------|
| A. Modular PID Controller | ⚠️ Partial | Medium |
| B. Generate Desired Car Velocity | ⚠️ Needs Testing | High |
| C. Constrain Cube Inside Vehicle | ❌ Not Satisfied | **CRITICAL** |
| D. Cube Position Control Function | ⚠️ Partial | Medium |
| E. Landing Evaluation | ❌ Not Satisfied | High |

### Critical Issues to Address:
1. **Cube constraint** (C) - Cube is not physically attached to car
2. **Velocity control verification** (B) - Need to verify it actually works
3. **Landing metrics** (E) - No evaluation of landing quality

### Overall Assessment:
The project has a **working prototype** but needs significant improvements to meet all requirements, especially:
- Physical constraints for the cube
- Comprehensive landing evaluation metrics
- Better modularity and encapsulation

