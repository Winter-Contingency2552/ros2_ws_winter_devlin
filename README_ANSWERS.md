Robotics Actions vs Services

1. When an Action Is Superior to a Service
**Example:** Navigating a mobile robot to a waypoint in a dynamic environment.  
**Why use an Action instead of a Service:**
- Long-running task: Execution may take many seconds or minutes.
- Streaming progress: Requires periodic feedback (pose, remaining distance, ETA).
- Interruptibility: Must allow cancelling or replacing the goal if obstacles, new higher-priority commands, or safety events occur.
- Services are synchronous (request/response only), lack mid-execution feedback, and have no standard preemption mechanism.

2. Action Components
- **Goal:** The target specification (e.g., pose, tolerances, constraints) sent by the client to start execution.
- **Feedback:** Periodic progress data (e.g., current pose, percent complete, remaining distance) enabling UI updates or adaptive decision logic.
- **Result:** Final outcome after completion or termination (success/failure status, final pose, path summary, error codes) for post-processing or logging.

3. Importance of Preemption (Cancellation)
Preempting a goal is crucial because it enables:
- **Safety:** Immediate stop if a hazard or human intervention occurs.
- **Responsiveness:** Switch to higher-priority tasks (emergency stop, new urgent goal).
- **Adaptation:** React to environmental changes or updated objectives.
- **Resource Efficiency:** Avoid wasting time, power, and compute on obsolete goals.

Summary
Actions extend beyond Services by supporting asynchronous, cancellable, feedback-driven execution—essential for real-world, dynamic robotic tasks.
