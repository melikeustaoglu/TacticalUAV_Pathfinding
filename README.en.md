# Tactical UAV Pathfinding — Autonomy Simulation Prototype

*[Türkçe için buraya tıklayın](README.md)*

A Unity-based prototype simulating tactical UAV route planning and threat
avoidance under dynamic threats and uncertain sensor data. The project was
developed as part of an internship at Selçuk University Teknokent.

## Project Purpose

The system is not a single "find the shortest path" algorithm. The goal is
to simulate the full end-to-end chain found in a real autonomous system:

```
Sensor → State Estimation → Tracking → Threat Assessment → Replanning → PathFollower → Mission/Telemetry
```

Each link in this chain has been developed and tested individually.

## Architecture

| Layer | Responsibility | Main files |
|---|---|---|
| **Sensors** | GPS, IMU, Barometer, LiDAR, Radar simulation; Gaussian noise, sensor failure injection | `Assets/Scripts/Sensors/` |
| **State Estimation** | Position/velocity/heading estimation via Extended Kalman Filter, uncertainty (covariance) tracking | `Assets/Scripts/StateEstimation/` |
| **Tracking** | Multi-target tracking, sensor data association, track lifecycle | `Assets/Scripts/Tracking/` |
| **Threat Assessment** | TTC/CPA computation, uncertainty-aware threat scoring, multi-threat prioritization | `ThreatAssessment.cs` |
| **Pathfinding & Replanning** | A*-based route planning, Velocity Obstacle avoidance (3-stage: speed reduction → vertical evasion → spatial replan) | `Pathfinding.cs`, `ReplanningController.cs` |
| **Mission** | Mission state, score calculation, event logging | `MissionManager.cs`, `MissionScore.cs`, `MissionEventLogger.cs` |
| **Diagnostics** | In-scene 3D visualization (LiDAR/Radar/threats/EKF uncertainty) | `Assets/Scripts/Diagnostics/` |

## Testing and Validation

The project uses a two-tier test structure:

- **EditMode tests** (`Assets/Tests/EditMode/`, 46 files) — isolated
  validation of algorithm units: pathfinding, EKF, sensor fusion, threat
  assessment, multi-threat prioritization, GPS resilience.
- **PlayMode tests** (`Assets/Tests/PlayMode/`, 3 files) — runtime
  scenarios running in-scene.

There is also a benchmark infrastructure running through
`Assets/Editor/BenchmarkSuiteRunner.cs` and `BenchmarkReporter.cs`, along
with scenario assets defined under `Assets/Scenarios/` (dense obstacles,
dynamic threats, long range, 3D vertical climb, etc.).

**Note — an honest note on scope and limitations:**
Some benchmark scenarios are genuine end-to-end production scenarios; others
isolate the GPS/uncertainty math or the multi-threat logic using
**controlled injection** (e.g. instead of a continuous real-time GPS outage,
covariance/error is injected directly into the EKF to validate the
resulting behavior). This distinction is not hidden intentionally; which
scenario falls into which category can be understood from the test file
names and is spelled out in the final report.

## Out of Scope

- Testing on real physical UAV hardware (outside internship scope)
- Real GPS/IMU/LiDAR/Radar hardware — all sensors are simulated
- Long-duration, continuous real-time GPS outage benchmarking
- Real-time performance / CPU / frame-budget deployment metrics
- Bridging to real autonomy stacks such as ROS 2 / MAVLink / PX4

These items were deliberately left as "future work"; the architecture is
layered in a way that supports this expansion (e.g. the `ISensor` interface
allows the sensor source to be swapped for real hardware).

## Project Structure

```
Assets/
  Scripts/           Core autonomy code (sensors, state estimation, tracking, planning)
    Sensors/
    StateEstimation/
    Tracking/
    Diagnostics/
  Scenarios/          Benchmark and test scenario ScriptableObject assets
  Editor/              Benchmark runner (BenchmarkSuiteRunner)
  Tests/
    EditMode/          Unit and integration tests
    PlayMode/          Runtime/scene tests
```

## How to Run

1. Open the project with Unity Editor (Unity 6 / latest LTS recommended).
2. To run tests: `Window → General → Test Runner`, select the relevant
   tests from the EditMode and PlayMode tabs and run them.
3. To run a benchmark: use the Editor menu command exposed via
   `Assets/Editor/BenchmarkSuiteRunner.cs`; results are reported through
   `BenchmarkReporter`.
4. To try a scenario, pick one of the `.asset` files under
   `Assets/Scenarios/` and assign it to the relevant scene.

## Development Note

AI-assisted tools (including code generation and test writing) were used
during development. Architecture decisions and system design were directed
by the developer; generated code and tests were reviewed before being
integrated into the project.
