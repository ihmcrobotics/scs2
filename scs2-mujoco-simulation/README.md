# scs2-mujoco-simulation

MuJoCo physics backend for SCS2, alongside Bullet / ContactPointBased / ImpulseBased.

## Status

**v1 / scaffolded.** Java sources compile against a JavaCPP-generated `Mujoco.java` that is
produced by `just wrap` — the module is not buildable until you have run the install + wrap
pipeline below at least once.

Scope of v1 (per the design plan at
`~/.claude/plans/awesome-now-in-the-vivid-allen.md`):

- Headless only. No `mjr_*` rendering. SCS2's JavaFX visualizer renders the mecano state.
- Linux x86_64 only.
- Single composite MuJoCo world per `SimulationSession`. All robots and terrain must be added
  before the first `simulate()` call — MuJoCo does not support mid-run model edits in this build.
- URDF for robots, MJCF wrapper for world + terrain.

## Build pipeline

The native binding is built inside a Docker container, mirroring `ihmc-crocoddyl-wrapper`. From
this directory:

```bash
just docker     # one-time: builds the ubuntu:22.04 image with clang + Java 17
just install    # downloads the MuJoCo SDK release tarball, stages headers + libmujoco.so
just wrap       # runs JavaCPP: parses MujocoInfoMapper, generates Mujoco.java + libjniMujoco.so
```

`install.sh` pins `MUJOCO_VERSION=3.2.7` (released 2024-01-15). This is intentional, not
oversight: `MujocoInfoMapper.java` is a hand-maintained JavaCPP preset against that header
layout, and bumping the version requires re-validating the struct/API mapping against whatever
changed upstream before re-running `just wrap`.

After `just wrap` succeeds you should see:

```
src/main/generated-java/us/ihmc/scs2/simulation/mujoco/Mujoco.java
src/main/resources/mujoco/linux-x86_64/libjniMujoco.so
src/main/resources/mujoco/linux-x86_64/libmujoco.so.3.x.y
```

`./gradlew :scs2-mujoco-simulation:compileJava` then builds the Java side.

## Verification

1. **Native smoke test.** Once `just wrap` has produced the binding, run
   `./gradlew :scs2-mujoco-simulation:test --tests MujocoNativeSmokeTest`. This drops a sphere
   under gravity and asserts z decreases over 1000 steps.

2. **End-to-end with a real SCS2 example.** Port one of the SCS2 example simulations (e.g.
   `SphereAtRestExperimentalSimulation`) to use
   `MujocoPhysicsEngineFactory.newMujocoPhysicsEngineFactory()`. Compare trajectories against the
   existing Bullet / ImpulseBased engines for sanity.

## Layout

```
build.gradle.kts                                Gradle module wiring (mirrors scs2-bullet-simulation)
Dockerfile, justfile, install.sh, wrap.sh       Native build pipeline
src/main/java/us/ihmc/scs2/simulation/mujoco/
  preset/MujocoInfoMapper.java                  JavaCPP @Platform mapper for mujoco.h
  MujocoNativeLibrary.java                      Loads libjniMujoco.so + libmujoco.so
  physicsEngine/
    MujocoPhysicsEngine.java                    Implements scs2-simulation PhysicsEngine
    MujocoPhysicsEngineFactory.java             newMujocoPhysicsEngineFactory()
    MujocoMultiBodyDynamicsWorld.java           Owns mjModel*, mjData*
    MujocoRobot.java                            SCS2 RobotExtension wrapper
    MujocoMultiBodyRobot.java                   joint-name -> mjModel address map
    MujocoMultiBodyRobotFactory.java            RobotDefinition -> URDF + MJCF wrapper
    MujocoTerrainFactory.java                   TerrainObjectDefinition -> MJCF <geom>s
    MujocoTerrainObject.java
    MujocoTools.java                            transform <-> MJCF attribute strings
    parameters/MujocoSimulationParameters.java
    parameters/YoMujocoSimulationParameters.java
src/main/generated-java/.../Mujoco.java         (produced by `just wrap`)
src/main/resources/mujoco/linux-x86_64/         (libs produced by `just install` + `just wrap`)
src/test/java/.../MujocoNativeSmokeTest.java    Headless smoke test
```

## Known risks / TODO

- **MJCF `<include>` of URDF.** `MujocoMultiBodyRobotFactory.buildWorldMjcf` emits
  `<include file="robot.urdf"/>`. If MuJoCo treats the included file strictly as MJCF and refuses
  URDF, the fallback is direct MJCF generation from `RobotDefinition` (see the plan, ~500 extra
  lines). Verify with the smoke test once the binding is wrapped.
- **Mesh + convex polytope terrain** is not yet supported by `MujocoTerrainFactory`. v1 covers
  box / sphere / cylinder / capsule.
- **Multi-DoF non-root joints** (planar, spherical, cross-four-bar) are silently skipped by
  `MujocoRobot.pushStateToMujoco`. They need MJCF-side modeling and a corresponding state
  push/pull strategy.
- **Sensors and contact wrenches.** The `RobotPhysicsOutput` / `RigidBodyWrenchRegistry` plumbing
  used by `BulletRobot` is not yet replicated. Adding it requires reading MuJoCo's `mjData.contact`
  and `mjData.cfrc_ext` arrays per body and is the natural next step for closed-loop locomotion
  controllers that read external wrenches.
- **Rewindability** (the `SimulationRewindabilityTester` pattern) needs `mj_setState` /
  `mj_getState` exposure plus matching mecano state snapshotting.
- **Non-Linux platforms.** `MujocoNativeLibrary.getPackage` returns `unknown` for non-Linux x86_64;
  add macOS / Windows when needed.
```
