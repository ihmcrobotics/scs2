# scs2-mujoco-simulation

MuJoCo physics backend for SCS2, alongside Bullet / ContactPointBased / ImpulseBased.

## Status

Java sources compile against a JavaCPP-generated `Mujoco.java`. The generated bindings and
compiled native libraries (`libjniMujoco.so` / `mujoco.dll` and dependencies) are committed
directly under `src/main/generated-java` and `src/main/resources`, so CI (`test-fast`) runs the
module's tests against them as-is -- no native build step required -- see
`.github/workflows/gradleCI-base.yml`. The `./build.sh install && ./build.sh wrap` pipeline below
is only needed when regenerating the bindings (e.g. bumping the MuJoCo version).

Scope:

- Headless only. No `mjr_*` rendering. SCS2's JavaFX visualizer renders the mecano state.
- Linux x86_64 and Windows x86_64.
- Single composite MuJoCo world per `SimulationSession`. All robots and terrain must be added
  before the first `simulate()` call -- MuJoCo does not support mid-run model edits in this build.
- URDF for robots, MJCF wrapper for world + terrain.

## Build pipeline

Only needed to regenerate the committed bindings (e.g. bumping the MuJoCo version) -- not part of
the normal build or CI run. The native binding is built inside a Docker container, mirroring
`ihmc-crocoddyl-wrapper`. From this directory:

```bash
./build.sh docker     # one-time: builds the ubuntu:22.04 image with clang + Java 17
./build.sh install    # downloads the MuJoCo SDK release tarball, stages headers + libmujoco.so
./build.sh wrap       # runs JavaCPP: parses MujocoInfoMapper, generates Mujoco.java + libjniMujoco.so
```

`install.sh` and `install.ps1` each pin `MUJOCO_VERSION`, and that pin is the single source of
truth for the version. `MujocoInfoMapper.java` is a hand-maintained JavaCPP preset against that
header layout, so bumping the pin requires re-validating the struct/API mapping against whatever
changed upstream before re-running the wrap step.

### Bumping the MuJoCo version

**The halves have to move together.** Each platform carries two binaries -- the upstream library
(`libmujoco.so` / `mujoco.dll`) and the JavaCPP JNI shim (`libjniMujoco.so` / `jniMujoco.dll`) --
and the shim is only valid against the exact headers it was compiled from. The committed
`Mujoco.java` has to match as well; its `mjVERSION_HEADER` records the version it came from.

Updating one without the others is the one mistake worth calling out, because it does **not** fail
loudly. MuJoCo keeps its exported symbols stable across minor versions, so a mismatched pair links
and loads fine, and only then does the shim read structs at the wrong field offsets. That is
silently wrong physics rather than an error.

#### Build them in CI (preferred)

Run the **MuJoCo natives** workflow (`.github/workflows/mujoco-natives.yml`) from the Actions tab.
It builds every platform in one dispatch -- Linux through the same Docker image used above, Windows
on a hosted `windows-2022` runner with MSVC -- so no Windows machine or local Docker is needed, and
both platforms come out of the same inputs.

It reads the version from the pin, so there is nothing to pass; the optional input only asserts the
pin is what you expect. Each platform's job checks that the regenerated bindings report the
matching `mjVERSION_HEADER` and that its shim exports JNI entry points at all, and then a final job
diffs the two shims' native method names against each other -- the check that actually catches one
platform having been built against different headers.

Download the artifacts, drop each platform's files into `src/main/resources/mujoco/<platform>/`, and
commit them together with `Mujoco.java`. The workflow commits nothing itself, so the binaries get
looked at before they land.

One manual step on Linux: `libmujoco.so` in the tree is a symlink to the versioned library, and
build artifacts cannot carry symlinks, so the workflow ships only the real files. Recreate the link
after dropping them in:

```bash
cd src/main/resources/mujoco/linux-x86_64
ln -sf libmujoco.so.<version> libmujoco.so
```

#### Or build Windows on a Windows machine

From a *Visual Studio Developer PowerShell* (so `cl.exe` is on PATH), in this directory:

```powershell
.\native-build\install.ps1   # downloads the Windows SDK, stages headers + mujoco.lib, copies mujoco.dll
.\native-build\wrap.ps1      # runs JavaCPP with MSVC: regenerates Mujoco.java + jniMujoco.dll
```

Neither script can run on Linux: `wrap.ps1` requires MSVC `cl.exe` and passes MSVC-only flags
(`/O2 /std:c++17 /EHsc /MD`, `LINK=/DLL`). Run both, in that order.

After `./build.sh wrap` succeeds you should see:

```
src/main/generated-java/us/ihmc/scs2/simulation/mujoco/Mujoco.java
src/main/resources/mujoco/linux-x86_64/libjniMujoco.so
src/main/resources/mujoco/linux-x86_64/libmujoco.so.3.x.y
```

`./gradlew :scs2-mujoco-simulation:compileJava` then builds the Java side.

## Verification

1. **Native smoke test.** Run
   `./gradlew :scs2-mujoco-simulation:test --tests MujocoNativeSmokeTest` against the committed
   bindings (or a freshly regenerated one via `./build.sh wrap`). This drops a sphere under
   gravity and asserts z decreases over 1000 steps.

2. **End-to-end with a real SCS2 example.** Port one of the SCS2 example simulations (e.g.
   `SphereAtRestExperimentalSimulation`) to construct a `PhysicsEngineFactory` lambda wrapping
   `new MujocoPhysicsEngine(frame, rootRegistry, parameters)`. Compare trajectories against the
   existing Bullet / ImpulseBased engines for sanity.

## Layout

```
build.gradle.kts                                Gradle module wiring (mirrors scs2-bullet-simulation)
build.sh                                        `./build.sh docker|install|wrap|clear` entry points
native-build/                                   Dockerfile, install.sh/.ps1, wrap.sh/.ps1
src/main/java/us/ihmc/scs2/simulation/mujoco/
  preset/MujocoInfoMapper.java                  JavaCPP @Platform mapper for mujoco.h
  MujocoNativeLibrary.java                      Loads libjniMujoco.so + libmujoco.so
  physicsEngine/
    MujocoPhysicsEngine.java                    Implements scs2-simulation PhysicsEngine
    MujocoMultiBodyDynamicsWorld.java           Owns mjModel*, mjData*
    MujocoRobot.java                            SCS2 RobotExtension wrapper
    MujocoMultiBodyRobot.java                   joint-name -> mjModel address map
    MujocoMultiBodyRobotFactory.java            RobotDefinition -> URDF + MJCF wrapper
    MujocoTerrainFactory.java                   TerrainObjectDefinition -> MJCF <geom>s
    MujocoTerrainObject.java
    MujocoTools.java                            transform <-> MJCF attribute strings
    parameters/MujocoSimulationParameters.java
    parameters/YoMujocoSimulationParameters.java
src/main/generated-java/.../Mujoco.java         (produced by `./build.sh wrap`)
src/main/resources/mujoco/linux-x86_64/         (libs produced by `./build.sh install` + `./build.sh wrap`)
src/test/java/.../MujocoNativeSmokeTest.java    Headless smoke test
```

## Known risks / TODO

- **MJCF `<include>` of URDF.** `MujocoMultiBodyRobotFactory.buildWorldMjcf` emits
  `<include file="robot.urdf"/>`. If MuJoCo treats the included file strictly as MJCF and refuses
  URDF, the fallback is direct MJCF generation from `RobotDefinition`. Verify with the smoke test
  if MJCF parsing changes upstream.
- **Mesh + convex polytope terrain** is not yet supported by `MujocoTerrainFactory`. Currently
  covers box / sphere / cylinder / capsule.
- **Multi-DoF non-root joints** (planar, spherical, cross-four-bar) are silently skipped by
  `MujocoRobot.pushStateToMujoco`. They need MJCF-side modeling and a corresponding state
  push/pull strategy.
- **Sensors and contact wrenches.** The `RobotPhysicsOutput` / `RigidBodyWrenchRegistry` plumbing
  used by `BulletRobot` is not yet replicated. Adding it requires reading MuJoCo's `mjData.contact`
  and `mjData.cfrc_ext` arrays per body and is the natural next step for closed-loop locomotion
  controllers that read external wrenches.
- **Rewindability** (the `SimulationRewindabilityTester` pattern) needs `mj_setState` /
  `mj_getState` exposure plus matching mecano state snapshotting.
- **macOS** is not supported; `MujocoNativeLibrary.getPackage` returns `unknown` for it.
```
