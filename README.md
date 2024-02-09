# ![SCS2](logo/SCS2.png)

![scs2](https://maven-badges.herokuapp.com/maven-central/us.ihmc/scs2-definition/badge.svg?style=plastic)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/bullet-simulation-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/definition-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/examples-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/session-logger-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/session-visualizer-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/session-visualizer-jfx-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/shared-memory-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/simulation-gradleCI.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/simulation-construction-set-2/actions/workflows/symbolic-gradleCI.yml/badge.svg)

# Install standalone application on Ubuntu 22.04 (Recommended)

1. Install Java 17: `sudo apt install openjdk-17-jdk`
2. Download the latest release (`scs2-[version].deb`) from the [releases page](https://github.com/ihmcrobotics/simulation-construction-set-2/releases).
3. Install the package:
    - Open your favorite terminal application (Ctrl+Alt+T)
    - `sudo dpkg -i scs2-[version].deb`
4. Shortly after the install completed, the desktop application should be available in the application menu as `SCS2 Session Visualizer` (`Super` key, then
   start looking up for SCS2).
5. If you want to run SCS2 from the command line, you can add the following line to your `~/.bashrc` file: `export PATH=$PATH:/opt/scs2-[version]/bin`.
    - You can then run SCS2 from the command line with: `SCS2SessionVisualizer`
    - The application accepts arguments, for instance:
        - `SCS2SessionVisualizer --help` to get the list of available arguments.
        - `SCS2SessionVisualizer -l /path/to/log/file` to load a log file.
        - `SCS2SessionVisualizer -l /path/to/log/file -r /path/to/robot/urdf/file` to load a log file and a robot definition file.

To uninstall SCS2, run: `sudo dpkg -r scs2`

# This is still an experimental version

This version is not bug-free and still misses lots of features.
Do not expect a fully functional application, however the main features should be available to make it usable.
If you'd like to help improving this application, please provide feedback on
the [GitHub issue page](https://github.com/ihmcrobotics/simulation-construction-set-2/issues).

Any type of feedback would be greatly appreciated, in particular:

- If the application throws an exception, the most helpful set of information is: strack-trace of the exception (available from the console) and the steps you
  proceeded through the GUI before getting the exception so it can be reproduced locally.
- Missing minor features: when it feels like there should be that one small option, for instance a button to reset a view or something, available to the user
  but it's not there. They're often easy to implement without involving large development and still make the application much better.
- Missing major features: provide context or a concrete example illustrating the need for a new major feature. A major feature is for instance: the ability to
  record videos (already in the roadmap) or the ability to do some post-processing on the YoVariables. It will very likely take a while to implement it, but may
  be so much worth it.
- GUI visual feel: Ugh that button is ugly... That background could be darker... Well if that helps making the GUI look nicer, everybody will enjoy it.
- GUI unexpected/frustrating behavior: When after hitting that button 30 times you're still expecting a different result and that is frustrating... If you can
  provide the details on what's unexpected/frustrating and what you would expect, that'll help make the GUI fell smoother to work with.

# GUI differences and limitations w.r.t. to SCS1

- Where is the search panel? See that hamburger control in the top left corner of the 3D view? There, click it.
- Where do you create empty charts? You see that small blue square with an inverted triangle in the top left corner of the chart area? There, click it.
- There's too few control buttons, where the others? See that "plus" button? There, click it.
- The navigation controls for the 3D view differ from SCS1, they are the same to our other JavaFX GUI applications though and all controls are available when
  using a touchpad instead of a mouse.
- Overhead plotter navigation controls differ from SCS1. Left click'n'drag: translate. Mouse wheel: zoom in/out. Note that all controls are available when using
  a touchpad instead of a mouse.
- The camera is not tracking the robot?? Right click on the graphic you want to track.
- Can I visualize a log? Go to the menu `Session > Start log session...`.
- Can connect to a remote session? Go to the menu `Session > Start remote session...`.

# Main new features compared to SCS1 Visualizer

## `YoComposite`

A `YoComposite` is a group of `YoVariable`s that represents a high-level of information.
The main advantage of `YoComposite`s is the ability to search for a specific type of composite which better filters searched, and also the ability to plot all
the components of the composite over one or more charts given a predefined configuration.

There is a default set of `YoComposite` defined in SCS2:

- `YoTuple2D` & `YoTuple3D`: represent respectively 2D & 3D points and/or vectors.
- `YoQuaternion`: represents orientations defined as quaternions.
- `YoYawPitchRoll`: represents orientations defined as Euler angles or yaw-pitch-roll angles.

To search for a certain `YoComposite`, open the search panel, to right of the search field, click that button with "..." and change the search target in the
dialog that pops up.

Custom `YoComposite`s can be defined view the menu _YoComposite > YoCompositePattern properties..._
Preferred chart configurations for each custom composite can also be defined there. You can export/import custom `YoComposite` to a file via _YoComposite >
Load/Save YoCompositePattern..._.

## `YoGraphic`

While `YoGraphic` were available in SCS1 by implementing them in Java, they are available from the GUI in SCS2.

`YoGraphic3D` represents the group of 3D graphics that will show up in the 3D view, while `YoGraphic2D` represents the group of 2D graphics that will show up in
the overhead plotter. The latter can be displayed via the menu _YoGraphic > Overhead Plotter_.
`YoGraphic`s can be created via the menu _YoGraphic > YoGraphic properties..._. You can export/import `YoGraphic` to a file via _YoComposite > Load/Save
YoGraphic..._.

# System properties:

- `scs2.home`: [String] Defines the home folder in which SCS2 is saving configuration files.
- `scs2.session.buffer.initialsize`: [Integer] Defines the default buffer size for any type of session.
- `scs2.session.buffer.recordtickperiod`: [Integer] Defines the default record frequency for any type of session.
- `scs2.session.runrealtime`: [Boolean] If defined, the simulation will not go faster than real-time.
- `scs2.session.playrealtime`: [Boolean] Defines the default real-time rate when playing back data for any type of session.
- `scs2.session.buffer.publishperiod`: [Long] Defines the default period in nanoseconds at which the GUI is refreshed for any type of session.
- `scs2.session.gui.mainwindow.loadconfig`: [Boolean] Determines whether or not the main window configuration (width, height, position, maximized or not) from
  the configuration file. Enabled by default, can be disabled if the behavior is not desired.
- `scs2.session.gui.buffersize.loadconfig`: [Boolean] Determines whether or not the buffer size should be initialized from the configuration file. Disabled by
  default, can be enabled if the behavior is desired.
- `scs2.session.gui.yovariable.enablefuzzysearch`: [Boolean] If a `YoVariable` cannot be found by name in the session registry, when loading configuration file,
  the fuzzy search can help retrieving it. This helps with variable rename or with variable that gets moved. Disabled by default. The search can be
  computationally expensive and is currently blocking the rendering thread.
- `scs2.session.gui.skybox.theme`: [String] Allows changing skybox theme. 3 options: `CLOUDY_CROWN_MIDDAY` (default), `SCS1`, and `CUSTOM`. When `CUSTOM` is
  set, the path to the skybox image file(s) is to be provided as well.
- `scs2.session.gui.skybox.custompath`: [String] Defines the path to load a custom skybox, not that the skybox theme is to be set to `CUSTOM` for this property
  to be used. The path should lead to either a single image file that contains the 6 panes to use as the skybox (looks like an unfolded box), or lead to a
  directory that contains 6 image files named: Top, Bottom, Left, Right, Front, and Back and which file extension can be either `*.png`, `*jpg`, or `*.bmp`.
- `scs2.session.gui.loadconfig.synchronous`: [Boolean] Modifies the behavior for initializing a session:
    - `true` best attempt is made at initializing sequentially, increasing initialization time and improving predictability of when the GUI is fully
      initialized. This is expected to improved the accuracy of `SessionVisualizerControls.waitUntilVisualizerFullyUp()`.
    - `false` initialization is done in parallel, decreasing initialization time and impairing the ability to predict when the GUI is fully initialized.
- `scs2.session.gui.loadconfig.time`: [Boolean] Whether to print the time taken for initialization.

# Environment variables:

- `SCS2_HOME`: [String] Defines the home folder in which SCS2 is saving configuration files.
- `SCS2_SKYBOX_THEME`: [String] Allows changing skybox theme. 3 options: `CLOUDY_CROWN_MIDDAY` (default), `SCS1`, and `CUSTOM`. When `CUSTOM` is set, the path
  to the skybox image file(s) is to be provided as well.
- `SCS2_SKYBOX_CUSTOM_PATH`: [String] Defines the path to load a custom skybox, not that the skybox theme is to be set to `CUSTOM` for this property to be used.
  The path should lead to either a single image file that contains the 6 panes to use as the skybox (looks like an unfolded box), or lead to a directory that
  contains 6 image files named: Top, Bottom, Left, Right, Front, and Back and which file extension can be either `*.png`, `*jpg`, or `*.bmp`.
- `SCS2_GUI_LOAD_SESSION_SYNCHRONOUS`: [Boolean] Modifies the behavior for initializing a session. See `scs2.session.gui.loadconfig.synchronous` above.

# Useful Tools classes:

- `URDFTools`: for creating a `RobotDefinition` from a URDF file.
- `SDFTools`: for creating a `RobotDefinition` from a SDF file.
- `DefinitionIOTools`: for writing/reading definitions into/from XML file.
- `SimMultiBodySystemTools`: common operations with system composed of `SimRigidBodyBasics` and `SimJointBasics` which are used to define the robot in
  simulation.
- `SharedMemoryIOTools`: for writing a `SharedBuffer` into a data file and reading a `SharedBuffer` from a data file.
- `JavaFXMissingTools`: provides additional tools for working with JavaFX.
- `YoGraphicConversionTools` (in `ihmc-graphics-description`): for converting SCS1 `YoGraphic` and `Artifact` into SCS2 `YoGraphicDefinition`.
- `SegmentedLine3DTriangleMeshFactory`: for creating a 3D "noodle" that goes through 3D waypoint.
- `VisualDefinitionFactory`: for creating `VisualDefinition`.
- `YoGraphicDefinitionFactory`: for creating `YoGraphicDefinition`.

# Repository Projects:

- `scs2-definition`: This project gathers classes to help define: robots, terrain environment, yoGraphics, visuals, and so on. It is meant to be a low-level
  project with low overhead in terms of dependencies such that it can easily be imported in other low-level projects.
- `scs2-shared-memory`: This project provides an implementation for a yoVariable buffer. This buffer allows storing yoVariable values over time which in turn
  allows playing back simulation data for instance. This project is meant to remain rather low-level in terms of dependency and limit the scope to yoVariables.
- `scs2-session`: This project defines the base implementation of a session. A session is an abstract base layer for defining a simulation session, log session,
  or remote session.
- `scs2-simulation`: This project provides the implementation for the simulation backend as well as 2 physics engines: contact point based physics engine (SCS1
  like) and an impulse based physics engine.
- `scs2-bullet-simulation`: This project provides a new physics engine that is a bridge to Bullet.
- `scs2-session-logger`: This project provides the backend for log session and remote session.
- `scs2-session-visualizer`: This project provides tools for the SCS2 GUI that are graphics engine agnostic.
- `scs2-session-visualizer-jfx`: This project provides a JavaFX implementation of the SCS2 GUI.
- `scs2-simulation-construction-set`: This project provides the class `SimulationConstructionSet2` which bundles backend and frontend of SCS2 in a similar
  fashion as `SimulationConstructionSet` in SCS1.

# Known issues:

See [Wiki - Known Issues](https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/Known-Issues-and-workaround).

# External references:

## Icons:

- <img src="scs2-session-visualizer-jfx/src/main/resources/icons/valid-icon.png" alt="Valid icon" width="25"/> & <img src="scs2-session-visualizer-jfx/src/main/resources/icons/invalid-icon.png" alt="Invalid icon" width="25"/> icons from <a href="https://icons8.com/icons/">Icons8</a>

## Skybox:

- <img src="scs2-session-visualizer-jfx/src/main/resources/skybox/cloudy/Front.png" alt="Skybox" width="50"/> default skybox from <a href="https://assetstore.unity.com/packages/2d/textures-materials/sky/farland-skies-cloudy-crown-60004">Cloudy Crown Skybox</a>



