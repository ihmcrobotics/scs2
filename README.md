# ![SCS2](logo/SCS2.png)
[ ![scs2](https://maven-badges.herokuapp.com/maven-central/us.ihmc/scs2-definition/badge.svg?style=plastic)](https://maven-badges.herokuapp.com/maven-central/us.ihmc/scs2-definition)
[ ![buildstatus](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-SIMULATIONCONSTRUCTIONSET2)](https://bamboo.ihmc.us/plugins/servlet/wittified/build-status/LIBS-SIMULATIONCONSTRUCTIONSET2)

# This is still an experimental version
This version is not bug-free and still misses lots of features.
Do not expect a fully functional application, however the main features should be available to make it usable.
If you'd like to help improving this application, please provide feedback on the [GitHub issue page](https://github.com/ihmcrobotics/simulation-construction-set-2/issues).

Any type of feedback would be greatly appreciated, in particular:
- If the application throws an exception, the most helpful set of information is: strack-trace of the exception (available from the console) and the steps you proceeded through the GUI before getting the exception so it can be reproduced locally.
- Missing minor features: when it feels like there should be that one small option, for instance a button to reset a view or something, available to the user but it's not there. They're often easy to implement without involving large development and still make the application much better.
- Missing major features: provide context or a concrete example illustrating the need for a new major feature. A major feature is for instance: the ability to record videos (already in the roadmap) or the ability to do some post-processing on the YoVariables. It will very likely take a while to implement it, but may be so much worth it.
- GUI visual feel: Ugh that button is ugly... That background could be darker... Well if that helps making the GUI look nicer, everybody will enjoy it.
- GUI unexpected/frustrating behavior: When after hitting that button 30 times you're still expecting a different result and that is frustrating... If you can provide the details on what's unexpected/frustrating and what you would expect, that'll help make the GUI fell smoother to work with.

# GUI differences and limitations w.r.t. to SCS1
- Where is the search panel? See that hamburger control in the top left corner of the 3D view? There, click it.
- Where do you create empty charts? You see that small blue square with an inverted triangle in the top left corner of the chart area? There, click it.
- There's too few control buttons, where the others? See that "plus" button? There, click it.
- The navigation controls for the 3D view differ from SCS1, they are the same to our other JavaFX GUI applications though and all controls are available when using a touchpad instead of a mouse.
- Overhead plotter navigation controls differ from SCS1. Left click'n'drag: translate. Mouse wheel: zoom in/out. Note that all controls are available when using a touchpad instead of a mouse.
- The camera is not tracking the robot?? Right click on the graphic you want to track.
- Can I visualize a log? Go to the menu `Session > Start log session...`.
- Can connect to a remote session? Go to the menu `Session > Start remote session...`.

# Main new features compared to SCS1 Visualizer
## `YoComposite`
A `YoComposite` is a group of `YoVariable`s that represents a high-level of information.
The main advantage of `YoComposite`s is the ability to search for a specific type of composite which better filters searched, and also the ability to plot all the components of the composite over one or more charts given a predefined configuration.

There is a default set of `YoComposite` defined in SCS2:
- `YoTuple2D` & `YoTuple3D`: represent respectively 2D & 3D points and/or vectors.
- `YoQuaternion`: represents orientations defined as quaternions.
- `YoYawPitchRoll`: represents orientations defined as Euler angles or yaw-pitch-roll angles.

To search for a certain `YoComposite`, open the search panel, to right of the search field, click that button with "..." and change the search target in the dialog that pops up.

Custom `YoComposite`s can be defined view the menu _YoComposite > YoCompositePattern properties..._
Preferred chart configurations for each custom composite can also be defined there. You can export/import custom `YoComposite` to a file via _YoComposite > Load/Save YoCompositePattern..._.

## `YoGraphic`
While `YoGraphic` were available in SCS1 by implementing them in Java, they are available from the GUI in SCS2.

`YoGraphic3D` represents the group of 3D graphics that will show up in the 3D view, while `YoGraphic2D` represents the group of 2D graphics that will show up in the overhead plotter. The latter can be displayed via the menu _YoGraphic > Overhead Plotter_.
`YoGraphic`s can be created via the menu _YoGraphic > YoGraphic properties..._. You can export/import `YoGraphic` to a file via _YoComposite > Load/Save YoGraphic..._.

# System properties:
- `scs2.home`: Defines the home folder in which SCS2 is saving configuration files.
- `scs2.session.buffer.initialsize`: Defines the default buffer size for any type of session.
- `scs2.session.buffer.recordtickperiod`: Defines the default record frequency for any type of session.
- `scs2.session.runrealtime`: If defined, the simulation will not go faster than real-time.
- `scs2.session.playrealtime`: Defines the default real-time rate when playing back data for any type of session.
- `scs2.session.buffer.publishperiod`: Defines the default rate at which the GUI is refreshed for any type of session.
- `scs2.session.gui.mainwindow.loadconfig`: Determines whether or not the main window configuration (width, height, position, maximized or not) from the configuration file. Enabled by default, can be disabled if the behavior is not desired.
- `scs2.session.gui.mainwindow.loadconfig`: Determines whether or not the main window configuration (width, height, position, maximized or not) from the configuration file. Enabled by default, can be disabled if the behavior is not desired.
- `scs2.session.gui.yovariable.enablefuzzysearch`: If a `YoVariable` cannot be found by name in the session registry, when loading configuration file, the fuzzy search can help retrieving it. This helps with variable rename or with variable that gets moved. Disabled by default. The search can be computationally expensive and is currently blocking the rendering thread.
- `scs2.session.gui.skybox.theme`: Allows changing skybox theme. 3 options: `CLOUDY_CROWN_MIDDAY` (default), `SCS1`, and `CUSTOM`. When `CUSTOM` is set, the path to the skybox image file(s) is to be provided as well.
- `scs2.session.gui.skybox.custompath`: Defines the path to load a custom skybox, not that the skybox theme is to be set to `CUSTOM` for this property to be used. The path should lead to either a single image file that contains the 6 panes to use as the skybox (looks like an unfolded box), or lead to a directory that contains 6 image files named: Top, Bottom, Left, Right, Front, and Back and which file extension can be either `*.png`, `*jpg`, or `*.bmp`.

# Environment variables:
- `SCS2_HOME`: Defines the home folder in which SCS2 is saving configuration files.
- `SCS2_SKYBOX_THEME`: Allows changing skybox theme. 3 options: `CLOUDY_CROWN_MIDDAY` (default), `SCS1`, and `CUSTOM`. When `CUSTOM` is set, the path to the skybox image file(s) is to be provided as well.
- `SCS2_SKYBOX_CUSTOM_PATH`: Defines the path to load a custom skybox, not that the skybox theme is to be set to `CUSTOM` for this property to be used. The path should lead to either a single image file that contains the 6 panes to use as the skybox (looks like an unfolded box), or lead to a directory that contains 6 image files named: Top, Bottom, Left, Right, Front, and Back and which file extension can be either `*.png`, `*jpg`, or `*.bmp`.

# Useful Tools classes:
- `URDFTools`: for creating a `RobotDefinition` from a URDF file.
- `SDFTools`: for creating a `RobotDefinition` from a SDF file.
- `DefinitionIOTools`: for writing/reading definitions into/from XML file.
- `SimMultiBodySystemTools`: common operations with system composed of `SimRigidBodyBasics` and `SimJointBasics` which are used to define the robot in simulation.
- `SharedMemoryIOTools`: for writing a `SharedBuffer` into a data file and reading a `SharedBuffer` from a data file.
- `JavaFXMissingTools`: provides additional tools for working with JavaFX.
- `SCS1GraphicConversionTools`: for converting SCS1 `YoGraphic` and `Artifact` into SCS2 `YoGraphicDefinition`.
- `SegmentedLine3DTriangleMeshFactory`: for creating a 3D "noodle" that goes through 3D waypoint.
- `VisualDefinitionFactory`: for creating `VisualDefinition`.
- `YoGraphicDefinitionFactory`: for creating `YoGraphicDefinition`.

# External references:
## Icons:
- <img src="scs2-session-visualizer-jfx/src/main/resources/icons/valid-icon.png" alt="Valid icon" width="25"/> & <img src="scs2-session-visualizer-jfx/src/main/resources/icons/invalid-icon.png" alt="Invalid icon" width="25"/> icons from <a href="https://icons8.com/icons/">Icons8</a>

## Skybox:
- <img src="scs2-session-visualizer-jfx/src/main/resources/skybox/cloudy/Front.png" alt="Skybox" width="50"/> default skybox from <a href="https://assetstore.unity.com/packages/2d/textures-materials/sky/farland-skies-cloudy-crown-60004">Cloudy Crown Skybox</a>



