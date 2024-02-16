package us.ihmc.scs2.sessionVisualizer.jfx;

import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.Pane;
import javafx.stage.Window;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.camera.YoLevelOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.camera.YoOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardType;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoMultiSliderboardWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoBooleanProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoIntegerProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoLongProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.xml.bind.JAXBException;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.Collection;
import java.util.Collections;
import java.util.function.Predicate;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoMultiSliderboardWindowController.DEFAULT_SLIDERBOARD_NAME;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoMultiSliderboardWindowController.DEFAULT_SLIDERBOARD_TYPE;

public interface SessionVisualizerControls
{

   /**
    * Sets the camera's orbit with respect to the focal point.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    *
    * @param latitude  controls the look up/down angle while keeping the focal point unchanged.
    * @param longitude controls the look left/right angle while keeping the focal point unchanged.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void setCameraOrientation(double latitude, double longitude);

   /**
    * Convenience methods to set the camera position.
    *
    * @param position the new camera position. Not modified.
    * @see #setCameraPosition(double, double, double)
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void setCameraPosition(Point3DReadOnly position)
   {
      setCameraPosition(position.getX(), position.getY(), position.getZ());
   }

   /**
    * Sets the camera position without moving the focal point.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    *
    * @param x the new x-coordinate for the camera position.
    * @param y the new y-coordinate for the camera position.
    * @param z the new z-coordinate for the camera position.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void setCameraPosition(double x, double y, double z);

   /** @deprecated Use {@link #setCameraFocalPosition(Point3DReadOnly)} instead. */
   @Deprecated
   default void setCameraFocusPosition(Point3DReadOnly position)
   {
      setCameraFocalPosition(position);
   }

   /** @deprecated Use {@link #setCameraFocalPosition(double, double, double)} instead. */
   @Deprecated
   default void setCameraFocusPosition(double x, double y, double z)
   {
      setCameraFocalPosition(x, y, z);
   }

   /**
    * Convenience methods to set the camera position.
    *
    * @param position the new focal position. Not modified.
    * @see #setCameraFocalPosition(double, double, double)
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void setCameraFocalPosition(Point3DReadOnly position)
   {
      setCameraFocalPosition(position.getX(), position.getY(), position.getZ());
   }

   /**
    * Sets the position of the focal point, i.e. what the camera is looking at.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    *
    * @param x the new x-coordinate for the focal point.
    * @param y the new y-coordinate for the focal point.
    * @param z the new z-coordinate for the focal point.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void setCameraFocalPosition(double x, double y, double z);

   /**
    * Sets the distance between the camera and focal point by moving the camera only.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    *
    * @param distanceFromFocus the new distance between the camera and the focal point.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void setCameraZoom(double distanceFromFocus);

   /**
    * Requests the camera's focal point to track the rigid-body of a robot.
    *
    * @param robotName     the name of the robot to track.
    * @param rigidBodyName the name of the body to track.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void requestCameraRigidBodyTracking(String robotName, String rigidBodyName);

   /**
    * Requests the camera's focal point to track the given coordinates.
    *
    * @param coordinatesToTrack the coordinates to track.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraFocalPositionTracking(YoFrameTuple3D coordinatesToTrack)
   {
      requestCameraFocalPositionTracking(coordinatesToTrack.getYoX(),
                                         coordinatesToTrack.getYoY(),
                                         coordinatesToTrack.getYoZ(),
                                         coordinatesToTrack.getReferenceFrame());
   }

   /**
    * Requests the camera's focal point to track the given coordinates.
    *
    * @param xCoordinateToTrack the x coordinate to track.
    * @param yCoordinateToTrack the y coordinate to track.
    * @param zCoordinateToTrack the z coordinate to track.
    * @param referenceFrame     the reference frame in which the coordinates are expressed.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraFocalPositionTracking(YoDouble xCoordinateToTrack,
                                                   YoDouble yCoordinateToTrack,
                                                   YoDouble zCoordinateToTrack,
                                                   ReferenceFrame referenceFrame)
   {
      requestCameraFocalPositionTracking(xCoordinateToTrack == null ? null : xCoordinateToTrack.getFullNameString(),
                                         yCoordinateToTrack == null ? null : yCoordinateToTrack.getFullNameString(),
                                         zCoordinateToTrack == null ? null : zCoordinateToTrack.getFullNameString(),
                                         referenceFrame == null ? null : referenceFrame.getNameId());
   }

   /**
    * Requests the camera's focal point to track the given coordinates.
    *
    * @param xCoordinateNameToTrack the name (fullname or short name) of the {@code YoVariable} to
    *                               track for the x coordinate. Can be a double value.
    * @param yCoordinateNameToTrack the name (fullname or short name) of the {@code YoVariable} to
    *                               track for the y coordinate. Can be a double value.
    * @param zCoordinateNameToTrack the name (fullname or short name) of the {@code YoVariable} to
    *                               track for the z coordinate. Can be a double value.
    * @param referenceFrameName     the name of the reference frame in which the coordinates are
    *                               expressed. If {@code null}, world frame is used.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraFocalPositionTracking(String xCoordinateNameToTrack,
                                                   String yCoordinateNameToTrack,
                                                   String zCoordinateNameToTrack,
                                                   String referenceFrameName)
   {
      requestCameraFocalPositionTracking(new YoTuple3DDefinition(xCoordinateNameToTrack, yCoordinateNameToTrack, zCoordinateNameToTrack, referenceFrameName));
   }

   /**
    * Requests the camera's focal point to track the given coordinates.
    *
    * @param coordinatesToTrack the coordinates to track.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void requestCameraFocalPositionTracking(YoTuple3DDefinition coordinatesToTrack);

   // Camera coordinates control:
   // Cartesian control:

   /**
    * Requests the camera to follow the given cartesian coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, such that if the focal point is tracking a node or
    * coordinate, the camera will remain at the same location, thus rotating to follow to the focal
    * point.
    * </p>
    *
    * @param cameraCoordinates the cartesian coordinates of the camera.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraPositionTracking(YoTuple3D cameraCoordinates)
   {
      requestCameraPositionTracking(cameraCoordinates.getYoX(), cameraCoordinates.getYoY(), cameraCoordinates.getYoZ());
   }

   /**
    * Requests the camera to follow the given cartesian coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, such that if the focal point is tracking a node or
    * coordinate, the camera will remain at the same location, thus rotating to follow to the focal
    * point.
    * </p>
    *
    * @param xCameraCoordinate the x coordinate of the camera.
    * @param yCameraCoordinate the y coordinate of the camera.
    * @param zCameraCoordinate the z coordinate of the camera.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraPositionTracking(YoDouble xCameraCoordinate, YoDouble yCameraCoordinate, YoDouble zCameraCoordinate)
   {
      requestCameraPositionTracking(xCameraCoordinate == null ? null : xCameraCoordinate.getFullNameString(),
                                    yCameraCoordinate == null ? null : yCameraCoordinate.getFullNameString(),
                                    zCameraCoordinate == null ? null : zCameraCoordinate.getFullNameString());
   }

   /**
    * Requests the camera to follow the given cartesian coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, such that if the focal point is tracking a node or
    * coordinate, the camera will remain at the same location, thus rotating to follow to the focal
    * point.
    * </p>
    *
    * @param xCameraCoordinateName the name (fullname or short name) of the {@code YoVariable} for the
    *                              x coordinate of the camera. Can be a double value.
    * @param yCameraCoordinateName the name (fullname or short name) of the {@code YoVariable} for the
    *                              y coordinate of the camera. Can be a double value.
    * @param zCameraCoordinateName the name (fullname or short name) of the {@code YoVariable} for the
    *                              z coordinate of the camera. Can be a double value.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraPositionTracking(String xCameraCoordinateName, String yCameraCoordinateName, String zCameraCoordinateName)
   {
      requestCameraPositionTracking(new YoTuple3DDefinition(xCameraCoordinateName, yCameraCoordinateName, zCameraCoordinateName));
   }

   /**
    * Requests the camera to follow the given cartesian coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, such that if the focal point is tracking a node or
    * coordinate, the camera will remain at the same location, thus rotating to follow to the focal
    * point.
    * </p>
    *
    * @param cameraCoordinates the cartesian coordinates of the camera.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void requestCameraPositionTracking(YoTuple3DDefinition cameraCoordinates);

   // Orbital control:

   /**
    * Requests the camera to follow the given orbital coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, such that if the focal point is tracking a node or
    * coordinate, the camera will be translating by the same amount as the focal point preserving the
    * orientation of the camera.
    * </p>
    *
    * @param distanceCameraCoordinate  the distance coordinate of the camera, range ]0, +&infin;[.
    *                                  Controls how far the camera is from the focal point.
    * @param longitudeCameraCoordinate the longitude coordinate of the camera, range [-<i>pi</i>,
    *                                  <i>pi</i>[. Controls the longitude/yaw orientation of the
    *                                  camera, at 0 the camera is pointing x+, at <i>pi</i>/2 it points
    *                                  y+, at <i>pi</i> towards x-, and at -<i>pi</i>/2 it points y-.
    * @param latitudeCameraCoordinate  the latitude coordinate of the camera, range [-<i>pi</i>/2,
    *                                  <i>pi</i>/2]. Controls the latitude/pitch of the camera, at
    *                                  -<i>pi</i>/2 the camera is under the focal point looking upward,
    *                                  at <i>pi</i>/2 the camera is above the focal point looking
    *                                  downward.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraOrbitTracking(YoDouble distanceCameraCoordinate, YoDouble longitudeCameraCoordinate, YoDouble latitudeCameraCoordinate)
   {
      requestCameraOrbitTracking(distanceCameraCoordinate == null ? null : distanceCameraCoordinate.getFullNameString(),
                                 longitudeCameraCoordinate == null ? null : longitudeCameraCoordinate.getFullNameString(),
                                 latitudeCameraCoordinate == null ? null : latitudeCameraCoordinate.getFullNameString());
   }

   /**
    * Requests the camera to follow the given orbital coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, such that if the focal point is tracking a node or
    * coordinate, the camera will be translating by the same amount as the focal point preserving the
    * orientation of the camera.
    * </p>
    *
    * @param distanceCameraCoordinateName  the name (fullname or short name) of the {@code YoVariable}
    *                                      for the distance coordinate of the camera, range ]0,
    *                                      +&infin;[. Can be a double value. Controls how far the
    *                                      camera is from the focal point.
    * @param longitudeCameraCoordinateName the name (fullname or short name) of the {@code YoVariable}
    *                                      for the longitude coordinate of the camera, range
    *                                      [-<i>pi</i>, <i>pi</i>[. Can be a double value. Controls the
    *                                      longitude/yaw orientation of the camera, at 0 the camera is
    *                                      pointing x+, at <i>pi</i>/2 it points y+, at <i>pi</i>
    *                                      towards x-, and at -<i>pi</i>/2 it points y-.
    * @param latitudeCameraCoordinateName  the name (fullname or short name) of the {@code YoVariable}
    *                                      for the latitude coordinate of the camera, range
    *                                      [-<i>pi</i>/2, <i>pi</i>/2]. Can be a double value. Controls
    *                                      the latitude/pitch of the camera, at -<i>pi</i>/2 the camera
    *                                      is under the focal point looking upward, at <i>pi</i>/2 the
    *                                      camera is above the focal point looking downward.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraOrbitTracking(String distanceCameraCoordinateName, String longitudeCameraCoordinateName, String latitudeCameraCoordinateName)
   {
      requestCameraOrbitTracking(new YoOrbitalCoordinateDefinition(distanceCameraCoordinateName, longitudeCameraCoordinateName, latitudeCameraCoordinateName));
   }

   /**
    * Requests the camera to follow the given orbital coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, such that if the focal point is tracking a node or
    * coordinate, the camera will be translating by the same amount as the focal point preserving the
    * orientation of the camera.
    * </p>
    *
    * @param cameraCoordinates the camera's coordinates.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void requestCameraOrbitTracking(YoOrbitalCoordinateDefinition cameraCoordinates);

   // Level-Orbital control:

   /**
    * Requests the camera to follow the given level-orbital coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, this is an hybrid mode between cartesian and orbital
    * control. On the horizontal plane, the camera translates together with the focal point.
    * Vertically, the focal point translation doesn't affect the camera position, instead the camera
    * pitches to follow the focal point.
    * </p>
    *
    * @param distanceCameraCoordinate  the distance coordinate of the camera, range ]0, +&infin;[.
    *                                  Controls how far the camera is from the focal point.
    * @param longitudeCameraCoordinate the longitude coordinate of the camera, range [-<i>pi</i>,
    *                                  <i>pi</i>[. Controls the longitude/yaw orientation of the
    *                                  camera, at 0 the camera is pointing x+, at <i>pi</i>/2 it points
    *                                  y+, at <i>pi</i> towards x-, and at -<i>pi</i>/2 it points y-.
    * @param heightCameraCoordinate    the z coordinate of the camera. Controls the height of the
    *                                  camera.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraLevelOrbitTracking(YoDouble distanceCameraCoordinate, YoDouble longitudeCameraCoordinate, YoDouble heightCameraCoordinate)
   {
      requestCameraLevelOrbitTracking(distanceCameraCoordinate == null ? null : distanceCameraCoordinate.getFullNameString(),
                                      longitudeCameraCoordinate == null ? null : longitudeCameraCoordinate.getFullNameString(),
                                      heightCameraCoordinate == null ? null : heightCameraCoordinate.getFullNameString());
   }

   /**
    * Requests the camera to follow the given level-orbital coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, this is an hybrid mode between cartesian and orbital
    * control. On the horizontal plane, the camera translates together with the focal point.
    * Vertically, the focal point translation doesn't affect the camera position, instead the camera
    * pitches to follow the focal point.
    * </p>
    *
    * @param distanceCameraCoordinateName  the name (fullname or short name) of the {@code YoVariable}
    *                                      for the distance coordinate of the camera, range ]0,
    *                                      +&infin;[. Can be a double value. Controls how far the
    *                                      camera is from the focal point.
    * @param longitudeCameraCoordinateName the name (fullname or short name) of the {@code YoVariable}
    *                                      for the longitude coordinate of the camera, range
    *                                      [-<i>pi</i>, <i>pi</i>[. Can be a double value. Controls the
    *                                      longitude/yaw orientation of the camera, at 0 the camera is
    *                                      pointing x+, at <i>pi</i>/2 it points y+, at <i>pi</i>
    *                                      towards x-, and at -<i>pi</i>/2 it points y-.
    * @param heightCameraCoordinateName    the name (fullname or short name) of the {@code YoVariable}
    *                                      for the height coordinate of the camera. Can be a double
    *                                      value.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   default void requestCameraLevelOrbitTracking(String distanceCameraCoordinateName, String longitudeCameraCoordinateName, String heightCameraCoordinateName)
   {
      requestCameraLevelOrbitTracking(new YoLevelOrbitalCoordinateDefinition(distanceCameraCoordinateName,
                                                                             longitudeCameraCoordinateName,
                                                                             heightCameraCoordinateName));
   }

   /**
    * Requests the camera to follow the given level-orbital coordinates expressed in world.
    * <p>
    * This changes the control mode of the camera, this is an hybrid mode between cartesian and orbital
    * control. On the horizontal plane, the camera translates together with the focal point.
    * Vertically, the focal point translation doesn't affect the camera position, instead the camera
    * pitches to follow the focal point.
    * </p>
    *
    * @param cameraCoordinates the camera's coordinates.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/3D-viewport-navigation">GUI
    *       controls: 3D viewport navigation</a>
    */
   void requestCameraLevelOrbitTracking(YoLevelOrbitalCoordinateDefinition cameraCoordinates);

   // End of section for: Camera coordinates control

   /**
    * Requests to show the overhead view next to the 3D viewport where 2D graphics are displayed.
    *
    * @param show whether the plotter 2D should be visible or not.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/2D-plotter-navigation">GUI
    *       controls: 2D plotter navigation</a>
    */
   void showOverheadPlotter2D(boolean show);

   /**
    * Requests the plotter to track the {@code YoVariable} coordinates.
    *
    * @param position the position to track.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/2D-plotter-navigation">GUI
    *       controls: 2D plotter navigation</a>
    */
   default void requestPlotter2DCoordinateTracking(YoFrameTuple2D position)
   {
      requestPlotter2DCoordinateTracking(position, position.getReferenceFrame().getNameId());
   }

   /**
    * Requests the plotter to track the {@code YoVariable} coordinates.
    *
    * @param position  the position to track.
    * @param frameName the name of the reference frame in which the coordinates are expressed. If
    *                  {@code null}, world frame is used.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/2D-plotter-navigation">GUI
    *       controls: 2D plotter navigation</a>
    */
   default void requestPlotter2DCoordinateTracking(YoTuple2D position, String frameName)
   {
      requestPlotter2DCoordinateTracking(position.getYoX(), position.getYoY(), frameName);
   }

   /**
    * Requests the plotter to track the {@code YoVariable} coordinates.
    *
    * @param xVariable the {@code YoVariable} to track for the x-coordinate.
    * @param yVariable the {@code YoVariable} to track for the y-coordinate.
    * @param frameName the name of the reference frame in which the coordinates are expressed. If
    *                  {@code null}, world frame is used.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/2D-plotter-navigation">GUI
    *       controls: 2D plotter navigation</a>
    */
   default void requestPlotter2DCoordinateTracking(YoDouble xVariable, YoDouble yVariable, String frameName)
   {
      requestPlotter2DCoordinateTracking(xVariable.getFullNameString(), yVariable.getFullNameString(), frameName);
   }

   /**
    * Requests the plotter to track the {@code YoVariable} coordinates.
    *
    * @param xVariableName the name (fullname or short name) of the {@code YoVariable} to track. Can be
    *                      a double value.
    * @param yVariableName the name (fullname or short name) of the {@code YoVariable} to track. Can be
    *                      a double value.
    * @param frameName     the name of the reference frame in which the coordinates are expressed. If
    *                      {@code null}, world frame is used.
    * @see <a href=
    *       "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/2D-plotter-navigation">GUI
    *       controls: 2D plotter navigation</a>
    */
   void requestPlotter2DCoordinateTracking(String xVariableName, String yVariableName, String frameName);

   /**
    * Adds a static graphic to the 3D scene.
    *
    * @param visualDefinition the visual to be added to the 3D scene.
    * @see VisualDefinitionFactory
    */
   void addStaticVisual(VisualDefinition visualDefinition);

   /**
    * Adds a collection of static graphic to the 3D scene.
    *
    * @param visualDefinitions the collection of visuals to be added to the 3D scene.
    * @see VisualDefinitionFactory
    */
   default void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         addStaticVisual(visualDefinition);
      }
   }

   /**
    * Removes a static graphic that was previously added via
    * {@link #addStaticVisual(VisualDefinition)}.
    *
    * @param visualDefinition the visual to remove from the 3D scene.
    * @see VisualDefinitionFactory
    */
   void removeStaticVisual(VisualDefinition visualDefinition);

   /**
    * Removes a collection of static graphics that were previously added via
    * {@link #addStaticVisual(VisualDefinition)}.
    *
    * @param visualDefinitions the visuals to remove from the 3D scene.
    */
   default void removeStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         removeStaticVisual(visualDefinition);
      }
   }

   /**
    * Removes all yoGraphics that were added to the GUI.
    */
   default void removeAllYoGraphics()
   {
      removeYoGraphic(YoGraphicTools.GUI_ROOT_NAME);
   }

   /**
    * Finds the first discovered instance of a yoGraphic matching the given name and removes it from
    * the scene.
    * <p>
    * If the yoGraphic found is a group, it is removed as well as all of its children and descendants.
    * </p>
    *
    * @param name the name of the yoGraphic to remove. If the name contains
    *             {@value YoGraphicDefinition#SEPARATOR}, it is split at the last occurrence to extract
    *             a namespace and the actual yoGraphic name.
    */
   void removeYoGraphic(String name);

   /**
    * Sets the visible property for all yoGraphics that were added to the GUI.
    */
   default void setAllYoGraphicsVisible(boolean visible)
   {
      setYoGraphicVisible(YoGraphicTools.GUI_ROOT_NAME, visible);
   }

   /**
    * Finds the first discovered instance of a yoGraphic matching the given name and sets its visible
    * property.
    * <p>
    * If the yoGraphic found is a group, it is sets the visible property for all of its children and
    * descendants as well.
    * </p>
    *
    * @param name the name of the yoGraphic to remove. If the name contains
    *             {@value YoGraphicDefinition#SEPARATOR}, it is split at the last occurrence to extract
    *             a namespace and the actual yoGraphic name.
    */
   void setYoGraphicVisible(String name, boolean visible);

   /**
    * Adds a dynamic graphic to the 3D scene. The new graphic is added to root group.
    *
    * @param yoGraphicDefinition the definition of the graphic to be added.
    * @see YoGraphicConversionTools
    * @see YoGraphicDefinitionFactory
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/YoGraphic">GUI
    *       controls: YoGraphic</a>
    */
   void addYoGraphic(YoGraphicDefinition yoGraphicDefinition);

   /**
    * Adds dynamic graphics to the 3D scene. The new graphics are added to root group.
    *
    * @param yoGraphicDefinitions the definitions of the graphics to be added.
    * @see YoGraphicConversionTools
    * @see YoGraphicDefinitionFactory
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/YoGraphic">GUI
    *       controls: YoGraphic</a>
    */
   default void addYoGraphics(Collection<? extends YoGraphicDefinition> yoGraphicDefinitions)
   {
      for (YoGraphicDefinition yoGraphicDefinition : yoGraphicDefinitions)
      {
         addYoGraphic(yoGraphicDefinition);
      }
   }

   /**
    * Adds a dynamic graphic to the 3D scene.
    *
    * @param namespace           the desired namespace for the new graphic. The separator used is
    *                            {@value YoGraphicDefinition#SEPARATOR}.
    * @param yoGraphicDefinition the definition of the graphic to be added.
    * @see YoGraphicConversionTools
    * @see YoGraphicDefinitionFactory
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/YoGraphic">GUI
    *       controls: YoGraphic</a>
    */
   default void addYoGraphic(String namespace, YoGraphicDefinition yoGraphicDefinition)
   {
      String[] subNames = namespace.split(YoGraphicDefinition.SEPARATOR);
      if (subNames == null || subNames.length == 0)
      {
         addYoGraphic(yoGraphicDefinition);
      }
      else
      {
         for (int i = subNames.length - 1; i >= 0; i--)
         {
            yoGraphicDefinition = new YoGraphicGroupDefinition(subNames[i], yoGraphicDefinition);
         }

         addYoGraphic(yoGraphicDefinition);
      }
   }

   /**
    * @deprecated Use {@link #addYoGraphics(String, Collection)} instead.
    */
   @Deprecated
   default void addYoGraphic(String namespace, Collection<? extends YoGraphicDefinition> yoGraphicDefinitions)
   {
      addYoGraphics(namespace, yoGraphicDefinitions);
   }

   /**
    * Adds dynamic graphics to the 3D scene.
    *
    * @param namespace            the desired namespace for the new graphics. The separator used is
    *                             {@value YoGraphicDefinition#SEPARATOR}.
    * @param yoGraphicDefinitions the definitions of the graphics to be added.
    * @see YoGraphicConversionTools
    * @see YoGraphicDefinitionFactory
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/YoGraphic">GUI
    *       controls: YoGraphic</a>
    */
   default void addYoGraphics(String namespace, Collection<? extends YoGraphicDefinition> yoGraphicDefinitions)
   {
      String[] subNames = namespace.split(YoGraphicDefinition.SEPARATOR);
      if (subNames == null || subNames.length == 0)
      {
         addYoGraphics(yoGraphicDefinitions);
      }
      else
      {
         YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(subNames[subNames.length - 1], yoGraphicDefinitions);

         for (int i = subNames.length - 2; i >= 0; i--)
         {
            group = new YoGraphicGroupDefinition(subNames[i], group);
         }

         addYoGraphic(group);
      }
   }

   /**
    * Adds a variable entry to the default entry tab.
    *
    * @param variableName the name of the variable to add. The variable will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/Side-pane"GUI
    *       controls: Side pane</a>
    */
   default void addYoEntry(String variableName)
   {
      addYoEntry(Collections.singletonList(variableName));
   }

   /**
    * Adds variable entries to the default entry tab.
    *
    * @param variableNames the name of the variables to add. The variables will be looked up using
    *                      {@link YoRegistry#findVariable(String)}.
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/Side-pane"GUI
    *       controls: Side pane</a>
    */
   default void addYoEntry(Collection<String> variableNames)
   {
      addYoEntry(null, variableNames);
   }

   /**
    * Adds a variable entry to the entry tab named {@code groupName}. The tab will be created if it
    * doesn't exist yet.
    *
    * @param groupName    the name of the tab.
    * @param variableName the name of the variable to add. The variable will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/Side-pane"GUI
    *       controls: Side pane</a>
    */
   default void addYoEntry(String groupName, String variableName)
   {
      addYoEntry(groupName, Collections.singletonList(variableName));
   }

   /**
    * Adds variable entries to the entry tab named {@code groupName}. The tab will be created if it
    * doesn't exist yet.
    *
    * @param groupName     the name of the tab.
    * @param variableNames the name of the variables to add. The variables will be looked up using
    *                      {@link YoRegistry#findVariable(String)}.
    * @see <a href="https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/Side-pane"GUI
    *       controls: Side pane</a>
    */
   void addYoEntry(String groupName, Collection<String> variableNames);

   /**
    * Request the visualizer to perform a force update of the charts.
    */
   void requestChartsForceUpdate();

   /**
    * Loads a sliderboard configuration from an input stream and overrides the current configuration.
    *
    * @param inputStream the input stream used to load the sliderboard configuration.
    * @see FileInputStream
    */
   default void loadSliderboards(InputStream inputStream)
   {
      try
      {
         setSliderboards(XMLTools.loadYoSliderboardListDefinition(inputStream));
      }
      catch (JAXBException | IOException e)
      {
         LogTools.error("Failed to load sliderboard configuration: {}, {}", e.getClass().getSimpleName(), e.getMessage());
      }
   }

   /**
    * Overrides the current sliderboard configuration with the given list of sliderboards.
    *
    * @param sliderboardListDefinition the list of sliderboard configurations to use.
    */
   void setSliderboards(YoSliderboardListDefinition sliderboardListDefinition);

   /**
    * Clears the current sliderboard configuration.
    * <p>
    * After calling this method, only the default sliderboard remains and it is empty.
    * </p>
    */
   void clearAllSliderboards();

   /**
    * Sets the configuration of the default sliderboard.
    * <p>
    * Note that the default sliderboard named
    * {@value YoMultiSliderboardWindowController#DEFAULT_SLIDERBOARD_NAME} always exists even after
    * clearing all sliderboards, in such case it is only cleared of its bindings.
    * </p>
    *
    * @param sliderboardDefinition the new configuration for the default sliderboard.
    */
   default void setDefaultSliderboard(YoSliderboardDefinition sliderboardDefinition)
   {
      sliderboardDefinition.setName(DEFAULT_SLIDERBOARD_NAME);
      setSliderboard(sliderboardDefinition);
   }

   /**
    * Clears the default sliderboard configuration.
    */
   default void clearDefaultSliderboard()
   {
      removeSliderboard(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE);
   }

   /**
    * Sets a sliderboard configuration.
    * <p>
    * The sliderboards are organized by name, i.e. {@link YoSliderboardDefinition#getName()}. If the
    * name of the given configuration does not match any existing sliderboard, a new sliderboard is
    * created and configured, otherwise the configuration of the matching sliderboard is overridden to
    * the given one.
    * </p>
    * <p>
    * Note that the default sliderboard named
    * {@value YoMultiSliderboardWindowController#DEFAULT_SLIDERBOARD_NAME} always exists even after
    * clearing all sliderboards. If the given configuration is named
    * {@value YoMultiSliderboardWindowController#DEFAULT_SLIDERBOARD_NAME}, then the default
    * sliderboard configuration overridden.
    * </p>
    *
    * @param sliderboardDefinition the sliderboard configuration to set or add, depending if there is
    *                              an existing sliderboard matching the name or not.
    */
   void setSliderboard(YoSliderboardDefinition sliderboardConfiguration);

   /**
    * Removes the sliderboard that matches the given name.
    * <p>
    * If there is no existing sliderboard matching the given name, nothing happens. If the given name
    * is {@value YoMultiSliderboardWindowController#DEFAULT_SLIDERBOARD_NAME}, then the default
    * sliderboard is cleared but not removed.
    * </p>
    *
    * @param sliderboardName the name of the sliderboard to remove.
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    */
   void removeSliderboard(String sliderboardName, YoSliderboardType sliderboardType);

   /**
    * Configures a button of the default sliderboard.
    * <p>
    * If the button was already configured, then its configuration is overridden.
    * </p>
    *
    * @param buttonIndex  the position of the button on the sliderboard. The first button has an index
    *                     of 0.
    * @param variableName the name of the {@code YoVariable} the button should be bound to. The
    *                     {@code YoVariable} has to be a {@code YoBoolean}, if not, this method has no
    *                     effect. The name can either be the fullname (including namespace) or the
    *                     simple name of the variable.
    */
   default void setDefaultSliderboardButton(int buttonIndex, String variableName)
   {
      setSliderboardButton(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, new YoButtonDefinition(variableName, buttonIndex));
   }

   /**
    * Configures a button of the default sliderboard.
    * <p>
    * If the button was already configured, then its configuration is overridden.
    * </p>
    *
    * @param buttonDefinition the new configuration for the button.
    */
   default void setDefaultSliderboardButton(YoButtonDefinition buttonDefinition)
   {
      setSliderboardButton(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, buttonDefinition);
   }

   /**
    * Configures a button of a sliderboard.
    *
    * @param sliderboardName used to retrieve the sliderboard for which the button is to be configured.
    *                        If no sliderboard could be found, a new empty sliderboard is created and
    *                        added to the current list of available sliderboards.
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param buttonIndex     the position of the button on the sliderboard. The first button has an
    *                        index of 0.
    * @param variableName    the name of the {@code YoVariable} the button should be bound to. The
    *                        {@code YoVariable} has to be a {@code YoBoolean}, if not, this method has
    *                        no effect. The name can either be the fullname (including namespace) or
    *                        the simple name of the variable.
    */
   default void setSliderboardButton(String sliderboardName, YoSliderboardType sliderboardType, int buttonIndex, String variableName)
   {
      setSliderboardButton(sliderboardName, sliderboardType, new YoButtonDefinition(variableName, buttonIndex));
   }

   /**
    * Configures a button of a sliderboard.
    *
    * @param sliderboardName  used to retrieve the sliderboard for which the button is to be
    *                         configured. If no sliderboard could be found, a new empty sliderboard is
    *                         created and added to the current list of available sliderboards.
    * @param sliderboardType  the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                         YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param buttonDefinition the new configuration for the button.
    */
   void setSliderboardButton(String sliderboardName, YoSliderboardType sliderboardType, YoButtonDefinition buttonDefinition);

   /**
    * Clears the configuration of a button of the default sliderboard.
    *
    * @param buttonIndex the position of the button on the sliderboard. The first button has an index
    *                    of 0.
    */
   default void clearDefaultSliderboardButton(int buttonIndex)
   {
      clearSliderboardButton(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, buttonIndex);
   }

   /**
    * Clears the configuration of a button.
    *
    * @param sliderboardName used to retrieve the sliderboard for which the button is to be cleared. If
    *                        no sliderboard could be found, this method has no effect.
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param buttonIndex     the position of the button on the sliderboard. The first button has an
    *                        index of 0.
    */
   void clearSliderboardButton(String sliderboardName, YoSliderboardType sliderboardType, int buttonIndex);

   /**
    * Configures a knob of the default sliderboard.
    * <p>
    * If the knob was already configured, then its configuration is overridden.
    * </p>
    *
    * @param knobIndex    the position of the knob on the sliderboard. The first knob has an index of
    *                     0.
    * @param variableName the name of the {@code YoVariable} the knob should be bound to. The name can
    *                     either be the fullname (including namespace) or the simple name of the
    *                     variable.
    */
   default void setDefaultSliderboardKnob(int knobIndex, String variableName)
   {
      setSliderboardKnob(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, new YoKnobDefinition(variableName, knobIndex));
   }

   /**
    * Configures a knob of the default sliderboard.
    * <p>
    * If the knob was already configured, then its configuration is overridden.
    * </p>
    *
    * @param knobDefinition the new configuration for the knob.
    */
   default void setDefaultSliderboardKnob(YoKnobDefinition knobDefinition)
   {
      setSliderboardKnob(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, knobDefinition);
   }

   /**
    * Configures a knob of a sliderboard.
    *
    * @param sliderboardName used to retrieve the sliderboard for which the knob is to be configured.
    *                        If no sliderboard could be found, a new empty sliderboard is created and
    *                        added to the current list of available sliderboards.
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param knobIndex       the position of the knob on the sliderboard. The first knob has an index
    *                        of 0.
    * @param variableName    the name of the {@code YoVariable} the knob should be bound to. The name
    *                        can either be the fullname (including namespace) or the simple name of the
    *                        variable.
    */
   default void setSliderboardKnob(String sliderboardName, YoSliderboardType sliderboardType, int knobIndex, String variableName)
   {
      setSliderboardKnob(sliderboardName, sliderboardType, new YoKnobDefinition(variableName, knobIndex));
   }

   /**
    * Configures a knob of a sliderboard.
    *
    * @param sliderboardName used to retrieve the sliderboard for which the knob is to be configured.
    *                        If no sliderboard could be found, a new empty sliderboard is created and
    *                        added to the current list of available sliderboards.
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param knobDefinition  the new configuration for the knob.
    */
   void setSliderboardKnob(String sliderboardName, YoSliderboardType sliderboardType, YoKnobDefinition knobDefinition);

   /**
    * Clears the configuration of a knob of the default sliderboard.
    *
    * @param knobIndex the position of the knob on the sliderboard. The first knob has an index of 0.
    */
   default void clearDefaultSliderboardKnob(int knobIndex)
   {
      clearSliderboardKnob(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, knobIndex);
   }

   /**
    * Clears the configuration of a knob.
    *
    * @param sliderboardName used to retrieve the sliderboard for which the knob is to be cleared. If
    *                        no sliderboard could be found, this method has no effect
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param knobIndex       the position of the knob on the sliderboard. The first knob has an index
    *                        of 0.
    */
   void clearSliderboardKnob(String sliderboardName, YoSliderboardType sliderboardType, int knobIndex);

   /**
    * Configures a slider of the default sliderboard.
    * <p>
    * If the slider was already configured, then its configuration is overridden.
    * </p>
    *
    * @param sliderIndex  the position of the slider on the sliderboard. The first slider has an index
    *                     of 0.
    * @param variableName the name of the {@code YoVariable} the slider should be bound to. The name
    *                     can either be the fullname (including namespace) or the simple name of the
    *                     variable.
    */
   default void setDefaultSliderboardSlider(int sliderIndex, String variableName)
   {
      setSliderboardSlider(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, new YoSliderDefinition(variableName, sliderIndex));
   }

   /**
    * Configures a slider of the default sliderboard.
    * <p>
    * If the slider was already configured, then its configuration is overridden.
    * </p>
    *
    * @param sliderDefinition the new configuration for the slider.
    */
   default void setDefaultSliderboardSlider(YoSliderDefinition sliderDefinition)
   {
      setSliderboardSlider(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, sliderDefinition);
   }

   /**
    * Configures a slider of a sliderboard.
    *
    * @param sliderboardName used to retrieve the sliderboard for which the slider is to be configured.
    *                        If no sliderboard could be found, a new empty sliderboard is created and
    *                        added to the current list of available sliderboards.
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param sliderIndex     the position of the slider on the sliderboard. The first slider has an
    *                        index of 0.
    * @param variableName    the name of the {@code YoVariable} the slider should be bound to. The name
    *                        can either be the fullname (including namespace) or the simple name of the
    *                        variable.
    */
   default void setSliderboardSlider(String sliderboardName, YoSliderboardType sliderboardType, int sliderIndex, String variableName)
   {
      setSliderboardSlider(sliderboardName, sliderboardType, new YoSliderDefinition(variableName, sliderIndex));
   }

   /**
    * Configures a slider of a sliderboard.
    *
    * @param sliderboardName  used to retrieve the sliderboard for which the slider is to be
    *                         configured. If no sliderboard could be found, a new empty sliderboard is
    *                         created and added to the current list of available sliderboards.
    * @param sliderboardType  the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                         YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param sliderDefinition the new configuration for the slider.
    */
   void setSliderboardSlider(String sliderboardName, YoSliderboardType sliderboardType, YoSliderDefinition sliderDefinition);

   default void clearDefaultSliderboardSlider(int sliderIndex)
   {
      clearSliderboardSlider(DEFAULT_SLIDERBOARD_NAME, DEFAULT_SLIDERBOARD_TYPE, sliderIndex);
   }

   /**
    * Clears the configuration of a slider of the default sliderboard.
    *
    * @param sliderboardType the type of the sliderboard (YoSliderBoardDefintion.BFC2000 or
    *                        YoSliderBoardDefinition.XTOUCHCOMPACT)
    * @param sliderIndex     the position of the slider on the sliderboard. The first slider has an
    *                        index of 0.
    */
   void clearSliderboardSlider(String sliderboardName, YoSliderboardType sliderboardType, int sliderIndex);

   /**
    * Registers a custom filter that can then be accessed when exporting session data via the GUI.
    *
    * @param name           the name of the custom filter for easy retrieval.
    * @param variableFilter the filter to be applied on the {@code YoVariable}. Only the variables for
    *                       which this filter returns {@code true} will be exported.
    */
   default void addSessionDataFilterParameters(String name, Predicate<YoVariable> variableFilter)
   {
      addSessionDataFilterParameters(new SessionDataFilterParameters(name, variableFilter, null));
   }

   /**
    * Registers a custom filter that can then be accessed when exporting session data via the GUI.
    *
    * @param filterParameters the custom filter to add to this session.
    */
   void addSessionDataFilterParameters(SessionDataFilterParameters filterParameters);

   /**
    * Captures a video of the 3D scene from the playback data.
    * <p>
    * The file extension should be {@value SessionVisualizerIOTools#videoFileExtension}.
    * </p>
    *
    * @param file the target file where the video is to be written.
    */
   default void exportVideo(File file)
   {
      SceneVideoRecordingRequest request = new SceneVideoRecordingRequest();
      request.setFile(file);
      exportVideo(request);
   }

   /**
    * Captures a video of the 3D scene from the playback data.
    *
    * @param request the request.
    */
   void exportVideo(SceneVideoRecordingRequest request);

   /**
    * Disables GUI controls. Can be used to prevent the user from interfering with a background process
    * temporarily.
    */
   void disableGUIControls();

   /**
    * Enables GUI controls.
    */
   void enableGUIControls();

   /**
    * Gets the main window's instance.
    *
    * @return the main window.
    */
   Window getPrimaryGUIWindow();

   /**
    * Adds a custom JavaFX control, for instance a {@link Button}, which is displayed in the user side
    * panel on the right side of the main window.
    *
    * @param control the custom control to add.
    */
   void addCustomGUIControl(Node control);

   /**
    * Removes a custom JavaFX control that was previously added via {@link #addCustomGUIControl(Node)}.
    *
    * @param control the control to be removed.
    * @return whether the control was found and removed successfully.
    */
   boolean removeCustomGUIControl(Node control);

   /**
    * Loads and adds a mini-GUI from an FXML file. The GUI is displayed in the user side panel on the
    * right side of the main window.
    *
    * @param name         the title of the new pane.
    * @param fxmlResource the locator to the FXML resource.
    */
   void loadCustomGUIPane(String name, URL fxmlResource);

   /**
    * Adds a mini-GUI to the user side panel on the right side of the main window.
    *
    * @param name the title of the new pane.
    * @param pane the pane to be added.
    */
   void addCustomGUIPane(String name, Pane pane);

   /**
    * Removes a pane previously added via {@link #loadCustomGUIPane(String, URL)} or
    * {@link #addCustomGUIPane(String, Pane)}.
    *
    * @param name the title of the pane to remove.
    */
   boolean removeCustomGUIPane(String name);

   /**
    * Creates a property that is bound to the {@code YoDouble} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoDoubleProperty#get()} and to safely submit a new value via
    * {@link YoDoubleProperty#set(double)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    *
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *       variable doesn't match, or there is no active session.
    */
   YoDoubleProperty newYoDoubleProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoInteger} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoIntegerProperty#get()} and to safely submit a new value via
    * {@link YoIntegerProperty#set(int)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    *
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *       variable doesn't match, or there is no active session.
    */
   YoIntegerProperty newYoIntegerProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoLong} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoLongProperty#get()} and to safely submit a new value via
    * {@link YoLongProperty#set(long)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    *
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *       variable doesn't match, or there is no active session.
    */
   YoLongProperty newYoLongProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoBoolean} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoBooleanProperty#get()} and to safely submit a new value via
    * {@link YoBooleanProperty#set(boolean)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    *
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *       variable doesn't match, or there is no active session.
    */
   YoBooleanProperty newYoBooleanProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoEnum<E>} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoEnumAsStringProperty#get()} and to safely submit a new value via
    * {@link YoEnumAsStringProperty#set(E)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    *
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *       variable doesn't match, or there is no active session.
    */
   <E extends Enum<E>> YoEnumAsStringProperty<E> newYoEnumProperty(String variableName);

   /**
    * Adds a listener to be notified when the active session has changed.
    * <p>
    * This is particularly useful for initializing controls once the session has changed.
    * </p>
    *
    * @param listener the session changed listener.
    */
   void addSessionChangedListener(SessionChangeListener listener);

   /**
    * Removes a listener previously added.
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed.
    */
   boolean removeSessionChangedListener(SessionChangeListener listener);

   /**
    * Requests to "gently" shutdown the visualizer and the session.
    * <p>
    * A confirmation dialog may popup to confirm the shutdown and ask for saving the GUI configuration.
    * </p>
    * <p>
    * After shutdown, the visualizer and the session become useless.
    * </p>
    */
   void requestVisualizerShutdown();

   /**
    * Requests an immediate shutdown of the visualizer and the session.
    * <p>
    * No confirmation dialog will be shown and the GUI configuration is not saved.
    * </p>
    * <p>
    * After shutdown, the visualizer and the session become useless.
    * </p>
    */
   void shutdownSession();

   /**
    * Indicates whether the visualizer has been shutdown or not.
    *
    * @return {@code true} if the visualizer has been shutdown, {@code false} otherwise.
    */
   boolean isVisualizerShutdown();

   /**
    * Adds a listener to be notified when the visualizer just shutdown.
    *
    * @param listener the shutdown listener.
    */
   void addVisualizerShutdownListener(Runnable listener);

   /**
    * Causes the caller's thread to pause until the visualizer is fully operational.
    */
   void waitUntilVisualizerFullyUp();

   /**
    * Causes the caller's thread to pause until the visualizer gets shutdown.
    */
   void waitUntilVisualizerDown();
}
