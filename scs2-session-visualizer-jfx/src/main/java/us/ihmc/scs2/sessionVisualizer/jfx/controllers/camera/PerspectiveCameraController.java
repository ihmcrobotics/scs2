package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Predicate;

import javafx.animation.Interpolator;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SubScene;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.input.ScrollEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Transform;
import javafx.util.Duration;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraOrbitHandler.LevelOrbitalCoordinateProvider;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraOrbitHandler.OrbitalCoordinateProvider;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TranslateSCS2;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

/**
 * This class provides a simple controller for a JavaFX {@link PerspectiveCamera}. The control is
 * achieved via event handling by adding this controller as an {@link EventHandler} to the scene or
 * sub-scene the camera is attached to.
 * <p>
 * Behavior of this camera controller:
 * <li>The camera is always pointing toward a focus point.
 * <li>The focus point can be translated via keyboard bindings, or instantly moved with a mouse
 * shortcut only if {@link #setupRayBasedFocusTranslation(Predicate)} or
 * {@link #enableShiftClickFocusTranslation()} has been called.
 * <li>The camera zoom can be changed vi the mouse wheel.
 * <li>Using the mouse, the camera can be rotated around the focus point.
 *
 * @author Sylvain Bertrand
 */
public class PerspectiveCameraController extends ObservedAnimationTimer implements EventHandler<Event>
{
   private static final boolean FOCUS_POINT_SHOW = SessionPropertiesHelper.loadBooleanPropertyOrEnvironment("scs2.session.gui.camera.focuspoint.show",
                                                                                                            "SCS2_GUI_CAMERA_FOCUS_SHOW",
                                                                                                            true);
   private static final double FOCUS_POINT_SIZE = SessionPropertiesHelper.loadDoublePropertyOrEnvironment("scs2.session.gui.camera.focuspoint.size",
                                                                                                          "SCS2_GUI_CAMERA_FOCUS_SIZE",
                                                                                                          0.0025);

   private final Sphere focalPointViz;

   private final CameraFocalPointHandler focalPointHandler;
   private final CameraOrbitHandler orbitHandler;

   /** Minimum value of a translation offset when using the keyboard. */
   private final DoubleProperty minTranslationOffset = new SimpleDoubleProperty(this, "minTranslationOffset", 0.1);
   /**
    * The zoom-to-translation pow is used to define the relation between the current zoom value and the
    * translation speed of the camera.
    */
   private final DoubleProperty zoomToTranslationPow = new SimpleDoubleProperty(this, "zoomToTranslationPow", 0.75);

   private final EventHandler<ScrollEvent> zoomEventHandler;
   private final EventHandler<MouseEvent> orbitalRotationEventHandler;
   private final EventHandler<KeyEvent> translationEventHandler;

   private final PerspectiveCamera camera;

   private final Property<Tuple3DProperty> cameraPositionCoordinatesToTrack = new SimpleObjectProperty<>(this, "cameraPositionCoordinatesToTrack", null);
   private final Property<OrbitalCoordinateProvider> cameraOrbitalCoordinatesToTrack = new SimpleObjectProperty<>(this, "cameraOrbitCoordinatesToTrack", null);
   private final Property<LevelOrbitalCoordinateProvider> cameraLevelOrbitalCoordinatesToTrack = new SimpleObjectProperty<>(this,
                                                                                                                            "cameraOrbit2DCoordinatesToTrack",
                                                                                                                            null);

   public PerspectiveCameraController(ReadOnlyDoubleProperty sceneWidthProperty,
                                      ReadOnlyDoubleProperty sceneHeightProperty,
                                      PerspectiveCamera camera,
                                      Vector3DReadOnly up,
                                      Vector3DReadOnly forward)
   {
      this.camera = camera;
      Vector3D left = new Vector3D();
      left.cross(up, forward);
      if (!MathTools.epsilonEquals(left.norm(), 1.0, Epsilons.ONE_HUNDRED_THOUSANDTH))
         throw new RuntimeException("The vectors up and forward must be orthogonal. Received: up = " + up + ", forward = " + forward);

      orbitHandler = new CameraOrbitHandler(up, forward);
      orbitHandler.fastModifierPredicateProperty().set(event -> event.isShiftDown());
      orbitalRotationEventHandler = orbitHandler.createMouseEventHandler(sceneWidthProperty, sceneHeightProperty);
      zoomEventHandler = orbitHandler.createScrollEventHandler();
      orbitHandler.minDistanceProperty().set(1.10 * camera.getNearClip());
      orbitHandler.maxDistanceProperty().set(0.90 * camera.getFarClip());

      focalPointHandler = new CameraFocalPointHandler(up);
      focalPointHandler.fastModifierPredicateProperty().set(event -> event.isShiftDown());
      focalPointHandler.setCameraOrientation(orbitHandler.getCameraRotation());
      translationEventHandler = focalPointHandler.createKeyEventHandler();

      focalPointHandler.setTranslationRateModifier(translationRate ->
      {
         return Math.min(translationRate * Math.pow(orbitHandler.distanceProperty().get(), zoomToTranslationPow.get()), minTranslationOffset.get());
      });

      setCameraPosition(-2.0, 0.7, 1.0);

      camera.getTransforms().addAll(focalPointHandler.getTranslation(), orbitHandler.getCameraPose());

      cameraPositionCoordinatesToTrack.addListener((o, oldValue, newValue) ->
      {
         if (newValue != null)
         {
            cameraOrbitalCoordinatesToTrack.setValue(null);
            cameraLevelOrbitalCoordinatesToTrack.setValue(null);
         }
      });
      cameraOrbitalCoordinatesToTrack.addListener((o, oldValue, newValue) ->
      {
         if (newValue != null)
         {
            cameraPositionCoordinatesToTrack.setValue(null);
            cameraLevelOrbitalCoordinatesToTrack.setValue(null);
         }
      });
      cameraLevelOrbitalCoordinatesToTrack.addListener((o, oldValue, newValue) ->
      {
         if (newValue != null)
         {
            cameraPositionCoordinatesToTrack.setValue(null);
            cameraOrbitalCoordinatesToTrack.setValue(null);
         }
      });

      if (FOCUS_POINT_SHOW)
      {
         focalPointViz = new Sphere(0.01);
         PhongMaterial material = new PhongMaterial();
         material.setDiffuseColor(Color.DARKRED);
         material.setSpecularColor(Color.RED);
         focalPointViz.setMaterial(material);
         focalPointViz.getTransforms().add(focalPointHandler.getTranslation());
         orbitHandler.distanceProperty().addListener((o, oldValue, newValue) ->
         {
            focalPointViz.setRadius(FOCUS_POINT_SIZE * newValue.doubleValue());
         });
      }
      else
      {
         focalPointViz = null;
      }

      setupCameraRotationHandler();
   }

   @Override
   public void handleImpl(long now)
   {
      focalPointHandler.update();

      {
         Tuple3DProperty newPosition = cameraPositionCoordinatesToTrack.getValue();
         if (newPosition != null)
            setCameraPosition(newPosition.toPoint3DInWorld(), false);
      }

      {
         OrbitalCoordinateProvider newOrbit = cameraOrbitalCoordinatesToTrack.getValue();
         if (newOrbit != null)
            setCameraOrbit(newOrbit.getDistance(), newOrbit.getLatitude(), newOrbit.getLongitude(), newOrbit.getRoll(), false);
      }

      {
         LevelOrbitalCoordinateProvider newOrbit = cameraLevelOrbitalCoordinatesToTrack.getValue();
         if (newOrbit != null)
            setCameraOrbit(newOrbit.getDistance(), newOrbit.getHeight(), newOrbit.getLongitude(), newOrbit.getRoll(), false);
      }
   }

   public void setCameraPosition(Point3DReadOnly desiredCameraPosition)
   {
      setCameraPosition(desiredCameraPosition, false);
   }

   public void setCameraPosition(Point3DReadOnly desiredCameraPosition, boolean translateFocalPoint)
   {
      setCameraPosition(desiredCameraPosition.getX(), desiredCameraPosition.getY(), desiredCameraPosition.getZ(), translateFocalPoint);
   }

   public void setCameraPosition(double x, double y, double z)
   {
      setCameraPosition(x, y, z, false);
   }

   public void setCameraPosition(double x, double y, double z, boolean translateFocalPoint)
   {
      if (translateFocalPoint)
      {
         Transform cameraTransform = camera.getLocalToSceneTransform();
         focalPointHandler.translateWorldFrame(x - cameraTransform.getTx(), y - cameraTransform.getTy(), z - cameraTransform.getTz());
      }
      else
      {
         TranslateSCS2 focalPoint = focalPointHandler.getTranslation();
         orbitHandler.setPosition(x - focalPoint.getX(), y - focalPoint.getY(), z - focalPoint.getZ(), 0.0);
      }
   }

   /**
    * Sets the coordinates of the focus point the camera is looking at.
    * <p>
    * This can be done in 2 different ways controlled by the argument {@code translateCamera}:
    * <ul>
    * <li>translating the camera: the offset between the focus point and the camera is preserved as
    * well as the camera orientation. This will be used when {@code translateCamera = true}.
    * <li>rotating the camera: the distance between the focus point and the camera changes, the camera
    * will pitch and/or yaw as a result of this operation. This will be used when
    * {@code translateCamera = false}.
    * </ul>
    * </p>
    * 
    * @param desiredFocalPoint the new focus location.
    * @param translateCamera   whether to translate or rotate the camera when updating the focus point.
    */
   public void setFocalPoint(Point3DReadOnly desiredFocalPoint, boolean translateCamera)
   {
      setFocalPoint(desiredFocalPoint.getX(), desiredFocalPoint.getY(), desiredFocalPoint.getZ(), translateCamera);
   }

   /**
    * Sets the coordinates of the focus point the camera is looking at.
    * <p>
    * This can be done in 2 different ways controlled by the argument {@code translateCamera}:
    * <ul>
    * <li>translating the camera: the offset between the focus point and the camera is preserved as
    * well as the camera orientation. This will be used when {@code translateCamera = true}.
    * <li>rotating the camera: the distance between the focus point and the camera changes, the camera
    * will pitch and/or yaw as a result of this operation. This will be used when
    * {@code translateCamera = false}.
    * </ul>
    * </p>
    * 
    * @param x               the x-coordinate of the new focus location.
    * @param y               the y-coordinate of the new focus location.
    * @param z               the z-coordinate of the new focus location.
    * @param translateCamera whether to translate or rotate the camera when updating the focus point.
    */
   public void setFocalPoint(double x, double y, double z, boolean translateCamera)
   {
      focalPointHandler.disableTracking();

      if (translateCamera)
      {
         focalPointHandler.setPositionWorldFrame(x, y, z);
      }
      else
      {
         // The focus position is used to compute the camera transform, so first want to get the camera position.
         Transform cameraTransform = camera.getLocalToSceneTransform();
         orbitHandler.setPosition(cameraTransform.getTx() - x, cameraTransform.getTy() - y, cameraTransform.getTz() - z, 0.0);
         focalPointHandler.setPositionWorldFrame(x, y, z);
      }
   }

   public void setCameraOrientation(double latitude, double longitude, double roll, boolean translateFocalPoint)
   {
      if (translateFocalPoint)
      {
         Vector3D focalPointShift = orbitHandler.setRotation(latitude, longitude, roll, true);
         focalPointHandler.translateWorldFrame(focalPointShift);

      }
      else
      {
         orbitHandler.setRotation(latitude, longitude, roll);
      }
   }

   public void setCameraOrbit(double distance, double latitude, double longitude, double roll, boolean translateFocalPoint)
   {
      if (translateFocalPoint)
      {
         Vector3D focalPointShift = orbitHandler.setOrbit(distance, latitude, longitude, roll, true);
         focalPointHandler.translateWorldFrame(focalPointShift);

      }
      else
      {
         orbitHandler.setOrbit(distance, latitude, longitude, roll);
      }
   }

   public void setCameraLevelOrbit(double distance, double height, double longitude, double roll, boolean translateFocalPoint)
   {
      if (translateFocalPoint)
      {
         height -= focalPointHandler.getTranslation().getZ();
         Vector3D focalPointShift = orbitHandler.setLevelOrbit(distance, height, longitude, roll, true);
         focalPointHandler.translateWorldFrame(focalPointShift);

      }
      else
      {
         orbitHandler.setLevelOrbit(distance, height, longitude, roll);
      }
   }

   public void rotateCamera(double deltaLatitude, double deltaLongitude, double deltaRoll, boolean translateFocalPoint)
   {
      if (translateFocalPoint)
      {
         Vector3D focalPointShift = orbitHandler.rotate(deltaLatitude, deltaLongitude, deltaRoll, true);
         focalPointHandler.translateWorldFrame(focalPointShift);
      }
      else
      {
         orbitHandler.rotate(deltaLatitude, deltaLongitude, deltaRoll);
      }
   }

   @Override
   public void handle(Event event)
   {
      if (event instanceof ScrollEvent)
         zoomEventHandler.handle((ScrollEvent) event);
      if (event instanceof KeyEvent)
         translationEventHandler.handle((KeyEvent) event);

      if (event instanceof MouseEvent)
      {
         MouseEvent mouseEvent = (MouseEvent) event;

         if (rayBasedFocusTranslation != null)
            rayBasedFocusTranslation.handle(mouseEvent);

         if (!event.isConsumed())
            orbitalRotationEventHandler.handle(mouseEvent);

         if (!event.isConsumed())
         {
            if (cameraRotationEventHandler != null)
               cameraRotationEventHandler.handle(mouseEvent);
         }
      }
   }

   private EventHandler<MouseEvent> rayBasedFocusTranslation = null;

   public void enableShiftClickFocusTranslation()
   {
      enableShiftClickFocusTranslation(MouseEvent::getPickResult);
   }

   public void enableShiftClickFocusTranslation(Function<MouseEvent, PickResult> nodePickingFunction)
   {
      setupRayBasedFocusTranslation(event ->
      {
         if (!event.isShiftDown())
            return false;
         if (event.getButton() != MouseButton.PRIMARY)
            return false;
         if (!event.isStillSincePress())
            return false;
         if (event.getEventType() != MouseEvent.MOUSE_CLICKED)
            return false;

         return true;
      }, nodePickingFunction);
   }

   public void setupRayBasedFocusTranslation(Predicate<MouseEvent> condition)
   {
      setupRayBasedFocusTranslation(condition, MouseEvent::getPickResult);
   }

   public void setupRayBasedFocusTranslation(Predicate<MouseEvent> condition, Function<MouseEvent, PickResult> nodePickingFunction)
   {
      setupRayBasedFocusTranslation(condition, nodePickingFunction, 0.1);
   }

   public void setupRayBasedFocusTranslation(Predicate<MouseEvent> condition, double animationDuration)
   {
      setupRayBasedFocusTranslation(condition, MouseEvent::getPickResult, animationDuration);
   }

   public void setupRayBasedFocusTranslation(Predicate<MouseEvent> condition, Function<MouseEvent, PickResult> nodePickingFunction, double animationDuration)
   {
      rayBasedFocusTranslation = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            if (condition.test(event))
            {
               PickResult pickResult = nodePickingFunction.apply(event);
               if (pickResult == null)
                  return;
               Node intersectedNode = pickResult.getIntersectedNode();
               if (intersectedNode == null || intersectedNode instanceof SubScene)
                  return;
               javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
               javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);
               focalPointHandler.disableTracking();
               TranslateSCS2 translate = focalPointHandler.getOffsetTranslation();

               if (animationDuration > 0.0)
               {
                  Timeline animation = new Timeline(new KeyFrame(Duration.seconds(animationDuration),
                                                                 new KeyValue(translate.xProperty(), scenePoint.getX(), Interpolator.EASE_BOTH),
                                                                 new KeyValue(translate.yProperty(), scenePoint.getY(), Interpolator.EASE_BOTH),
                                                                 new KeyValue(translate.zProperty(), scenePoint.getZ(), Interpolator.EASE_BOTH)));
                  animation.playFromStart();
               }
               else
               {
                  translate.set(scenePoint);
               }

               event.consume();
            }
         }
      };
   }

   private EventHandler<MouseEvent> cameraRotationEventHandler;

   public void setupCameraRotationHandler()
   {
      setupCameraRotationHandler(MouseButton.SECONDARY);
   }

   public void setupCameraRotationHandler(MouseButton mouseButton)
   {
      setupCameraRotationHandler(mouseButton, () -> 0.003);
   }

   public void setupCameraRotationHandler(MouseButton mouseButton, DoubleSupplier modifier)
   {
      cameraRotationEventHandler = new EventHandler<MouseEvent>()
      {
         private final Point2D oldMouseLocation = new Point2D();

         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() != mouseButton)
               return;

            // Filters single clicks
            if (event.isStillSincePress())
               return;

            if (event.getEventType() == MouseEvent.MOUSE_PRESSED)
            {
               oldMouseLocation.set(event.getSceneX(), event.getSceneY());
               return;
            }

            if (event.getEventType() != MouseEvent.MOUSE_DRAGGED)
               return;

            // Acquire the new mouse coordinates from the recent event
            Point2D newMouseLocation = new Point2D(event.getSceneX(), event.getSceneY());

            Vector2D drag = new Vector2D();
            drag.sub(newMouseLocation, oldMouseLocation);
            drag.scale(modifier.getAsDouble());
            focalPointHandler.disableTracking(); // Automatically disabling tracking when rotating the camera on itself
            rotateCamera(drag.getY(), -drag.getX(), 0.0, true);
            oldMouseLocation.set(newMouseLocation);
         }
      };
   }

   public Sphere getFocalPointViz()
   {
      return focalPointViz;
   }

   public TranslateSCS2 getFocalPointTranslate()
   {
      return focalPointHandler.getTranslation();
   }

   public PerspectiveCamera getCamera()
   {
      return camera;
   }

   public CameraFocalPointHandler getFocalPointHandler()
   {
      return focalPointHandler;
   }

   public CameraOrbitHandler getOrbitHandler()
   {
      return orbitHandler;
   }

   public Property<Tuple3DProperty> cameraPositionCoordinatesToTrackProperty()
   {
      return cameraPositionCoordinatesToTrack;
   }

   public Property<OrbitalCoordinateProvider> cameraOrbitalCoordinatesToTrackProperty()
   {
      return cameraOrbitalCoordinatesToTrack;
   }

   public Property<LevelOrbitalCoordinateProvider> cameraLevelOrbitalCoordinatesToTrackProperty()
   {
      return cameraLevelOrbitalCoordinatesToTrack;
   }
}
