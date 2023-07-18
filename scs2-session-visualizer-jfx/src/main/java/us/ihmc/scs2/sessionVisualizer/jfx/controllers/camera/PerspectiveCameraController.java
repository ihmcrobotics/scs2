package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Predicate;

import javafx.animation.Interpolator;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.ReadOnlyDoubleProperty;
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
import javafx.scene.transform.Affine;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import javafx.util.Duration;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TranslateSCS2;

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
public class PerspectiveCameraController implements EventHandler<Event>
{
   private static final double DEFAULT_DISTANCE_FROM_FOCUS_POINT = 10.0;

   private static final boolean FOCUS_POINT_SHOW = SessionPropertiesHelper.loadBooleanPropertyOrEnvironment("scs2.session.gui.camera.focuspoint.show",
                                                                                                            "SCS2_GUI_CAMERA_FOCUS_SHOW",
                                                                                                            true);
   private static final double FOCUS_POINT_SIZE = SessionPropertiesHelper.loadDoublePropertyOrEnvironment("scs2.session.gui.camera.focuspoint.size",
                                                                                                          "SCS2_GUI_CAMERA_FOCUS_SIZE",
                                                                                                          0.0025);

   private final Sphere focalPointViz;

   /**
    * Rotation about the focus point. By construction it is meant to make the camera orbit about the
    * focus.
    */
   private final Affine cameraOrientation;
   /** Translation to control the distance separating the camera from the focus point. */
   private final Translate offsetFromFocusPoint = new Translate(0.0, 0.0, -DEFAULT_DISTANCE_FROM_FOCUS_POINT);

   private final CameraFocalPointHandler focalPointHandler;
   private final CameraZoomCalculator zoomCalculator = new CameraZoomCalculator();
   private final CameraRotationCalculator rotationCalculator;

   private final EventHandler<ScrollEvent> zoomEventHandler = zoomCalculator.createScrollEventHandler();
   /** For rotating around the focus point. */
   private final EventHandler<MouseEvent> orbitalRotationEventHandler;
   private final EventHandler<KeyEvent> translationEventHandler;

   private final PerspectiveCamera camera;

   private final ObservedAnimationTimer updater;

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

      zoomCalculator.zoomProperty().bindBidirectional(offsetFromFocusPoint.zProperty());
      zoomCalculator.invertZoomDirectionProperty().set(true);
      zoomCalculator.minZoomProperty().set(-0.90 * camera.getFarClip());
      zoomCalculator.maxZoomProperty().set(-1.10 * camera.getNearClip());

      rotationCalculator = new CameraRotationCalculator(up, forward);
      rotationCalculator.fastModifierPredicateProperty().set(event -> event.isShiftDown());
      cameraOrientation = rotationCalculator.getRotation();
      orbitalRotationEventHandler = rotationCalculator.createMouseEventHandler(sceneWidthProperty, sceneHeightProperty);

      focalPointHandler = new CameraFocalPointHandler(up);
      focalPointHandler.fastModifierPredicateProperty().set(event -> event.isShiftDown());
      focalPointHandler.setCameraOrientation(cameraOrientation);
      focalPointHandler.setZoom(zoomCalculator.zoomProperty());
      Translate focalPointTranslate = focalPointHandler.getTranslation();
      translationEventHandler = focalPointHandler.createKeyEventHandler();

      setCameraPosition(-2.0, 0.7, 1.0);

      camera.getTransforms().addAll(focalPointTranslate, cameraOrientation, offsetFromFocusPoint);

      if (FOCUS_POINT_SHOW)
      {
         focalPointViz = new Sphere(0.01);
         PhongMaterial material = new PhongMaterial();
         material.setDiffuseColor(Color.DARKRED);
         material.setSpecularColor(Color.RED);
         focalPointViz.setMaterial(material);
         focalPointViz.getTransforms().addAll(focalPointTranslate);
         offsetFromFocusPoint.zProperty().addListener((o, oldValue, newValue) ->
         {
            double sphereRadius = FOCUS_POINT_SIZE * Math.abs(offsetFromFocusPoint.getTz());
            focalPointViz.setRadius(sphereRadius);
         });
      }
      else
      {
         focalPointViz = null;
      }

      setupCameraRotationHandler();

      updater = new ObservedAnimationTimer(getClass().getSimpleName())
      {
         @Override
         public void handleImpl(long now)
         {
            focalPointHandler.update();
         }
      };
      updater.start();
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
         Point3D desiredFocalPoint = new Point3D(x, y, z);
         desiredFocalPoint.sub(cameraTransform.getTx(), cameraTransform.getTy(), cameraTransform.getTz());
         desiredFocalPoint.add(focalPointHandler.getTranslation());
         focalPointHandler.setPositionWorldFrame(desiredFocalPoint);
      }
      else
      {
         Translate focalPoint = focalPointHandler.getTranslation();
         x -= focalPoint.getX();
         y -= focalPoint.getY();
         z -= focalPoint.getZ();

         offsetFromFocusPoint.setZ(-EuclidCoreTools.norm(x, y, z));
         rotationCalculator.setLookDirection(x, y, z, 0.0);
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
    * @param x               the x-coordinate of the new focus location.
    * @param y               the y-coordinate of the new focus location.
    * @param z               the z-coordinate of the new focus location.
    * @param translateCamera whether to translate or rotate the camera when updating the focus point.
    */
   public void setFocalPoint(double x, double y, double z, boolean translateCamera)
   {
      setFocalPoint(new Point3D(x, y, z), translateCamera);
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
      focalPointHandler.disableTracking();

      if (translateCamera)
      {
         focalPointHandler.setPositionWorldFrame(desiredFocalPoint);
      }
      else
      {
         // The focus position is used to compute the camera transform, so first want to get the camera position.
         Transform cameraTransform = camera.getLocalToSceneTransform();
         Point3D currentCameraPosition = new Point3D(cameraTransform.getTx(), cameraTransform.getTy(), cameraTransform.getTz());

         focalPointHandler.setPositionWorldFrame(desiredFocalPoint);

         double distanceFromFocusPoint = currentCameraPosition.distance(desiredFocalPoint);
         offsetFromFocusPoint.setZ(-distanceFromFocusPoint);
         rotationCalculator.setRotationFromCameraAndFocusPositions(currentCameraPosition, desiredFocalPoint, 0.0);
      }
   }

   public void rotateCameraOnItself(double deltaLatitude, double deltaLongitude, double deltaRoll)
   {
      focalPointHandler.disableTracking();

      Transform cameraTransform = camera.getLocalToSceneTransform();
      Point3D currentCameraPosition = new Point3D(cameraTransform.getTx(), cameraTransform.getTy(), cameraTransform.getTz());

      rotationCalculator.updateRotation(deltaLatitude, deltaLongitude, deltaRoll);

      Point3D newFocusPointTranslation = new Point3D(cameraOrientation.getMxz(), cameraOrientation.getMyz(), cameraOrientation.getMzz());
      newFocusPointTranslation.scale(-offsetFromFocusPoint.getZ());
      newFocusPointTranslation.add(currentCameraPosition);
      focalPointHandler.setPositionWorldFrame(newFocusPointTranslation);
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
            rotateCameraOnItself(drag.getY(), -drag.getX(), 0.0);
            oldMouseLocation.set(newMouseLocation);
         }
      };
   }

   public void dispose()
   {
      cameraRotationEventHandler = null;
      updater.stop();
   }

   public Sphere getFocalPointViz()
   {
      return focalPointViz;
   }

   public PerspectiveCamera getCamera()
   {
      return camera;
   }

   public CameraFocalPointHandler getFocalPointHandler()
   {
      return focalPointHandler;
   }

   public CameraZoomCalculator getZoomCalculator()
   {
      return zoomCalculator;
   }

   public CameraRotationCalculator getRotationCalculator()
   {
      return rotationCalculator;
   }
}
