package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TranslateSCS2;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

/**
 * This class handles the control in position of the camera's focal point.
 * <p>
 * The focal point can be controlled:
 * <ul>
 * <li>Pressing keyboards keys (ASDW) to translate in the world's horizontal plane (independent of
 * the camera's pitch angle).
 * <li>Setting directly the location of the focal point.
 * <li>Tracking a node or coordinates.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class CameraFocalPointHandler
{
   /** The current translation of the focal point. */
   private final Translate focalPointTranslation = new Translate();
   /** The translation used to map keyboard to translation. It is expressed in world frame */
   private final TranslateSCS2 offsetTranslation = new TranslateSCS2();
   /** Current orientation of the camera necessary when translating the camera in its local frame. */
   private final ObjectProperty<Transform> cameraOrientation = new SimpleObjectProperty<>(this, "cameraOrientation", null);
   /**
    * Current zoom of the camera. It is used to vary the translation speed when using the keyboard. As
    * the zoom increases, the translation speed increases too.
    */
   private ReadOnlyDoubleProperty currentZoom = null;

   /**
    * When set to true, the translations forward/backward and left/right will be performed on a
    * horizontal plane, i.e. perpendicular the given up axis.
    */
   private final BooleanProperty keepTranslationLeveled = new SimpleBooleanProperty(this, "keepTranslationLeveled", true);
   /**
    * Condition to trigger the use of the fast modifier to make the camera translate faster when using
    * the keyboard.
    */
   private final ObjectProperty<Predicate<KeyEvent>> fastModifierPredicate = new SimpleObjectProperty<>(this, "fastModifierPredicate", null);
   /** Slow camera translation modifier when using the keyboard. */
   private final DoubleProperty slowModifier = new SimpleDoubleProperty(this, "slowModifier", 0.0075);
   /**
    * Fast camera translation modifier when using the keyboard. It is triggered when the condition held
    * in {@link #fastModifierPredicate} is fulfilled.
    */
   private final DoubleProperty fastModifier = new SimpleDoubleProperty(this, "fastModifier", 0.0125);
   /** Minimum value of a translation offset when using the keyboard. */
   private final DoubleProperty minTranslationOffset = new SimpleDoubleProperty(this, "minTranslationOffset", 0.1);
   /**
    * The zoom-to-translation pow is used to define the relation between the current zoom value and the
    * translation speed of the camera.
    */
   private final DoubleProperty zoomToTranslationPow = new SimpleDoubleProperty(this, "zoomToTranslationPow", 0.75);

   /** Key binding for moving the camera forward. Its default value is: {@code KeyCode.W}. */
   private final ObjectProperty<KeyCode> forwardKey = new SimpleObjectProperty<>(this, "forwardKey", KeyCode.W);
   /** Key binding for moving the camera backward. Its default value is: {@code KeyCode.S}. */
   private final ObjectProperty<KeyCode> backwardKey = new SimpleObjectProperty<>(this, "backwardKey", KeyCode.S);

   /** Key binding for moving the camera to the left. Its default value is: {@code KeyCode.A}. */
   private final ObjectProperty<KeyCode> leftKey = new SimpleObjectProperty<>(this, "leftKey", KeyCode.A);
   /** Key binding for moving the camera to the right. Its default value is: {@code KeyCode.D}. */
   private final ObjectProperty<KeyCode> rightKey = new SimpleObjectProperty<>(this, "rightKey", KeyCode.D);

   /** Key binding for moving the camera upward. Its default value is: {@code KeyCode.Q}. */
   private final ObjectProperty<KeyCode> upKey = new SimpleObjectProperty<>(this, "upKey", KeyCode.Q);
   /** Key binding for moving the camera downward. Its default value is: {@code KeyCode.Z}. */
   private final ObjectProperty<KeyCode> downKey = new SimpleObjectProperty<>(this, "downKey", KeyCode.Z);

   // Following is for following a target
   public enum TrackingTargetType
   {
      Disabled, Node, YoCoordinates
   };

   private final ObjectProperty<TrackingTargetType> targetType = new SimpleObjectProperty<>(this, "", TrackingTargetType.Disabled);
   private final ObjectProperty<Tuple3DProperty> coordinatesTracked = new SimpleObjectProperty<>(this, "coordinatesTracked", null);
   private final ObjectProperty<Node> nodeTracked = new SimpleObjectProperty<>(this, "nodeTracked", null);
   private final TranslateSCS2 trackingTranslate = new TranslateSCS2();

   private final ChangeListener<Transform> nodeTrackingListener = (o, oldTransform, newTransform) ->
   {
      if (targetType.get() == TrackingTargetType.Node)
         trackingTranslate.setFrom(newTransform);
   };

   private final Vector3D down = new Vector3D();
   private final List<Runnable> updateTasks = new ArrayList<>();

   /**
    * Creates a calculator for the camera translation.
    *
    * @param up indicates which way is up.
    */
   public CameraFocalPointHandler(Vector3DReadOnly up)
   {
      down.setAndNegate(up);

      targetType.addListener((o, oldValue, newValue) ->
      {
         if (newValue == TrackingTargetType.Disabled)
         {
            offsetTranslation.set(trackingTranslate);
            trackingTranslate.setToZero();
         }
      });

      nodeTracked.addListener((o, oldValue, newValue) ->
      {
         if (oldValue != null)
            oldValue.localToSceneTransformProperty().removeListener(nodeTrackingListener);

         if (newValue != null)
         {
            targetType.set(TrackingTargetType.Node);
            // Reset the focus translation whenever the target gets updated
            offsetTranslation.setToZero();

            newValue.localToSceneTransformProperty().addListener(nodeTrackingListener);
            nodeTrackingListener.changed(null, null, newValue.getLocalToSceneTransform());
         }
         else if (targetType.get() == TrackingTargetType.Node)
         {
            disableTracking();
         }
      });

      coordinatesTracked.addListener((o, oldValue, newValue) ->
      {
         if (newValue != null)
         {
            targetType.set(TrackingTargetType.YoCoordinates);
            // Reset the focus translation whenever the target gets updated
            offsetTranslation.setToZero();
         }
         else if (targetType.get() == TrackingTargetType.YoCoordinates)
         {
            disableTracking();
         }
      });

      updateTasks.add(() ->
      {
         Tuple3DProperty tuple = coordinatesTracked.get();
         if (tuple != null && targetType.get() == TrackingTargetType.YoCoordinates)
            trackingTranslate.set(tuple.toPoint3DInWorld());
      });

      focalPointTranslation.xProperty().bind(trackingTranslate.xProperty().add(offsetTranslation.xProperty()));
      focalPointTranslation.yProperty().bind(trackingTranslate.yProperty().add(offsetTranslation.yProperty()));
      focalPointTranslation.zProperty().bind(trackingTranslate.zProperty().add(offsetTranslation.zProperty()));
   }

   public void disableTracking()
   {
      targetType.set(TrackingTargetType.Disabled);
   }

   public boolean isTrackingDisabled()
   {
      return targetType.get() == TrackingTargetType.Disabled;
   }

   public void update()
   {
      for (int i = 0; i < updateTasks.size(); i++)
      {
         updateTasks.get(i).run();
      }
   }

   /**
    * Creates an {@link EventHandler} to translate the camera using keyboard bindings.
    *
    * @return an {@link EventHandler} to translate the camera with the keyboard.
    */
   public EventHandler<KeyEvent> createKeyEventHandler()
   {
      final Vector3D activeTranslationOffset = new Vector3D();

      updateTasks.add(() -> translateCameraFrame(activeTranslationOffset));

      EventHandler<KeyEvent> keyEventHandler = new EventHandler<KeyEvent>()
      {
         @Override
         public void handle(KeyEvent event)
         {
            double modifier;
            if (fastModifierPredicate.get() == null || !fastModifierPredicate.get().test(event))
               modifier = slowModifier.get();
            else
               modifier = fastModifier.get();

            if (currentZoom != null)
            {
               modifier *= Math.pow(Math.abs(currentZoom.get()), zoomToTranslationPow.get());
               modifier = Math.min(modifier, minTranslationOffset.get());
            }

            KeyCode keyDown = event.getCode();
            boolean isKeyReleased = event.getEventType() == KeyEvent.KEY_RELEASED;

            if (keyDown == forwardKey.get())
               activeTranslationOffset.setZ(isKeyReleased ? 0.0 : modifier);
            if (keyDown == backwardKey.get())
               activeTranslationOffset.setZ(isKeyReleased ? 0.0 : -modifier);

            if (keyDown == rightKey.get())
               activeTranslationOffset.setX(isKeyReleased ? 0.0 : modifier);
            if (keyDown == leftKey.get())
               activeTranslationOffset.setX(isKeyReleased ? 0.0 : -modifier);

            if (keyDown == downKey.get())
               activeTranslationOffset.setY(isKeyReleased ? 0.0 : modifier);
            if (keyDown == upKey.get())
               activeTranslationOffset.setY(isKeyReleased ? 0.0 : -modifier);
         }
      };

      return keyEventHandler;
   }

   /**
    * Update the camera translation after applying a translation offset in the camera local frame.
    *
    * @param translationOffset the translation offset in local frame to apply. Not modified.
    */
   public void translateCameraFrame(Vector3DReadOnly translationOffset)
   {
      translateCameraFrame(translationOffset.getX(), translationOffset.getY(), translationOffset.getZ());
   }

   /**
    * Update the camera translation after applying a translation offset in the camera local frame.
    *
    * @param dx the forward/backward translation offset in the camera local frame.
    * @param dy the left/right translation offset in the camera local frame.
    * @param dz the up/down translation offset in the camera local frame.
    */
   public void translateCameraFrame(double dx, double dy, double dz)
   {
      if (cameraOrientation.get() == null)
      {
         translateWorldFrame(dx, dy, dz);
         return;
      }

      Vector3D shift = new Vector3D(dx, dy, dz);

      if (keepTranslationLeveled.get())
      {
         double mxz = cameraOrientation.get().getMxz();
         double myz = cameraOrientation.get().getMyz();
         double mzz = cameraOrientation.get().getMzz();
         Vector3D cameraZAxis = new Vector3D(mxz, myz, mzz);
         Vector3D xAxisLeveled = new Vector3D();
         xAxisLeveled.cross(down, cameraZAxis);
         xAxisLeveled.normalize();
         Vector3D yAxisLeveled = new Vector3D(down);
         Vector3D zAxisLeveled = new Vector3D();
         zAxisLeveled.cross(xAxisLeveled, yAxisLeveled);

         RotationMatrix rotation = new RotationMatrix();
         rotation.setColumns(xAxisLeveled, yAxisLeveled, zAxisLeveled);
         rotation.transform(shift);
      }
      else
      {
         JavaFXMissingTools.applyTranform(cameraOrientation.get(), shift);
      }

      offsetTranslation.add(shift);
   }

   /**
    * Update the camera translation after applying a translation offset in the world frame.
    *
    * @param dx the translation offset along the world x-axis.
    * @param dy the translation offset along the world y-axis.
    * @param dz the translation offset along the world z-axis.
    */
   public void translateWorldFrame(double dx, double dy, double dz)
   {
      offsetTranslation.add(dx, dy, dz);
   }

   public void setPositionWorldFrame(Tuple3DReadOnly position)
   {
      setPositionWorldFrame(position.getX(), position.getY(), position.getZ());
   }

   public void setPositionWorldFrame(double x, double y, double z)
   {
      offsetTranslation.set(x, y, z);
      if (targetType.get() != TrackingTargetType.Disabled)
         offsetTranslation.sub(trackingTranslate);
   }

   /**
    * Sets the reference to the current camera orientation to enable translation in the camera frame,
    * i.e. first person.
    *
    * @param cameraOrientation the reference to the current camera orientation. Not modified.
    */
   public void setCameraOrientation(Transform cameraOrientation)
   {
      this.cameraOrientation.set(cameraOrientation);
   }

   /**
    * Sets the reference to the current camera zoom to enable translation speed varying on the zoom
    * value.
    *
    * @param zoom
    */
   public void setZoom(ReadOnlyDoubleProperty zoom)
   {
      currentZoom = zoom;
   }

   /**
    * Get the reference to the translation of the focal point expressed in world frame.
    *
    * @return the focal point position.
    */
   public Translate getTranslation()
   {
      return focalPointTranslation;
   }

   /**
    * Gets the translation that can be set to change the focal point position.
    * 
    * @return
    */
   public TranslateSCS2 getOffsetTranslation()
   {
      return offsetTranslation;
   }

   public final BooleanProperty keepTranslationLeveledProperty()
   {
      return keepTranslationLeveled;
   }

   public final ObjectProperty<Predicate<KeyEvent>> fastModifierPredicateProperty()
   {
      return fastModifierPredicate;
   }

   public final DoubleProperty slowModifierProperty()
   {
      return slowModifier;
   }

   public final DoubleProperty fastModifierProperty()
   {
      return fastModifier;
   }

   public final DoubleProperty minTranslationOffsetProperty()
   {
      return minTranslationOffset;
   }

   public final DoubleProperty zoomToTranslationPowProperty()
   {
      return zoomToTranslationPow;
   }

   public final ObjectProperty<KeyCode> forwardKeyProperty()
   {
      return forwardKey;
   }

   public final ObjectProperty<KeyCode> backwardKeyProperty()
   {
      return backwardKey;
   }

   public final ObjectProperty<KeyCode> leftKeyProperty()
   {
      return leftKey;
   }

   public final ObjectProperty<KeyCode> rightKeyProperty()
   {
      return rightKey;
   }

   public final ObjectProperty<KeyCode> upKeyProperty()
   {
      return upKey;
   }

   public final ObjectProperty<KeyCode> downKeyProperty()
   {
      return downKey;
   }

   public TranslateSCS2 getTrackingTranslate()
   {
      return trackingTranslate;
   }

   public ObjectProperty<TrackingTargetType> targetTypeProperty()
   {
      return targetType;
   }

   public ObjectProperty<Tuple3DProperty> coordinatesToTrackProperty()
   {
      return coordinatesTracked;
   }

   public ObjectProperty<Node> nodeToTrackProperty()
   {
      return nodeTracked;
   }
}
