package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.Node;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler.TrackingTargetType;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TranslateSCS2;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

/**
 * Helper class for tracking a moving target. A target can be for instance:
 * <ul>
 * <li>a JavaFX node
 * <li>a reference frame
 * <li>a set of YoVariables defining coordinates
 * </ul>
 *
 * @author Sylvain Bertrand
 */
public class CameraFocalPointTargetTracker
{

   private final ObjectProperty<TrackingTargetType> targetType = new SimpleObjectProperty<>(this, "", TrackingTargetType.Disabled);
   private final ObjectProperty<Tuple3DProperty> coordinatesTracked = new SimpleObjectProperty<>(this, "coordinatesTracked", null);
   private final ObjectProperty<Node> nodeTracked = new SimpleObjectProperty<>(this, "nodeTracked", null);

   private final TranslateSCS2 trackingTranslate = new TranslateSCS2();

   private final ChangeListener<Transform> nodeTrackingListener = (o, oldTransform, newTransform) ->
   {
      if (targetType.get() == TrackingTargetType.Node)
         trackingTranslate.setFrom(newTransform);
   };

   private final ObservedAnimationTimer coordinatesTrackingAnimation = new ObservedAnimationTimer(getClass().getSimpleName())
   {
      @Override
      public void handleImpl(long now)
      {
         Tuple3DProperty tuple = coordinatesTracked.get();
         if (targetType.get() == TrackingTargetType.YoCoordinates && tuple != null)
            trackingTranslate.set(tuple.toPoint3DInWorld());
      }
   };

   public CameraFocalPointTargetTracker()
   {
      nodeTracked.addListener((o, oldValue, newValue) ->
      {
         if (oldValue != null)
         {
            oldValue.localToSceneTransformProperty().removeListener(nodeTrackingListener);
         }

         if (newValue != null)
         {
            newValue.localToSceneTransformProperty().addListener(nodeTrackingListener);
            nodeTrackingListener.changed(null, null, newValue.getLocalToSceneTransform());
         }
      });

      coordinatesTracked.addListener((o, oldValue, newValue) ->
      {
         if (newValue != null)
            coordinatesTrackingAnimation.start();
         else
            coordinatesTrackingAnimation.stop();
      });
   }

   public void dispose()
   {
      coordinatesTrackingAnimation.stop();
      nodeTracked.set(null);
      coordinatesTracked.set(null);
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

   public void resetTranslate()
   {
      trackingTranslate.setX(0.0);
      trackingTranslate.setY(0.0);
      trackingTranslate.setZ(0.0);
   }

   public Translate getTrackingTranslate()
   {
      return trackingTranslate;
   }
}
