package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.Node;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
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
public class CameraTargetTracker
{
   public enum TrackingTargetType
   {
      Disabled, Node, YoCoordinates
   };

   private final ObjectProperty<TrackingTargetType> targetType = new SimpleObjectProperty<>(this, "", TrackingTargetType.Disabled);
   private final ObjectProperty<Tuple3DProperty> coordinatesTracked = new SimpleObjectProperty<>(this, "coordinatesTracked", null);
   private final ObjectProperty<Node> nodeTracked = new SimpleObjectProperty<>(this, "nodeTracked", null);

   private final Translate trackingTranslate = new Translate();

   private final ChangeListener<Transform> nodeTrackingListener = (o, oldTransform, newTransform) ->
   {
      if (targetType.get() == TrackingTargetType.Node)
      {
         trackingTranslate.setX(newTransform.getTx());
         trackingTranslate.setY(newTransform.getTy());
         trackingTranslate.setZ(newTransform.getTz());
      }
   };

   private final ObservedAnimationTimer coordinatesTrackingAnimation = new ObservedAnimationTimer(getClass().getSimpleName())
   {
      @Override
      public void handleImpl(long now)
      {
         Tuple3DProperty tuple = coordinatesTracked.get();
         if (targetType.get() == TrackingTargetType.YoCoordinates && tuple != null)
         {
            Point3D pointInWorld = tuple.toPoint3DInWorld();
            trackingTranslate.setX(pointInWorld.getX());
            trackingTranslate.setY(pointInWorld.getY());
            trackingTranslate.setZ(pointInWorld.getZ());
         }
      }
   };

   public CameraTargetTracker()
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
