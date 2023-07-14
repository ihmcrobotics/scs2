package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.Node;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
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
   private final ObjectProperty<Tuple3DProperty> coordinatesTracked = new SimpleObjectProperty<>(this, "coordinatesTracked", null);
   private final ObjectProperty<Node> nodeTracked = new SimpleObjectProperty<>(this, "nodeTracked", null);

   private final Translate trackingTranslate = new Translate();

   private final ChangeListener<Transform> nodeTrackingListener = (o, oldTransform, newTransform) ->
   {
      trackingTranslate.setX(newTransform.getTx());
      trackingTranslate.setY(newTransform.getTy());
      trackingTranslate.setZ(newTransform.getTz());
   };

   public CameraTargetTracker(Translate cameraFocusTranslate)
   {
      nodeTracked.addListener((o, oldValue, newValue) ->
      {
         if (oldValue != null)
         {
            oldValue.localToSceneTransformProperty().removeListener(nodeTrackingListener);
         }

         if (newValue != null)
         {
            cameraFocusTranslate.setX(0.0);
            cameraFocusTranslate.setY(0.0);
            cameraFocusTranslate.setZ(0.0);
            newValue.localToSceneTransformProperty().addListener(nodeTrackingListener);
            nodeTrackingListener.changed(null, null, newValue.getLocalToSceneTransform());
         }
      });
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
