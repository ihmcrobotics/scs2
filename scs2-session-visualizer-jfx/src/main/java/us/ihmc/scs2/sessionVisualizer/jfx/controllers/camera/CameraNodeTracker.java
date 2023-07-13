package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.Node;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;

/**
 * Helper class for tracking a 3D node with a camera when using
 * {@link FocusBasedCameraMouseEventHandler}.
 *
 * @author Sylvain Bertrand
 */
public class CameraNodeTracker
{
   private final ObjectProperty<Node> nodeTracked = new SimpleObjectProperty<>(this, "nodeTracked", null);
   private final Translate nodeTrackingTranslate = new Translate();

   private final ChangeListener<Transform> nodeTrackingListener = (o, oldTransform, newTransform) ->
   {
      nodeTrackingTranslate.setX(newTransform.getTx());
      nodeTrackingTranslate.setY(newTransform.getTy());
      nodeTrackingTranslate.setZ(newTransform.getTz());
   };

   public CameraNodeTracker(Translate cameraFocusTranslate)
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

   public Node getNodeToTrack()
   {
      return nodeTracked.get();
   }

   public void setNodeToTrack(Node nodeToTrack)
   {
      nodeTracked.set(nodeToTrack);
   }

   public void resetTranslate()
   {
      nodeTrackingTranslate.setX(0.0);
      nodeTrackingTranslate.setY(0.0);
      nodeTrackingTranslate.setZ(0.0);
   }

   public Translate getNodeTrackingTranslate()
   {
      return nodeTrackingTranslate;
   }
}
