package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.function.Predicate;

import javafx.beans.property.ObjectProperty;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.control.MenuItem;
import javafx.scene.input.PickResult;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;

public class CameraTools
{
   public static void setupNodeTrackingContextMenu(FocusBasedCameraMouseEventHandler cameraController, SubScene ownerSubScene)
   {
      setupNodeTrackingContextMenu(cameraController.getNodeTracker().nodeToTrackProperty(), ownerSubScene, node -> true);
   }

   private static void setupNodeTrackingContextMenu(ObjectProperty<Node> nodeTrackedProperty, SubScene ownerSubScene, Predicate<Node> filter)
   {
      ContextMenuTools.setupContextMenu(ownerSubScene, (owner, event) ->
      {
         
         PickResult pickResult = event.getPickResult();
         Node intersectedNode = pickResult.getIntersectedNode();
         if (intersectedNode == null || intersectedNode instanceof SubScene || intersectedNode == nodeTrackedProperty.get() || !filter.test(intersectedNode))
            return null;
         MenuItem menuItem = new MenuItem("Start tracking node: " + intersectedNode.getId());
         menuItem.setOnAction(e -> nodeTrackedProperty.set(intersectedNode));
         return menuItem;
      }, (owner, event) ->
      {
         if (nodeTrackedProperty.get() == null)
            return null;
         MenuItem menuItem = new MenuItem("Stop tracking node: " + nodeTrackedProperty.get().getId());
         menuItem.setOnAction(e -> nodeTrackedProperty.set(null));
         return menuItem;
      });
   }
}
