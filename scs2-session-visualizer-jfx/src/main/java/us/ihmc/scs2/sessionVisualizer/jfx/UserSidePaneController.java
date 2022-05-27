package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.IOException;
import java.net.URL;
import java.util.Objects;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.control.Accordion;
import javafx.scene.control.TitledPane;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.Pane;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class UserSidePaneController implements VisualizerController
{
   @FXML
   private AnchorPane userMainSidePane;
   @FXML
   private Accordion accordion;
   @FXML
   private TitledPane userCustomControlsTitledPane;
   @FXML
   private FlowPane userCustomControlsPane;

   private SessionVisualizerWindowToolkit toolkit;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      accordion.setExpandedPane(userCustomControlsTitledPane);
   }

   public void addControl(Node control)
   {
      userCustomControlsPane.getChildren().add(control);
   }

   public boolean removeControl(Node control)
   {
      return userCustomControlsPane.getChildren().remove(control);
   }

   public void loadCustomPane(String name, URL fxmlResource)
   {
      loadCustomPane(name, fxmlResource, null);
   }

   public void loadCustomPane(String name, URL fxmlResource, VisualizerController controller)
   {
      try
      {
         FXMLLoader fxmlLoader = new FXMLLoader(fxmlResource);
         Pane pane = fxmlLoader.load();
         if (controller != null)
            addCustomPane(name, pane, controller);
         else if (fxmlLoader.getController() instanceof VisualizerController)
            addCustomPane(name, pane, (VisualizerController) fxmlLoader.getController());
         else
            addCustomPane(name, pane);

      }
      catch (IOException e)
      {
         LogTools.error("Couldn't load FXML resource, skipping. Following is the stack-trace:");
         e.printStackTrace();
      }
   }

   public void addCustomPane(String name, Pane pane)
   {
      addCustomPane(name, pane, null);
   }

   public void addCustomPane(String name, Pane pane, VisualizerController controller)
   {
      if (controller != null)
         controller.initialize(toolkit);
      TitledPane titledPane = new TitledPane(name, pane);
      accordion.getPanes().add(titledPane);
      accordion.setExpandedPane(titledPane);
   }

   public boolean removeCustomPane(String name)
   {
      return accordion.getPanes().removeIf(pane -> Objects.equals(pane.getText(), name));
   }

   public AnchorPane getUserMainSidePane()
   {
      return userMainSidePane;
   }
}
