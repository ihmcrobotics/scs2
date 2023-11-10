package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.ListView;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;

public class YoCompositeCreatorWindowController
{
   private static final String NEW_COMPOSITE_NAME = "NewVariable";

   @FXML
   private Pane mainPane;
   @FXML
   private ListView<YoCompositeSymbolicController> yoCompositeListView;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
   }

   @FXML
   public void newYoComposite(ActionEvent actionEvent)
   {
   }

   @FXML
   public void deleteYoComposite(ActionEvent actionEvent)
   {
   }

   public Pane getMainPane()
   {
      return mainPane;
   }

   public void showWindow()
   {
   }

   public Stage getWindow()
   {
   }

   public static class YoCompositeSymbolicController
   {
      @FXML
      private Pane mainPane;
      @FXML
      private TextField yoCompositeNameTextField;
      @FXML
      private TextArea yoCompositeEquationTextArea;

      public void initialize(SessionVisualizerToolkit toolkit)
      {
      }

      public Pane getMainPane()
      {
         return mainPane;
      }
   }
}
