package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.stage.Stage;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

import java.io.File;

public class YoCompositeMenuController implements VisualizerController
{
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private Stage owner;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      owner = toolkit.getWindow();
   }

   @FXML
   public void createYoComposite()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), NewWindowRequest.compositeCreatorWindow(owner));
   }
   @FXML
   public void openYoCompositePatternEditor()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), NewWindowRequest.compositePatternEditorWindow(owner));
   }

   @FXML
   public void loadYoCompositePattern()
   {
      File result = SessionVisualizerIOTools.yoCompositeConfigurationOpenFileDialog(owner);
      if (result != null)
         messager.submitMessage(topics.getYoCompositePatternLoadRequest(), result);
   }

   @FXML
   public void saveYoCompositePattern()
   {
      File result = SessionVisualizerIOTools.yoCompositeConfigurationSaveFileDialog(owner);
      if (result != null)
         messager.submitMessage(topics.getYoCompositePatternSaveRequest(), result);
   }

   @FXML
   public void refreshAllYoComposite()
   {
      messager.submitMessage(topics.getYoCompositeRefreshAll(), true);
   }

}
