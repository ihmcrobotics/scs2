package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.scene.control.Menu;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class SessionMenuController
{
   @FXML
   private Menu menu;

   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      messager.registerJavaFXSyncedTopicListener(topics.getDisableUserControls(), disable -> menu.setDisable(disable));
   }

   @FXML
   void startLogSession()
   {
      messager.submitMessage(topics.getLogSessionControlsRequest(), true);
   }

   @FXML
   void startRemoteSession()
   {
      messager.submitMessage(topics.getRemoteSessionControlsRequest(), true);
   }
}
