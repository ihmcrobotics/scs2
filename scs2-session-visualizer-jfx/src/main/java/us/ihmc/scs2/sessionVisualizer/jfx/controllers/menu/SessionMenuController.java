package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.scene.control.Menu;
import javafx.stage.Stage;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest.SessionType;

public class SessionMenuController implements VisualizerController
{
   @FXML
   private Menu menu;

   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private Stage owner;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      owner = toolkit.getWindow();
      messager.addFXTopicListener(topics.getDisableUserControls(), disable -> menu.setDisable(disable));
   }

   @FXML
   void startLogSession()
   {
      messager.submitMessage(topics.getOpenSessionControlsRequest(), new OpenSessionControlsRequest(owner, SessionType.LOG));
   }

   @FXML
   void startRemoteSession()
   {
      messager.submitMessage(topics.getOpenSessionControlsRequest(), new OpenSessionControlsRequest(owner, SessionType.REMOTE));
   }
}
