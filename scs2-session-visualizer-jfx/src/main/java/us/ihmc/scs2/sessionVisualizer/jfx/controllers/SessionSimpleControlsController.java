package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.layout.HBox;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionAdvancedControlsController.setupMainControlsActiveMode;

public class SessionSimpleControlsController implements VisualizerController
{
   private SCS2Messager messager;
   private SessionVisualizerTopics topics;
   private SessionVisualizerWindowToolkit toolkit;

   @FXML
   private HBox controlsHBox;
   @FXML
   private Node runningIconView, playbackIconView, pauseIconView;

   public SessionSimpleControlsController()
   {
   }

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      messager.addTopicListener(topics.getShowAdvancedControls(), showAdvancedControls -> show(!showAdvancedControls));
      messager.addFXTopicListener(topics.getDisableUserControls(), disable -> controlsHBox.setDisable(disable));

      setupMainControlsActiveMode(this, toolkit, runningIconView, playbackIconView, pauseIconView);
   }

   @FXML
   private void startRunning()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().setSessionMode(SessionMode.RUNNING);
   }

   @FXML
   private void startPlayback()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().setSessionMode(SessionMode.PLAYBACK);
   }

   @FXML
   private void openAdvancedControls()
   {
      messager.submitMessage(topics.getShowAdvancedControls(), true);
   }

   @FXML
   private void pause()
   {
      if (toolkit.getSession() != null)
         toolkit.getSession().setSessionMode(SessionMode.PAUSE);
   }

   public void show(boolean show)
   {
      controlsHBox.setMouseTransparent(!show);
      controlsHBox.setVisible(show);
   }
}
