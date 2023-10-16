package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import java.io.File;
import java.io.IOException;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.TextArea;
import javafx.scene.layout.AnchorPane;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.mcap.MCAPDebugPrinter;
import us.ihmc.scs2.session.mcap.MCAPLogSession;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class MCAPLogSessionManagerController implements SessionControlsController
{
   private static final String LOG_FILE_KEY = "MCAPLogFilePath";

   @FXML
   private AnchorPane mainPane;
   @FXML
   private Button openSessionButton, endSessionButton;
   @FXML
   private TextArea debugTextArea;

   private final ObjectProperty<MCAPLogSession> activeSessionProperty = new SimpleObjectProperty<>(this, "activeSession", null);

   private Stage stage;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private BackgroundExecutorManager backgroundExecutorManager;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit)
   {
      stage = new Stage();

      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      backgroundExecutorManager = toolkit.getBackgroundExecutorManager();

      openSessionButton.setOnAction(e -> openLogFile());

      endSessionButton.setOnAction(e ->
      {
         MCAPLogSession logSession = activeSessionProperty.get();
         if (logSession != null)
            logSession.shutdownSession();
         activeSessionProperty.set(null);
         debugTextArea.clear();
      });

      stage.setScene(new Scene(mainPane));
      stage.setTitle("MCAP Log session controls");
      stage.getIcons().add(SessionVisualizerIOTools.LOG_SESSION_IMAGE);
      toolkit.getMainWindow().addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            shutdown();
      });
      // TODO Auto-generated method stub
   }

   public void openLogFile()
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setInitialDirectory(SessionVisualizerIOTools.getDefaultFilePath(LOG_FILE_KEY));
      fileChooser.getExtensionFilters().add(new ExtensionFilter("MCAP Log file", "*.mcap"));
      fileChooser.setTitle("Choose MCAP log file");
      File result = fileChooser.showOpenDialog(stage);
      if (result == null)
         return;

      unloadSession();

      backgroundExecutorManager.executeInBackground(() ->
      {
         MCAPLogSession newSession;
         try
         {
            debugTextArea.clear();
            LogTools.info("Creating log session.");
            newSession = new MCAPLogSession(result, new MCAPDebugPrinter()
            {
               @Override
               public void print(String string)
               {
                  JavaFXMissingTools.runLater(getClass(), () -> debugTextArea.appendText(string));
               }
            });
            LogTools.info("Created log session.");
            JavaFXMissingTools.runLater(getClass(), () -> activeSessionProperty.set(newSession));
            SessionVisualizerIOTools.setDefaultFilePath(LOG_FILE_KEY, result);
         }
         catch (IOException ex)
         {
            ex.printStackTrace();
         }
      });
   }

   @Override
   public void notifySessionLoaded()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void unloadSession()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void shutdown()
   {
      // TODO Auto-generated method stub

      stage.close();
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }
}
