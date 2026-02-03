package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.session.BindSynchronizingVariablesRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class LogVariableComparisonSearchDialogController
{
   @FXML
   private TextField mainLogSearchField;
   @FXML
   private StackPane mainLogSearchStackPane;
   @FXML
   private TextField addedLogSearchField;
   @FXML
   private StackPane addedLogSearchStackPane;
   @FXML
   private Label mainLogBestMatchLabel;
   @FXML
   private Label addedLogBestMatchLabel;
   @FXML
   private Label mainLogBestMatchLabelShort;
   @FXML
   private Label addedLogBestMatchLabelShort;
   @FXML
   private Button selectButton;

   private Stage stage;
   private SessionVisualizerToolkit toolkit;
   private LogDataReader childLogReader;
   private ChangeListener<? super String> mainListener;
   private ChangeListener<? super String> addedListener;

   public void initialize(SessionVisualizerToolkit toolkit, Stage stage, LogDataReader parentLogReader, LogDataReader childLogReader)
   {
      this.toolkit = toolkit;
      this.stage = stage;
      this.childLogReader = childLogReader;

      mainListener = getStringChangeListener(mainLogBestMatchLabel, mainLogBestMatchLabelShort, parentLogReader);
      addedListener = getStringChangeListener(addedLogBestMatchLabel, addedLogBestMatchLabelShort, childLogReader);
      mainLogSearchField.textProperty().addListener(mainListener);
      addedLogSearchField.textProperty().addListener(addedListener);
   }

   private ChangeListener<? super String> getStringChangeListener(Label bestMatchLabel,
                                                                  Label bestMatchLabelShort,
                                                                  LogDataReader logDataReader)
   {
      return (o, oldValue, newValue) ->
      {
         JavaFXMissingTools.runLater(getClass(), () ->
         {
            YoVariable match = findFirstMatch(newValue, logDataReader);
            if (match != null)
            {
               bestMatchLabel.setText(match.getFullNameString());
               bestMatchLabelShort.setText(match.getName());
            }
            else
            {
               bestMatchLabel.setText("N/A");
               bestMatchLabelShort.setText("N/A");
            }
         });
      };
   }

   private static YoVariable findFirstMatch(String query, LogDataReader logDataReader)
   {
      if (query == null || query.isEmpty())
         return null;

      String finalQuery = query.replaceAll("\n", "").trim();
      if (finalQuery.isEmpty())
         return null;

      return logDataReader.getYoVariablesList().stream().filter(v -> v.getName().contains(finalQuery)).findFirst().orElse(null);
   }

   @FXML
   public void select()
   {
      String mainLogVarName = mainLogSearchField.getText();
      String addedLogVarName = addedLogSearchField.getText();

      // If the user hasn't typed anything, we can't search.
      if (mainLogVarName == null || mainLogVarName.trim().isEmpty() || addedLogVarName == null || addedLogVarName.trim().isEmpty())
         return;

      String bestMatchMainName = mainLogBestMatchLabel.getText();
      String bestMatchAddedName = addedLogBestMatchLabel.getText();

      if (!bestMatchMainName.equals("N/A") && !bestMatchAddedName.equals("N/A"))
      {
         BindSynchronizingVariablesRequest request = new BindSynchronizingVariablesRequest(childLogReader.getLogDirectory().getAbsolutePath(),
                                                                                           mainLogVarName,
                                                                                           addedLogVarName);
         toolkit.getMessager().submitMessage(toolkit.getTopics().getBindSynchronizingVariablesRequest(), request);
      }

      mainLogSearchField.textProperty().removeListener(mainListener);
      addedLogSearchField.textProperty().removeListener(addedListener);
      stage.close();
   }
}
