package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import org.jetbrains.annotations.NotNull;
import us.ihmc.scs2.session.log.LogDataReaderInterface;
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
   private LogDataReaderInterface addedLogReader;
   private ChangeListener<? super String> mainListener;
   private ChangeListener<? super String> addedListener;

   public void initialize(SessionVisualizerToolkit toolkit, Stage stage, LogDataReaderInterface mainLogReader, LogDataReaderInterface addedLogReader)
   {
      this.toolkit = toolkit;
      this.stage = stage;
      this.addedLogReader = addedLogReader;

      mainListener = getStringChangeListener(mainLogBestMatchLabel, mainLogBestMatchLabelShort, mainLogReader);
      addedListener = getStringChangeListener(addedLogBestMatchLabel, addedLogBestMatchLabelShort, addedLogReader);
      mainLogSearchField.textProperty().addListener(mainListener);
      addedLogSearchField.textProperty().addListener(addedListener);
   }

   private ChangeListener<? super String> getStringChangeListener(Label bestMatchLabel,
                                                                  Label bestMatchLabelShort,
                                                                  LogDataReaderInterface logDataReader)
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

   private static YoVariable findFirstMatch(String query, LogDataReaderInterface logDataReader)
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
         BindSynchronizingVariablesRequest request = new BindSynchronizingVariablesRequest(addedLogReader.getLogDirectory().getAbsolutePath(),
                                                                                           mainLogVarName,
                                                                                           addedLogVarName);
         toolkit.getMessager().submitMessage(toolkit.getTopics().getBindSynchronizingVariablesRequest(), request);
      }

      mainLogSearchField.textProperty().removeListener(mainListener);
      addedLogSearchField.textProperty().removeListener(addedListener);
      stage.close();
   }
}
