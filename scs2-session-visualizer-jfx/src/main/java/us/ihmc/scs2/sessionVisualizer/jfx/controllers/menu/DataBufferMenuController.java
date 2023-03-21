package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.Property;
import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.Menu;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.util.converter.IntegerStringConverter;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.FillBufferRequest;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

public class DataBufferMenuController implements VisualizerController
{
   @FXML
   private Menu menu;
   @FXML
   private CustomMenuItem bufferSizeMenuItem, bufferRecordTickPeriodMenuItem, numberPrecisionMenuItem;
   @FXML
   private TextField bufferSizeTextField;
   @FXML
   private TextField bufferRecordTickPeriodTextField;
   @FXML
   private Spinner<Integer> numberPrecisionSpinner;
   @FXML
   private CheckMenuItem showSCS2YoVariablesMenuItem;

   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;

   private boolean initializeBufferSizeTextField = true;
   private Property<YoBufferPropertiesReadOnly> bufferProperties;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      bufferProperties = messager.createPropertyInput(topics.getYoBufferCurrentProperties(), null);
      messager.addTopicListener(topics.getSessionCurrentState(), m ->
      {
         if (m == SessionState.INACTIVE)
            initializeBufferSizeTextField = true;
      });
      messager.addFXTopicListener(topics.getDisableUserControls(), disable -> menu.setDisable(disable));

      TextFormatter<Integer> bufferSizeFormatter = new TextFormatter<>(new IntegerStringConverter(), 0, new PositiveIntegerValueFilter());
      bufferSizeTextField.setTextFormatter(bufferSizeFormatter);

      MenuTools.configureTextFieldForCustomMenuItem(bufferSizeMenuItem, bufferSizeTextField);
      MenuTools.configureTextFieldForCustomMenuItem(bufferRecordTickPeriodMenuItem, bufferRecordTickPeriodTextField);

      MutableBoolean updatingBufferResize = new MutableBoolean(false);

      bufferProperties.addListener((o, oldValue, newValue) ->
      {
         if (!initializeBufferSizeTextField && (oldValue == null || newValue.getSize() == oldValue.getSize()))
            return;

         if (updatingBufferResize.isFalse())
         {
            updatingBufferResize.setTrue();
            bufferSizeFormatter.setValue(newValue.getSize());
            initializeBufferSizeTextField = false;
            updatingBufferResize.setFalse();
         }
      });

      bufferSizeFormatter.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (bufferProperties.getValue() != null && bufferProperties.getValue().getSize() == newValue.intValue())
            return;

         if (updatingBufferResize.isFalse())
         {
            updatingBufferResize.setTrue();
            messager.submitMessage(topics.getYoBufferCurrentSizeRequest(), newValue);
            updatingBufferResize.setFalse();
         }
      });

      TextFormatter<Integer> recordPeriodFormatter = new TextFormatter<>(new IntegerStringConverter(), 0, new PositiveIntegerValueFilter());
      bufferRecordTickPeriodTextField.setTextFormatter(recordPeriodFormatter);

      messager.bindBidirectional(topics.getBufferRecordTickPeriod(), recordPeriodFormatter.valueProperty(), false);

      IntegerSpinnerValueFactory numberPrecisionSpinnerValueFactory = new IntegerSpinnerValueFactory(1, 30, 3, 1);
      numberPrecisionSpinner.setValueFactory(numberPrecisionSpinnerValueFactory);
      if (numberPrecisionSpinner.isEditable())
      {
         numberPrecisionSpinner.focusedProperty().addListener((o, oldValue, newValue) ->
         {
            if (!newValue)
            { // Losing focus
              // Workaround: manually reset to the current value
               numberPrecisionSpinner.getEditor()
                                     .setText(numberPrecisionSpinnerValueFactory.getConverter().toString(numberPrecisionSpinnerValueFactory.getValue()));
            }
         });
      }
      messager.bindBidirectional(topics.getControlsNumberPrecision(), numberPrecisionSpinnerValueFactory.valueProperty(), false);

      messager.bindBidirectional(topics.getShowSCS2YoVariables(), showSCS2YoVariablesMenuItem.selectedProperty(), false);
   }

   @FXML
   private void requestCropDataBuffer()
   {
      if (bufferProperties.getValue() != null)
      {
         CropBufferRequest cropBufferRequest = new CropBufferRequest(bufferProperties.getValue().getInPoint(), bufferProperties.getValue().getOutPoint());
         messager.submitMessage(topics.getYoBufferCropRequest(), cropBufferRequest);
      }
   }

   @FXML
   private void requestFlushDataBuffer()
   {
      YoBufferPropertiesReadOnly properties = bufferProperties.getValue();
      if (properties != null)
      {
         FillBufferRequest fillBufferRequest = new FillBufferRequest(false,
                                                                     SharedMemoryTools.increment(properties.getOutPoint(), 1, properties.getSize()),
                                                                     SharedMemoryTools.decrement(properties.getInPoint(), 1, properties.getSize()));
         messager.submitMessage(topics.getYoBufferFillRequest(), fillBufferRequest);
      }
   }
}
