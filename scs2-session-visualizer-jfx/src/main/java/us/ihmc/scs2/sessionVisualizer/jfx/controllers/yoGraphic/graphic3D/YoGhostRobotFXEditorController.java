package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.graphic3D;

import javafx.beans.binding.Bindings;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ListCell;
import javafx.scene.control.RadioButton;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleGroup;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.util.Pair;
import javafx.util.StringConverter;
import org.jetbrains.annotations.NotNull;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.yoComposite.YoCompositePatternDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicRobotDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicRobotDefinition.YoOneDoFJointStateDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicRobotDefinition.YoRobotStateDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositePattern;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGhostRobotFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;

import jakarta.xml.bind.JAXBException;
import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

public class YoGhostRobotFXEditorController extends YoGraphicFX3DEditorController<YoGhostRobotFX>
{
   private static final String ROBOT_MODEL_PATH_KEY = "ROBOT_MODEL_PATH";

   @FXML
   private RadioButton robotModelFromSessionRadioButton, robotModelFromFileRadioButton;
   @FXML
   private ComboBox<RobotDefinition> sessionRobotModelsComboBox;
   @FXML
   private TextField loadedRobotModelFileTextField;
   @FXML
   private Button browseRobotModelsButton;
   @FXML
   private YoCompositeEditorPaneController rootJointPositionEditorController;
   @FXML
   private YoCompositeEditorPaneController rootJointOrientationEditorController;
   @FXML
   private Label oneDoFJointsPlaceholderLabel; // We use this label to keep track of where the actual controller goes.
   private YoCompositeEditorPaneController oneDoFJointPositionsEditorController;
   @FXML
   private CheckBox enableColorCheckBox;

   private YoGraphicRobotDefinition definitionBeforeEdits;

   private final Property<RobotDefinition> lastRobotModelFromFile = new SimpleObjectProperty<>(this, "lastRobotModelFromFile", null);
   private final Property<RobotDefinition> selectedRobotModel = new SimpleObjectProperty<>(this, "selectedRobotModel", null);

   private final Property<YoCompositePattern> oneDoFJointPattern = new SimpleObjectProperty<>(this, "currentOneDoFJointPattern", null);

   private final BooleanProperty oneDoFJointPositionValidityProperty = new SimpleBooleanProperty(this, "oneDoFJointPositionValidity", false);

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGhostRobotFX yoGraphicToEdit)
   {
      super.initialize(toolkit, yoGraphicToEdit);
      definitionBeforeEdits = YoGraphicTools.toYoGraphicRobotDefinition(yoGraphicToEdit);
      yoGraphicToEdit.visibleProperty().addListener((observable, oldValue, newValue) -> definitionBeforeEdits.setVisible(newValue));

      robotModelFromSessionRadioButton.setSelected(true);
      new ToggleGroup().getToggles().addAll(robotModelFromSessionRadioButton, robotModelFromFileRadioButton);
      sessionRobotModelsComboBox.disableProperty().bind(robotModelFromFileRadioButton.selectedProperty());
      loadedRobotModelFileTextField.disableProperty().bind(robotModelFromSessionRadioButton.selectedProperty());

      browseRobotModelsButton.disableProperty().bind(robotModelFromSessionRadioButton.selectedProperty());

      sessionRobotModelsComboBox.getItems().setAll(toolkit.getSessionRobotDefinitions());
      if (definitionBeforeEdits.getRobotDefinition() != null)
         sessionRobotModelsComboBox.getSelectionModel().select(definitionBeforeEdits.getRobotDefinition());
      sessionRobotModelsComboBox.setCellFactory(param -> new ListCell<>()
      {
         @Override
         protected void updateItem(RobotDefinition item, boolean empty)
         {
            super.updateItem(item, empty);
            if (empty || item == null)
               setText(null);
            else
               setText(item.getName());
         }
      });

      sessionRobotModelsComboBox.setConverter(new StringConverter<>()
      {
         @Override
         public String toString(RobotDefinition object)
         {
            return object == null ? "null" : object.getName();
         }

         @Override
         public RobotDefinition fromString(String string)
         {
            return sessionRobotModelsComboBox.getItems().stream().filter(r -> r.getName().equals(string)).findFirst().orElse(null);
         }
      });

      sessionRobotModelsComboBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) -> selectedRobotModel.setValue(newValue));
      robotModelFromSessionRadioButton.selectedProperty().addListener((o, oldValue, newValue) ->
                                                                      {
                                                                         if (newValue)
                                                                         {
                                                                            selectedRobotModel.setValue(sessionRobotModelsComboBox.getSelectionModel()
                                                                                                                                  .getSelectedItem());
                                                                         }
                                                                      });
      robotModelFromFileRadioButton.selectedProperty().addListener((o, oldValue, newValue) ->
                                                                   {
                                                                      if (newValue)
                                                                         selectedRobotModel.setValue(lastRobotModelFromFile.getValue());
                                                                   });
      browseRobotModelsButton.setOnAction(e ->
                                          {
                                             Pair<File, RobotDefinition> fileRobotDefinitionPair = loadRobotModelFromFile();
                                             if (fileRobotDefinitionPair != null)
                                             {
                                                lastRobotModelFromFile.setValue(fileRobotDefinitionPair.getValue());
                                                loadedRobotModelFileTextField.setText(fileRobotDefinitionPair.getKey().getAbsolutePath());
                                                selectedRobotModel.setValue(fileRobotDefinitionPair.getValue());
                                             }
                                          });

      selectedRobotModel.addListener((o, oldValue, newValue) ->
                                     {
                                        if (newValue != null)
                                        {
                                           RobotDefinition robotDefinition = new RobotDefinition(newValue);
                                           yoGraphicToEdit.setRobotStateDefinition(null);
                                           yoGraphicToEdit.setRobotDefinition(robotDefinition);
                                           setRobotStateFields(robotDefinition, null);

                                           YoCompositePatternDefinition oneDoFJointPatternDefinition = createOneDoFJointPatternDefinition(newValue);

                                           String newPatternType = oneDoFJointPatternDefinition.getName();
                                           YoCompositePattern currentPattern = oneDoFJointPattern.getValue();

                                           if (currentPattern != null)
                                           {
                                              if (!Objects.equals(newPatternType, currentPattern.getType()))
                                                 yoCompositeSearchManager.discardYoComposite(currentPattern.getType());
                                           }

                                           if (yoCompositeSearchManager.getCollectionFromType(newPatternType) == null)
                                           {
                                              YoCompositePattern newPattern = YoCompositeTools.toYoCompositePattern(oneDoFJointPatternDefinition);
                                              oneDoFJointPattern.setValue(newPattern);
                                              yoCompositeSearchManager.searchYoCompositeInBackground(newPattern, this::setupOneDoFJointPositionEditor);
                                           }
                                           else
                                           {
                                              oneDoFJointPattern.setValue(yoCompositeSearchManager.getPatternFromType(newPatternType));
                                              setupOneDoFJointPositionEditor(yoCompositeSearchManager.getCollectionFromType(newPatternType));
                                           }
                                        }
                                     });

      setupCompositePropertyEditor(rootJointPositionEditorController, "Root Joint Position", false, YoCompositeTools.YO_TUPLE3D, null);
      setupCompositePropertyEditor(rootJointOrientationEditorController, "Root Joint Orientation", false, YoCompositeTools.YO_QUATERNION, null);

      rootJointPositionEditorController.addInputListener(coordinates ->
                                                         {
                                                            YoRobotStateDefinition newState = new YoRobotStateDefinition(yoGraphicToEdit.getRobotStateDefinition());
                                                            newState.setRootJointPosition(CompositePropertyTools.toYoTuple3DDefinition(coordinates));
                                                            yoGraphicToEdit.setRobotStateDefinition(newState);
                                                         });
      rootJointOrientationEditorController.addInputListener(orientation ->
                                                            {
                                                               YoRobotStateDefinition newState = new YoRobotStateDefinition(yoGraphicToEdit.getRobotStateDefinition());
                                                               newState.setRootJointOrientation(CompositePropertyTools.toYoQuaternionDefinition(orientation));
                                                               yoGraphicToEdit.setRobotStateDefinition(newState);
                                                            });

      inputsValidityProperty = Bindings.and(inputsValidityProperty, oneDoFJointPositionValidityProperty);

      enableColorCheckBox.setSelected(definitionBeforeEdits.getColor() != null);
      styleEditorController.getMainPane().setDisable(!enableColorCheckBox.isSelected());
      styleEditorController.bindYoGraphicFX3D(yoGraphicToEdit);
      enableColorCheckBox.selectedProperty().addListener((o, oldValue, newValue) ->
                                                         {
                                                            styleEditorController.getMainPane().setDisable(!newValue);
                                                            if (!newValue)
                                                               yoGraphicToEdit.setColor((BaseColorFX) null);
                                                            else
                                                               yoGraphicToEdit.setColor(styleEditorController.colorProperty().get());
                                                         });

      resetFields();
   }

   private void setupOneDoFJointPositionEditor(YoCompositeCollection collection)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> setupOneDoFJointPositionEditorNow(collection));
   }

   private void setupOneDoFJointPositionEditorNow(YoCompositeCollection collection)
   {
      int index;
      if (oneDoFJointPositionsEditorController != null)
         index = mainPane.getChildren().indexOf(oneDoFJointPositionsEditorController.getMainPane());
      else
         index = mainPane.getChildren().indexOf(oneDoFJointsPlaceholderLabel);

      FXMLLoader fxmlLoader = new FXMLLoader(SessionVisualizerIOTools.YO_COMPOSITE_EDITOR_URL);
      try
      {
         fxmlLoader.load();
         oneDoFJointPositionsEditorController = fxmlLoader.getController();
         setupCompositePropertyEditor(oneDoFJointPositionsEditorController, "One DoF Joint Positions", false, collection.getPattern().getType(), null);

         oneDoFJointPositionsEditorController.addInputListener(positions ->
                                                               {
                                                                  YoRobotStateDefinition newState = new YoRobotStateDefinition(yoGraphicToEdit.getRobotStateDefinition());
                                                                  List<OneDoFJointDefinition> oneDoFJoints = yoGraphicToEdit.getRobotDefinition()
                                                                                                                            .getAllOneDoFJoints();
                                                                  if (oneDoFJoints.size() != positions.length)
                                                                  {
                                                                     LogTools.warn("Mismatch in the number of 1-DoF joints: expected={}, actual={}",
                                                                                   oneDoFJoints.size(),
                                                                                   positions.length);
                                                                     return;
                                                                  }

                                                                  List<YoOneDoFJointStateDefinition> newOneDoFJointStates = new ArrayList<>(oneDoFJoints.size());
                                                                  for (int i = 0; i < oneDoFJoints.size(); i++)
                                                                  {
                                                                     YoOneDoFJointStateDefinition newOneDoFJointState = new YoOneDoFJointStateDefinition();
                                                                     newOneDoFJointState.setJointName(oneDoFJoints.get(i).getName());
                                                                     newOneDoFJointState.setJointPosition(CompositePropertyTools.toDoublePropertyName(positions[i]));
                                                                     newOneDoFJointStates.add(newOneDoFJointState);
                                                                  }
                                                                  newState.setJointPositions(newOneDoFJointStates);
                                                                  yoGraphicToEdit.setRobotStateDefinition(newState);
                                                               });
         oneDoFJointPositionsEditorController.addInputNotification(() -> updateHasChangesPendingProperty(null, null, null));
         oneDoFJointPositionValidityProperty.bind(oneDoFJointPositionsEditorController.inputsValidityProperty());

         mainPane.getChildren().set(index, oneDoFJointPositionsEditorController.getMainPane());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @NotNull
   private static YoCompositePatternDefinition createOneDoFJointPatternDefinition(RobotDefinition newValue)
   {
      YoCompositePatternDefinition oneDoFJointPatternDefinition = new YoCompositePatternDefinition();
      List<OneDoFJointDefinition> oneDoFJoints = newValue.getAllOneDoFJoints();
      oneDoFJointPatternDefinition.setName(newValue.getName() + "OneDoFJoints[%d]".formatted(oneDoFJoints.size()));
      oneDoFJointPatternDefinition.setIdentifiers(oneDoFJoints.stream().map(OneDoFJointDefinition::getName).toArray(String[]::new));
      oneDoFJointPatternDefinition.setCrossRegistry(true);
      return oneDoFJointPatternDefinition;
   }

   @Override
   public void resetFields()
   {
      enableColorCheckBox.setSelected(definitionBeforeEdits.getColor() != null);
      styleEditorController.setInput(definitionBeforeEdits);
      nameEditorController.setInput(definitionBeforeEdits.getName(), yoGraphicToEdit.getNamespace());
      RobotDefinition robotDefinition = definitionBeforeEdits.getRobotDefinition();
      if (robotDefinition != null)
      {
         if (loadedRobotModelFileTextField.getText().isBlank() || !Objects.equals(robotDefinition, lastRobotModelFromFile.getValue()))

         {
            if (!sessionRobotModelsComboBox.getItems().contains(robotDefinition))
            { // This is to handle the case where the graphic is imported, we don't have the info about where the robot model came from.
               boolean hasHomonym = sessionRobotModelsComboBox.getItems().stream().anyMatch(r -> r.getName().equals(robotDefinition.getName()));
               if (hasHomonym)
               {
                  robotDefinition.setName("%s-Imported-DoFs[%d]".formatted(robotDefinition.getName(), robotDefinition.getAllOneDoFJoints().size()));
               }
               sessionRobotModelsComboBox.getItems().add(robotDefinition);
               sessionRobotModelsComboBox.getSelectionModel().select(robotDefinition);
            }
            else
            {
               sessionRobotModelsComboBox.getSelectionModel().select(robotDefinition);
            }
            robotModelFromSessionRadioButton.setSelected(true);
         }
      }
      else
      {
         robotModelFromSessionRadioButton.setSelected(true);
      }

      setRobotStateFields(robotDefinition, definitionBeforeEdits.getRobotStateDefinition());
   }

   private void setRobotStateFields(RobotDefinition robotDefinition, YoRobotStateDefinition robotStateDefinition)
   {
      if (robotDefinition == null)
      {
         rootJointPositionEditorController.clearInput();
         rootJointOrientationEditorController.clearInput();
         if (oneDoFJointPositionsEditorController != null)
         {
            int index = mainPane.getChildren().indexOf(oneDoFJointPositionsEditorController.getMainPane());
            mainPane.getChildren().set(index, oneDoFJointsPlaceholderLabel);
            oneDoFJointPositionsEditorController = null;
         }
         return;
      }

      if (robotStateDefinition == null)
      {
         rootJointPositionEditorController.clearInput();
         rootJointOrientationEditorController.clearInput();
      }
      else
      {
         rootJointPositionEditorController.setInput(robotStateDefinition.getRootJointPosition());
         rootJointOrientationEditorController.setInput(robotStateDefinition.getRootJointOrientation());
      }

      YoCompositePatternDefinition patternDefinition = createOneDoFJointPatternDefinition(robotDefinition);
      if (oneDoFJointPositionsEditorController != null && oneDoFJointPattern.getValue() != null && patternDefinition.getName()
                                                                                                                    .equals(oneDoFJointPattern.getValue()
                                                                                                                                              .getType()))
      {
         if (robotStateDefinition == null)
         {
            oneDoFJointPositionsEditorController.clearInput();
         }
         else
         {
            oneDoFJointPositionsEditorController.setInput(robotStateDefinition.getJointPositions()
                                                                              .stream()
                                                                              .map(YoOneDoFJointStateDefinition::getJointPosition)
                                                                              .toArray(String[]::new));
         }
      }
      else
      {
         YoCompositePattern newPattern = YoCompositeTools.toYoCompositePattern(patternDefinition);
         oneDoFJointPattern.setValue(newPattern);
         if (yoCompositeSearchManager.getCollectionFromType(newPattern.getType()) == null)
         {
            yoCompositeSearchManager.searchYoCompositeNow(newPattern, this::setupOneDoFJointPositionEditor);
         }
         else
         {
            setupOneDoFJointPositionEditor(yoCompositeSearchManager.getCollectionFromType(newPattern.getType()));
         }
      }
      if (robotStateDefinition == null)
      {
         oneDoFJointPositionsEditorController.clearInput();
      }
      else
      {
         oneDoFJointPositionsEditorController.setInput(robotStateDefinition.getJointPositions()
                                                                           .stream()
                                                                           .map(YoOneDoFJointStateDefinition::getJointPosition)
                                                                           .toArray(String[]::new));
      }
   }

   @Override
   public void saveChanges()
   {
      definitionBeforeEdits = YoGraphicTools.toYoGraphicRobotDefinition(yoGraphicToEdit);
      hasChangesPendingProperty.set(false);
   }

   @Override
   public boolean hasChangesPending()
   {
      return super.hasChangesPending();
   }

   @Override
   protected <T> void updateHasChangesPendingProperty(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      hasChangesPendingProperty.set(!definitionBeforeEdits.equals(YoGraphicTools.toYoGraphicRobotDefinition(yoGraphicToEdit)));
   }

   private Pair<File, RobotDefinition> loadRobotModelFromFile()
   {
      File result = SessionVisualizerIOTools.showOpenDialog(toolkit.getMainWindow(),
                                                            "Load Robot Model",
                                                            List.of(new ExtensionFilter("All Model Files", "*.urdf", "*.sdf"),
                                                                    new ExtensionFilter("URDF Files", "*.urdf"),
                                                                    new ExtensionFilter("SDF Files", "*.sdf")),
                                                            ROBOT_MODEL_PATH_KEY);
      if (result == null)
         return null;
      ClassLoader resourceClassLoader = null;
      try
      {
         resourceClassLoader = createResourceClassLoader(result);
      }
      catch (MalformedURLException e)
      {
         throw new RuntimeException(e);
      }

      InputStream inputStream = null;
      try
      {
         inputStream = new BufferedInputStream(new FileInputStream(result));
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }

      if (result.getName().toLowerCase().endsWith("urdf"))
      {
         try
         {
            URDFModel urdfModel = URDFTools.loadURDFModel(inputStream, Collections.emptyList(), resourceClassLoader);
            RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel);
            robotDefinition.setResourceClassLoader(resourceClassLoader);
            return new Pair<>(result, robotDefinition);
         }
         catch (JAXBException e)
         {
            throw new RuntimeException(e);
         }
      }
      else if (result.getName().toLowerCase().endsWith("sdf"))
      {
         try
         {
            SDFRoot sdfRoot = SDFTools.loadSDFRoot(inputStream, Collections.emptyList(), resourceClassLoader);
            RobotDefinition robotDefinition = SDFTools.toFloatingRobotDefinition(sdfRoot.getModels().get(0));
            robotDefinition.setResourceClassLoader(resourceClassLoader);
            return new Pair<>(result, robotDefinition);
         }
         catch (JAXBException e)
         {
            throw new RuntimeException(e);
         }
      }
      else
      {
         return null;
      }
   }

   private static ClassLoader createResourceClassLoader(File modelFile) throws MalformedURLException
   {
      List<URL> resourceURLs = new ArrayList<>();
      resourceURLs.add(modelFile.getParentFile().toURI().normalize().toURL());

      File candidate = modelFile.getParentFile();

      for (int i = 0; i < 3; i++)
      {
         candidate = candidate.getParentFile();

         if (candidate == null)
            break;
         if (candidate.getName().equals("models"))
         {
            resourceURLs.add(candidate.toURI().normalize().toURL());
            break;
         }
      }
      return new URLClassLoader(resourceURLs.toArray(URL[]::new));
   }
}
