package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.pattern;

import javafx.animation.AnimationTimer;
import javafx.beans.binding.BooleanBinding;
import javafx.beans.binding.BooleanExpression;
import javafx.beans.property.*;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ListView;
import javafx.scene.control.TextField;
import javafx.scene.control.cell.TextFieldListCell;
import javafx.scene.image.ImageView;
import javafx.scene.layout.VBox;
import javafx.util.converter.DefaultStringConverter;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoChart.YoChartGroupModelDefinition;
import us.ihmc.scs2.definition.yoChart.YoChartIdentifierDefinition;
import us.ihmc.scs2.definition.yoComposite.YoCompositePatternDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.ControllerListCell;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.UIElement;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.YoChartGroupModelEditorController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.IntFunction;
import java.util.stream.Collectors;

import static us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools.*;

public class YoCompositePatternEditorController implements UIElement
{
   private static final String NEW_COMPONENT_IDENTIFIER = "c";

   @FXML
   private VBox mainPane;
   @FXML
   private TextField compositeNameTextField;
   @FXML
   private CheckBox crossRegistryCompositeCheckBox;
   @FXML
   private Button addComponentButton;
   @FXML
   private ListView<String> componentIdentifiersListView;
   @FXML
   private ListView<YoChartGroupModelEditorController> chartGroupModelEditorListView;
   @FXML
   private ImageView patternNameValidImageView;

   private final BooleanProperty patternNameValidityProperty = new SimpleBooleanProperty(this, "patternNameValidity", false);
   private final BooleanProperty componentIdentifiersValidityProperty = new SimpleBooleanProperty(this, "componentIdentifiersValidity", false);
   private final BooleanProperty chartGroupModelsValidityProperty = new SimpleBooleanProperty(this, "chartGroupModelsValidity", true);
   private final BooleanBinding inputsValidityProperty = patternNameValidityProperty.and(componentIdentifiersValidityProperty)
                                                                                    .and(chartGroupModelsValidityProperty);

   private final ObjectProperty<YoCompositePatternDefinition> patternDefinitionProperty = new SimpleObjectProperty<>(this, "patternDefinition", null);
   private YoCompositePatternDefinition definitionBeforeEdits;

   private BooleanProperty hasChangesPendingProperty = new SimpleBooleanProperty(this, "hasChangesPending", false);

   private List<String> nameOfOtherPatterns = new ArrayList<>();

   private SessionVisualizerToolkit toolkit;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;

      compositeNameTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                        {
                                                           YoCompositePatternDefinition newPattern = new YoCompositePatternDefinition(patternDefinitionProperty.get());
                                                           newPattern.setName(newValue);
                                                           patternDefinitionProperty.set(newPattern);
                                                        });

      crossRegistryCompositeCheckBox.selectedProperty().addListener((o, oldValue, newValue) ->
                                                                    {
                                                                       YoCompositePatternDefinition newPattern = new YoCompositePatternDefinition(
                                                                             patternDefinitionProperty.get());
                                                                       newPattern.setCrossRegistry(newValue);
                                                                       patternDefinitionProperty.set(newPattern);
                                                                    });

      componentIdentifiersListView.setCellFactory(param -> new TextFieldListCell<>(new DefaultStringConverter()));
      componentIdentifiersListView.getItems().addListener((ListChangeListener<String>) change ->
      {
         YoCompositePatternDefinition newPattern = new YoCompositePatternDefinition(patternDefinitionProperty.get());
         newPattern.setIdentifiers(change.getList().toArray(String[]::new));
      });

      componentIdentifiersListView.getItems().addListener((ListChangeListener<String>) change ->
      {
         YoCompositePatternDefinition newPattern = new YoCompositePatternDefinition(patternDefinitionProperty.get());
         List<String> newIdentifiers;
         if (newPattern.getIdentifiers() == null)
            newIdentifiers = new ArrayList<>();
         else
            newIdentifiers = new ArrayList<>(Arrays.asList(newPattern.getIdentifiers()));

         while (change.next())
         {
            if (change.wasPermutated())
            {
               for (int oldIndex = change.getFrom(); oldIndex < change.getTo(); oldIndex++)
               {
                  int newIndex = change.getPermutation(oldIndex);
                  Collections.swap(newIdentifiers, oldIndex, newIndex);

                  for (YoChartGroupModelDefinition model : newPattern.getPreferredConfigurations())
                     Collections.swap(model.getChartIdentifiers(), oldIndex, newIndex);
               }
            }
            else if (change.wasUpdated())
            {
               LogTools.error("wasUpdated is not handled!");
            }
            else if (change.wasReplaced())
            { // The component has been renamed
               for (int i = change.getFrom(); i < change.getTo(); i++)
               {
                  newIdentifiers.set(i, change.getList().get(i));
               }
            }
            else
            {
               if (change.wasRemoved())
               {
                  for (int i = 0; i < change.getRemovedSize(); i++)
                  {
                     newIdentifiers.remove(change.getFrom());
                     for (YoChartGroupModelDefinition model : newPattern.getPreferredConfigurations())
                        model.getChartIdentifiers().remove(change.getFrom());
                  }
               }

               if (change.wasAdded())
               {
                  for (int i = change.getFrom(); i < change.getTo(); i++)
                  {
                     newIdentifiers.add(i, change.getList().get(i));
                     for (YoChartGroupModelDefinition model : newPattern.getPreferredConfigurations())
                        model.getChartIdentifiers().add(i, new YoChartIdentifierDefinition());
                  }
               }
            }

            newPattern.setIdentifiers(newIdentifiers.toArray(String[]::new));
            patternDefinitionProperty.set(newPattern);
            ObservableList<YoChartGroupModelEditorController> chartGroupModelControllers = chartGroupModelEditorListView.getItems();

            for (int i = 0; i < chartGroupModelControllers.size(); i++)
            {
               YoChartGroupModelEditorController controller = chartGroupModelControllers.get(i);
               YoChartGroupModelDefinition chartGroupModel = newPattern.getPreferredConfigurations().get(i);
               controller.setInput(chartGroupModel, newIdentifiers);
            }
         }
      });

      chartGroupModelEditorListView.setCellFactory(param -> new ControllerListCell<>());
      chartGroupModelEditorListView.getItems().addListener((ListChangeListener<YoChartGroupModelEditorController>) change ->
      {
         ObservableList<? extends YoChartGroupModelEditorController> newList = change.getList();

         { // Initialize with the new list values.
            YoCompositePatternDefinition newPattern = new YoCompositePatternDefinition(patternDefinitionProperty.get());
            List<YoChartGroupModelDefinition> newPreferredConfigurations = newList.stream()
                                                                                  .map(controller -> controller.chartGroupModelProperty().get())
                                                                                  .collect(Collectors.toList());
            newPattern.setPreferredConfigurations(newPreferredConfigurations);
            patternDefinitionProperty.set(newPattern);
         }

         for (int i = 0; i < newList.size(); i++)
         { // Setup listeners for each individual controller.
            int indexFinal = i;
            newList.get(i).chartGroupModelProperty().addListener((o, oldValue, newValue) ->
                                                                 {
                                                                    YoCompositePatternDefinition updatedPattern = new YoCompositePatternDefinition(
                                                                          patternDefinitionProperty.get());
                                                                    updatedPattern.getPreferredConfigurations().set(indexFinal, newValue);
                                                                    patternDefinitionProperty.set(updatedPattern);
                                                                 });
         }
      });

      setupValidityProperties();

      patternDefinitionProperty.addListener((o, oldValue, newValue) -> hasChangesPendingProperty.set(!definitionBeforeEdits.equals(newValue)));

      IntFunction<String> addAction = index ->
      {
         startEditingComponentIdentifier(index);
         return NEW_COMPONENT_IDENTIFIER + index;
      };
      MenuTools.setupContextMenu(componentIdentifiersListView,
                                 addBeforeMenuItemFactory(addAction),
                                 addAfterMenuItemFactory(addAction),
                                 removeMenuItemFactory(false));
      MenuTools.setupContextMenu(chartGroupModelEditorListView,
                                 addBeforeMenuItemFactory(() -> newYoChartGroupModelEditor()),
                                 addAfterMenuItemFactory(() -> newYoChartGroupModelEditor()),
                                 removeMenuItemFactory(false));

      setPrefHeight();
   }

   private void setupValidityProperties()
   {
      YoGraphicFXControllerTools.bindValidityImageView(patternNameValidityProperty, patternNameValidImageView);

      compositeNameTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                        {
                                                           if (newValue == null || newValue.isEmpty())
                                                              patternNameValidityProperty.set(false);
                                                           else
                                                              patternNameValidityProperty.set(!nameOfOtherPatterns.contains(newValue));
                                                        });

      componentIdentifiersListView.setStyle(YoCompositePatternControllerTools.getValidityStyleBorder(componentIdentifiersValidityProperty.get()));
      componentIdentifiersValidityProperty.addListener((o, oldValue, newValue) -> componentIdentifiersListView.setStyle(YoCompositePatternControllerTools.getValidityStyleBorder(
            newValue)));

      componentIdentifiersListView.getItems()
                                  .addListener((ListChangeListener<String>) change -> componentIdentifiersValidityProperty.set(YoCompositePatternControllerTools.areComponentIdentifierNamesValid(
                                        change.getList())));

      chartGroupModelEditorListView.setStyle(YoCompositePatternControllerTools.getValidityStyleBorder(chartGroupModelsValidityProperty.get()));
      chartGroupModelsValidityProperty.addListener((o, oldValue, newValue) -> chartGroupModelEditorListView.setStyle(YoCompositePatternControllerTools.getValidityStyleBorder(
            newValue)));

      chartGroupModelEditorListView.getItems().addListener((ListChangeListener<YoChartGroupModelEditorController>) change ->
      {
         if (change.getList().isEmpty())
         {
            chartGroupModelsValidityProperty.set(true);
            return;
         }

         while (change.next() && change.wasAdded())
         {
            for (YoChartGroupModelEditorController controller : change.getAddedSubList())
            {
               controller.configurationNameProperty().addListener((o, oldValue, newValue) ->
                                                                  {
                                                                     chartGroupModelsValidityProperty.set(YoCompositePatternControllerTools.areChartGroupModelNamesValid(
                                                                           chartGroupModelEditorListView.getItems()));
                                                                  });
            }
         }

         chartGroupModelsValidityProperty.set(YoCompositePatternControllerTools.areChartGroupModelNamesValid(change.getList()));
      });
   }

   public void setInput(YoCompositePatternDefinition definitionBeforeEdits)
   {
      this.definitionBeforeEdits = definitionBeforeEdits;
      resetFields();
   }

   public void saveChanges()
   {
      definitionBeforeEdits = new YoCompositePatternDefinition(patternDefinitionProperty.get());
      hasChangesPendingProperty.set(false);
   }

   public void resetFields()
   {
      compositeNameTextField.setText(definitionBeforeEdits.getName());
      crossRegistryCompositeCheckBox.setSelected(definitionBeforeEdits.isCrossRegistry());

      ObservableList<String> idsListItems = componentIdentifiersListView.getItems();
      String[] identifiers = definitionBeforeEdits.getIdentifiers();
      int numberOfIdentifiers = identifiers == null ? 0 : identifiers.length;
      while (idsListItems.size() < numberOfIdentifiers)
         addComponent(false);
      while (idsListItems.size() > numberOfIdentifiers)
         idsListItems.remove(idsListItems.size() - 1);
      for (int i = 0; i < idsListItems.size(); i++)
         idsListItems.set(i, identifiers[i]);

      ObservableList<YoChartGroupModelEditorController> modelListItems = chartGroupModelEditorListView.getItems();
      while (modelListItems.size() < definitionBeforeEdits.getPreferredConfigurations().size())
         addChartGroupModel();
      while (modelListItems.size() > definitionBeforeEdits.getPreferredConfigurations().size())
      {
         YoChartGroupModelEditorController removedController = modelListItems.remove(modelListItems.size() - 1);
         removedController.closeAndDispose();
      }
      List<String> identifiersList = identifiers == null ? Collections.emptyList() : Arrays.asList(identifiers);
      for (int i = 0; i < modelListItems.size(); i++)
         modelListItems.get(i).setInput(definitionBeforeEdits.getPreferredConfigurations().get(i), identifiersList);
   }

   private AnimationTimer prefHeightAdjustmentAnimation;

   private void setPrefHeight()
   {
      if (prefHeightAdjustmentAnimation == null)
      {
         prefHeightAdjustmentAnimation = new ObservedAnimationTimer(getClass().getSimpleName())
         {
            @Override
            public void handleImpl(long now)
            {
               if (componentIdentifiersListView.getItems().isEmpty())
               {
                  componentIdentifiersListView.setMinHeight(0.0);
                  componentIdentifiersListView.setMaxHeight(0.0);
                  componentIdentifiersListView.setPrefHeight(0.0);
               }
               else
               {
                  double minHeight = 24.0;
                  int size = componentIdentifiersListView.getItems().size();
                  componentIdentifiersListView.setMinHeight(0.5 * size * minHeight);
                  componentIdentifiersListView.setPrefHeight(size * minHeight);
                  componentIdentifiersListView.setMaxHeight(2.0 * size * minHeight);
               }

               if (chartGroupModelEditorListView.getItems().isEmpty())
               {
                  chartGroupModelEditorListView.setMinHeight(0.0);
                  chartGroupModelEditorListView.setMaxHeight(0.0);
                  chartGroupModelEditorListView.setPrefHeight(0.0);
               }
               else
               {
                  double minHeight = chartGroupModelEditorListView.getItems().get(0).getMainPane().getHeight() + 10.0;
                  chartGroupModelEditorListView.setMinHeight(minHeight);
                  chartGroupModelEditorListView.setPrefHeight(minHeight);
                  chartGroupModelEditorListView.setMaxHeight(2.0 * minHeight);
               }
            }
         };
      }

      prefHeightAdjustmentAnimation.start();
   }

   public YoCompositePatternDefinition getDefinitionBeforeEdits()
   {
      return definitionBeforeEdits;
   }

   public ObjectProperty<YoCompositePatternDefinition> patternDefinitionProperty()
   {
      return patternDefinitionProperty;
   }

   public void startEditingCompositePatternName()
   {
      compositeNameTextField.requestFocus();
   }

   @FXML
   private void addComponent()
   {
      addComponent(true);
   }

   private void addComponent(boolean startEditing)
   {
      int newItemIndex = componentIdentifiersListView.getItems().size();
      componentIdentifiersListView.getItems().add(newItemIndex, NEW_COMPONENT_IDENTIFIER + newItemIndex);
      if (startEditing)
         startEditingComponentIdentifier(newItemIndex);
   }

   public void startEditingComponentIdentifier(int editIndex)
   {
      JavaFXMissingTools.runNFramesLater(1, () ->
      {
         componentIdentifiersListView.requestFocus();
         componentIdentifiersListView.getSelectionModel().select(editIndex);
         componentIdentifiersListView.edit(editIndex);
      });
   }

   @FXML
   private void addChartGroupModel()
   {
      chartGroupModelEditorListView.getItems().add(newYoChartGroupModelEditor());
   }

   private YoChartGroupModelEditorController newYoChartGroupModelEditor()
   {
      YoChartGroupModelDefinition initialModel = new YoChartGroupModelDefinition("New Chart Group Model");
      while (initialModel.getChartIdentifiers().size() < patternDefinitionProperty.get().getIdentifiers().length)
         initialModel.getChartIdentifiers().add(new YoChartIdentifierDefinition());
      return newYoChartGroupModelEditor(initialModel);
   }

   private YoChartGroupModelEditorController newYoChartGroupModelEditor(YoChartGroupModelDefinition initialModel)
   {
      FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CHART_GROUP_MODEL_EDITOR_PANE_URL);
      try
      {
         loader.load();
         YoChartGroupModelEditorController editor = loader.getController();
         editor.initialize(toolkit);
         if (initialModel != null)
            editor.setInput(initialModel, Arrays.asList(patternDefinitionProperty.get().getIdentifiers()));
         JavaFXMissingTools.runNFramesLater(1, () -> editor.startEditingChartGroupModelName());
         return editor;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public void setNameOfOtherPatterns(List<String> names)
   {
      nameOfOtherPatterns.clear();
      nameOfOtherPatterns.addAll(names);
   }

   public BooleanExpression inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public boolean hasChangesPending()
   {
      return hasChangesPendingProperty.get();
   }

   public ReadOnlyBooleanProperty hasChangesPendingProperty()
   {
      return hasChangesPendingProperty;
   }

   public void closeAndDispose()
   {
      prefHeightAdjustmentAnimation.stop();
   }

   @Override
   public VBox getMainPane()
   {
      return mainPane;
   }
}
