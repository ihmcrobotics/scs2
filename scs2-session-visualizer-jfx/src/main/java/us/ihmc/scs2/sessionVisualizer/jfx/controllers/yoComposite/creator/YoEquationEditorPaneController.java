package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator;

import com.jfoenix.controls.JFXTextField;
import javafx.beans.binding.Bindings;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableMap;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.image.ImageView;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.util.Pair;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationAliasDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationInputDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.StringSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools;
import us.ihmc.scs2.symbolic.Equation;
import us.ihmc.scs2.symbolic.parser.EquationAliasManager.EquationAlias;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

public class YoEquationEditorPaneController
{
   @FXML
   private Pane mainPane;
   @FXML
   private TextField equationNameTextField;
   @FXML
   private TextArea equationTextArea;
   @FXML
   private ListView<YoEquationAliasController> aliasListView;
   @FXML
   private ImageView equationNameValidImageView, equationValidImageView, equationAliasesValidImageView;

   private final ObservableMap<String, YoEquationAliasController> aliasNameToControllerMap = FXCollections.observableHashMap();
   private final Property<YoEquationDefinition> definitionToEdit = new SimpleObjectProperty<>(this, "definitionToEdit", null);
   private final BooleanProperty equationValidityProperty = new SimpleBooleanProperty(this, "equationValidity", false);
   private final BooleanProperty equationNameValidityProperty = new SimpleBooleanProperty(this, "equationNameValidity", false);
   private final BooleanProperty equationAliasesValidityProperty = new SimpleBooleanProperty(this, "equationAliasesValidity", false);
   private ObservableBooleanValue validityProperty;
   private SessionVisualizerToolkit toolkit;
   private Runnable updateListener;

   public void initialize(SessionVisualizerToolkit toolkit, Predicate<YoEquationDefinition> nameValidator)
   {
      this.toolkit = toolkit;

      MutableBoolean isUpdatingDefinition = new MutableBoolean(false);

      definitionToEdit.addListener((o, oldValue, newValue) ->
                                   {
                                      isUpdatingDefinition.setTrue();

                                      try
                                      {
                                         equationNameTextField.setText(newValue.getName());
                                         equationNameValidityProperty.set(!newValue.getName().isBlank() && nameValidator.test(newValue));
                                         equationTextArea.setText(newValue.getEquation());

                                         List<EquationAliasDefinition> aliases = newValue.getAliases();
                                         setAliases(aliases.stream().map(a -> new Pair<>(a.getName(), a.getValue())).collect(Collectors.toList()), true);

                                         // FIXME Kinda hackish: when loading the equation from file, we don't want to update the equation in case there's a missing yoVariable, but making a new equation we do.
                                         updateEquation(aliases.isEmpty(), false);
                                      }
                                      finally
                                      {
                                         isUpdatingDefinition.setFalse();
                                      }
                                   });

      equationNameTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                       {
                                                          if (definitionToEdit.getValue() == null)
                                                             return;

                                                          if (isUpdatingDefinition.isTrue())
                                                             return;

                                                          YoEquationDefinition definition = definitionToEdit.getValue();
                                                          definition.setName(newValue);
                                                          equationNameValidityProperty.set(!newValue.isBlank() && nameValidator.test(definition));
                                                          updateListener.run();
                                                       });

      equationTextArea.textProperty().addListener((o, oldValue, newValue) ->
                                                  {
                                                     if (definitionToEdit.getValue() == null || isUpdatingDefinition.isTrue())
                                                        return;

                                                     YoEquationDefinition definition = definitionToEdit.getValue();
                                                     definition.setEquation(newValue);
                                                     updateEquation(true, false);
                                                  });

      aliasListView.setCellFactory(param -> new YoEquationAliasListCell());
      aliasListView.getItems().addListener(((ListChangeListener<YoEquationAliasController>) c ->
      {
         aliasNameToControllerMap.clear();
         aliasNameToControllerMap.putAll(aliasListView.getItems().stream().collect(Collectors.toMap(a -> a.aliasNameLabel.getText(), a -> a)));

         equationAliasesValidityProperty.unbind();
         if (c.getList().isEmpty())
         {
            equationAliasesValidityProperty.set(true);
         }
         else
         {
            ObservableBooleanValue aliasInputValidity = aliasListView.getItems().get(0).aliasValueSearchField.getValidityProperty();
            for (int i = 1; i < c.getList().size(); i++)
               aliasInputValidity = Bindings.and(aliasInputValidity, aliasListView.getItems().get(i).aliasValueSearchField.getValidityProperty());
            equationAliasesValidityProperty.bind(aliasInputValidity);
         }
      }));
      validityProperty = Bindings.createBooleanBinding(() -> equationValidityProperty.get() && equationNameValidityProperty.get()
                                                             && equationAliasesValidityProperty.get(),
                                                       equationValidityProperty,
                                                       equationNameValidityProperty,
                                                       equationAliasesValidityProperty);
      YoGraphicFXControllerTools.bindValidityImageView(equationNameValidityProperty, equationNameValidImageView);
      YoGraphicFXControllerTools.bindValidityImageView(equationValidityProperty, equationValidImageView);
      YoGraphicFXControllerTools.bindValidityImageView(equationAliasesValidityProperty, equationAliasesValidImageView);
   }

   private void updateEquation(boolean updateAliases, boolean removeUnusedAliases)
   {
      if (definitionToEdit.getValue() == null)
         return;

      YoEquationDefinition definition = definitionToEdit.getValue();
      Equation equation;
      try
      {
         equation = Equation.parse(definition.getEquation());
         equationValidityProperty.set(true);
      }
      catch (Exception e)
      {
         LogTools.error("Unable to parse equation: {}", e.getMessage());
         equationValidityProperty.set(false);
         updateListener.run();
         return;
      }

      if (updateAliases)
         updateAliases(equation, removeUnusedAliases);
      updateListener.run();
   }

   private void updateAliases(Equation equation, boolean removeUnusedAliases)
   {
      List<Pair<String, EquationInputDefinition>> newAliases = new ArrayList<>();

      for (EquationAlias alias : equation.getBuilder().getAliasManager().getUserAliases().values())
      {
         String aliasName = alias.name();
         EquationInputDefinition aliasValue = alias.input().toInputDefinition();
         newAliases.add(new Pair<>(aliasName, aliasValue));
      }

      for (String aliasName : equation.getBuilder().getAliasManager().getMissingInputs())
      {
         YoEquationAliasController aliasController = aliasNameToControllerMap.get(aliasName);
         EquationInputDefinition aliasValue = aliasController == null ? null : aliasController.getAliasValue();
         newAliases.add(new Pair<>(aliasName, aliasValue));
      }

      newAliases.sort(Comparator.comparing(Pair::getKey));

      setAliases(newAliases, removeUnusedAliases);
   }

   private void setAliases(List<Pair<String, EquationInputDefinition>> newAliases, boolean removeOldAliases)
   {
      if (removeOldAliases)
      {
         aliasListView.getItems().clear();

         for (Pair<String, EquationInputDefinition> alias : newAliases)
         {
            String aliasName = alias.getKey();
            EquationInputDefinition aliasValue = alias.getValue();
            aliasListView.getItems().add(newAliasController(aliasName, aliasValue));
         }
      }
      else
      {
         List<YoEquationAliasController> newAliasControllers = new ArrayList<>(aliasListView.getItems());

         for (Pair<String, EquationInputDefinition> alias : newAliases)
         {
            String aliasName = alias.getKey();
            EquationInputDefinition aliasValue = alias.getValue();
            YoEquationAliasController aliasController = aliasNameToControllerMap.get(aliasName);

            if (aliasController == null)
               newAliasControllers.add(newAliasController(aliasName, aliasValue));
            else
               aliasController.setAliasValue(aliasValue);
         }

         newAliasControllers.sort(Comparator.comparing(a -> a.aliasNameLabel.getText()));
         aliasListView.getItems().setAll(newAliasControllers);
      }
   }

   private YoEquationAliasController newAliasController(String aliasName, EquationInputDefinition aliasValue)
   {
      YoCompositeSearchManager yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();
      YoEquationAliasController aliasController = new YoEquationAliasController(aliasName, aliasValue, yoCompositeSearchManager);
      aliasController.aliasValueSearchField.supplierProperty().addListener((o, oldValue, newValue) ->
                                                                           {
                                                                              if (newValue != null)
                                                                                 updateAliasValue(aliasController);
                                                                           });
      return aliasController;
   }

   private void updateAliasValue(YoEquationAliasController aliasController)
   {
      if (definitionToEdit.getValue() == null)
         return;

      String aliasName = aliasController.getAliasName();
      List<EquationAliasDefinition> definitionAliases = definitionToEdit.getValue().getAliases();
      EquationAliasDefinition newAlias = new EquationAliasDefinition(aliasName, aliasController.getAliasValue());
      EquationAliasDefinition old = definitionAliases.stream().filter(a -> a.getName().equals(aliasName)).findFirst().orElse(null);

      if (old != null)
         definitionAliases.set(definitionAliases.indexOf(old), newAlias);
      else
         definitionAliases.add(newAlias);
      updateListener.run();
   }

   @FXML
   public void removeUnusedAliases()
   {
      updateEquation(true, true);
   }

   public Pane getMainPane()
   {
      return mainPane;
   }

   public TextField getEquationNameTextField()
   {
      return equationNameTextField;
   }

   public TextArea getEquationTextArea()
   {
      return equationTextArea;
   }

   public Property<YoEquationDefinition> definitionProperty()
   {
      return definitionToEdit;
   }

   public YoEquationDefinition getDefinition()
   {
      return definitionToEdit.getValue();
   }

   public ObservableBooleanValue validityProperty()
   {
      return validityProperty;
   }

   public void setUpdateListener(Runnable updateListener)
   {
      this.updateListener = updateListener;
   }

   private static class YoEquationAliasListCell extends ListCell<YoEquationAliasController>
   {
      public YoEquationAliasListCell()
      {
      }

      @Override
      protected void updateItem(YoEquationAliasController item, boolean empty)
      {
         super.updateItem(item, empty);

         if (item != null && !empty)
            setGraphic(item.getMainPane());
         else
            setGraphic(null);
      }
   }

   private static class YoEquationAliasController
   {
      private final Label aliasNameLabel;
      private final StringSearchField aliasValueSearchField;
      private final HBox mainPane = new HBox(5);

      public YoEquationAliasController(String aliasName, EquationInputDefinition aliasValue, YoCompositeSearchManager searchManager)
      {
         aliasNameLabel = new Label();
         setAliasName(aliasName);
         JFXTextField textField = new JFXTextField();
         mainPane.getChildren().addAll(aliasNameLabel, textField);

         aliasValueSearchField = new StringSearchField(textField, searchManager);
         setAliasValue(aliasValue);
         aliasValueSearchField.setupAutoCompletion();
         aliasValueSearchField.supplierProperty().addListener((o, oldValue, newValue) ->
                                                              {
                                                                 if (newValue == null)
                                                                    LogTools.info("Alias {} is not valid.", aliasName);
                                                              });
      }

      public void setAliasName(String aliasName)
      {
         aliasNameLabel.setText(aliasName);
      }

      private void setAliasValue(EquationInputDefinition aliasValue)
      {
         if (aliasValue == null)
            aliasValueSearchField.getTextField().setText("");
         else
            aliasValueSearchField.getTextField().setText(aliasValue.computeSimpleStringValue());
      }

      public Pane getMainPane()
      {
         return mainPane;
      }

      public String getAliasName()
      {
         return aliasNameLabel.getText();
      }

      public EquationInputDefinition getAliasValue()
      {
         if (aliasValueSearchField.supplierProperty().get() == null)
            return null;
         if (aliasValueSearchField.isNumber())
            return new EquationInputDefinition(aliasValueSearchField.supplierProperty().get().getValue());
         if (aliasValueSearchField.isYoComposite())
         {
            if (aliasValueSearchField.getYoComposite().getYoComponents().size() == 1)
               return new EquationInputDefinition(SharedMemoryIOTools.toYoVariableDefinition(aliasValueSearchField.getYoComposite().getYoComponents().get(0)));
            else
               LogTools.warn("Complex YoComposites are not handled yet, {}", aliasValueSearchField.getYoComposite().getPattern());
         }
         return null;
      }
   }
}
