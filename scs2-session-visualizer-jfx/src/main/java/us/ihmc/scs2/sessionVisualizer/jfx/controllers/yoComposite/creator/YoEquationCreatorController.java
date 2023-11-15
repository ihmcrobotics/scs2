package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator;

import com.jfoenix.controls.JFXTextField;
import javafx.beans.binding.Bindings;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableMap;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.image.ImageView;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.util.Pair;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationAliasDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.StringSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.symbolic.Equation;
import us.ihmc.scs2.symbolic.YoEquationManager.YoEquationListChange;
import us.ihmc.scs2.symbolic.parser.EquationAliasManager.EquationAlias;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.stream.Collectors;

public class YoEquationCreatorController
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

   private ObservableMap<String, YoEquationAliasController> aliasNameToControllerMap = FXCollections.observableHashMap();
   private YoEquationDefinition definition;
   private final BooleanProperty equationValidityProperty = new SimpleBooleanProperty(this, "equationValidity", false);
   private final BooleanProperty equationNameValidityProperty = new SimpleBooleanProperty(this, "equationNameValidity", false);
   private final BooleanProperty equationAliasesValidityProperty = new SimpleBooleanProperty(this, "equationAliasesValidity", false);
   private ObservableBooleanValue validityProperty;
   private SessionVisualizerToolkit toolkit;
   private Consumer<YoEquationDefinition> updateListener;

   public void initialize(SessionVisualizerToolkit toolkit, Predicate<String> nameValidator, YoEquationDefinition definition)
   {
      this.toolkit = toolkit;
      this.definition = definition;
      equationNameTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                       {
                                                          definition.setName(newValue);
                                                          equationNameValidityProperty.set(!newValue.isBlank() && nameValidator.test(newValue));
                                                          updateListener.accept(definition);
                                                       });

      equationTextArea.textProperty().addListener((o, oldValue, newValue) ->
                                                  {
                                                     definition.setEquation(newValue);
                                                     updateEquation();
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

   private void updateEquation()
   {
      Equation equation;
      try
      {
         equation = Equation.parse(definition.getEquation());
         equationValidityProperty.set(true);
      }
      catch (Exception e)
      {
         LogTools.error("Unable to parse equation: {}", e.getMessage());
         e.printStackTrace();
         equationValidityProperty.set(false);
         return;
      }

      List<Pair<String, String>> newAliases = new ArrayList<>();

      for (EquationAlias alias : equation.getBuilder().getAliasManager().getUserAliases().values())
      {
         String aliasName = alias.name();
         String aliasValue = alias.input().valueAsString();
         newAliases.add(new Pair<>(aliasName, aliasValue));
      }

      for (String aliasName : equation.getBuilder().getAliasManager().getMissingInputs())
      {
         YoEquationAliasController aliasController = aliasNameToControllerMap.get(aliasName);
         String aliasValue;
         if (aliasController == null || aliasController.aliasValueSearchField.getSupplier() == null)
            aliasValue = null;
         else
            aliasValue = aliasController.aliasValueSearchField.getSupplier().getValue();

         newAliases.add(new Pair<>(aliasName, aliasValue));
      }

      newAliases.sort((p1, p2) -> p1.getKey().compareTo(p2.getKey()));

      YoCompositeSearchManager yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();
      LinkedYoRegistry linkedRootRegistry = toolkit.getYoManager().getLinkedRootRegistry();
      List<EquationAliasDefinition> definitionAliases = definition.getAliases();

      aliasListView.getItems().clear();

      for (Pair<String, String> alias : newAliases)
      {
         String aliasName = alias.getKey();
         String aliasValue = alias.getValue();
         YoEquationAliasController aliasController = new YoEquationAliasController(aliasName, aliasValue, yoCompositeSearchManager, linkedRootRegistry);
         aliasController.aliasValueSearchField.supplierProperty().addListener((o, oldValue, newValue) ->
                                                                              {
                                                                                 if (newValue != null)
                                                                                 {
                                                                                    definitionAliases.removeIf(a -> a.getName().equals(aliasName));
                                                                                    definitionAliases.add(new EquationAliasDefinition(aliasName,
                                                                                                                                      newValue.getValue()));
                                                                                    updateListener.accept(definition);
                                                                                 }
                                                                              });
         aliasListView.getItems().add(aliasController);
      }
      updateListener.accept(definition);
   }

   public Pane getMainPane()
   {
      return mainPane;
   }

   public YoEquationDefinition getEquationDefinition()
   {
      return definition;
   }

   public ObservableBooleanValue validityProperty()
   {
      return validityProperty;
   }

   public void setEquationDefinition(YoEquationDefinition definition)
   {
      this.definition.set(definition);
      if (!Objects.equals(equationNameTextField.getText(), definition.getName()))
         equationNameTextField.setText(definition.getName());
      if (!Objects.equals(equationTextArea.getText().trim(), definition.getEquation().trim()))
         equationTextArea.setText(definition.getEquation());
      updateEquation();
   }

   public void setUpdateListener(Consumer<YoEquationDefinition> updateListener)
   {
      this.updateListener = updateListener;
   }

   @FXML
   public void commitEquation(ActionEvent actionEvent)
   {
      Messager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();
      messager.submitMessage(topics.getSessionYoEquationListChangeRequest(), YoEquationListChange.add(definition));
   }

   @FXML
   public void deleteEquation(ActionEvent actionEvent)
   {
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

      public YoEquationAliasController(String aliasName, String aliasValue, YoCompositeSearchManager searchManager, LinkedYoRegistry linkedRootRegistry)
      {
         aliasNameLabel = new Label(aliasName);
         JFXTextField textField = new JFXTextField();
         mainPane.getChildren().addAll(aliasNameLabel, textField);

         aliasValueSearchField = new StringSearchField(textField, searchManager, linkedRootRegistry);
         textField.setText(aliasValue == null ? "" : aliasValue);
         aliasValueSearchField.setupAutoCompletion();
         aliasValueSearchField.supplierProperty().addListener((o, oldValue, newValue) ->
                                                              {
                                                                 if (newValue == null)
                                                                    LogTools.info("Alias {} is not valid.", aliasName);
                                                              });
      }

      public Pane getMainPane()
      {
         return mainPane;
      }
   }
}
