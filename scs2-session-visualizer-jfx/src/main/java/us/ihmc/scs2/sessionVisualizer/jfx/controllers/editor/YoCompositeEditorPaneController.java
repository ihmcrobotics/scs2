package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor;

import com.jfoenix.controls.JFXTextField;
import javafx.beans.binding.BooleanExpression;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.ReadOnlyProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.fxml.FXML;
import javafx.geometry.HPos;
import javafx.geometry.VPos;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.GridPane;
import us.ihmc.scs2.definition.yoComposite.YoCompositeDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.DoubleSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.ReferenceFrameSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.YoCompositeSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.CompositeProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositePattern;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class YoCompositeEditorPaneController
{
   private static final double VALID_IMAGE_HEIGHT = 25.0;

   @FXML
   private GridPane mainPane;
   @FXML
   private Label searchYoCompositeLabel;
   @FXML
   private TextField searchYoCompositeTextField;

   private Label[] componentLabels;
   private TextField[] componentSearchTextFields;
   private ImageView[] componentValidImageViews;

   private YoCompositeSearchField yoCompositeTextField;
   private DoubleSearchField[] yoComponentTextFields;

   private int numberOfComponents;
   private YoCompositePattern yoCompositePattern;

   private final StringProperty compositeNameProperty = new SimpleStringProperty(this, "compositeName", "YoComposite");

   private BooleanExpression inputsValidityProperty;
   private ObjectProperty<DoubleProperty[]> compositeSupplierProperty = new SimpleObjectProperty<>(this, "compositeSupplier", null);

   private Label referenceFrameLabel;
   private TextField referenceFrameSearchTextField;
   private ImageView referenceFrameValidImageView;
   private ReferenceFrameManager referenceFrameManager;

   private ReferenceFrameSearchField yoReferenceFrameTextField;

   private final List<Runnable> cleanupTasks = new ArrayList<>();

   public void initialize(SessionVisualizerToolkit toolkit, YoCompositeCollection yoCompositeCollection, boolean setupReferenceFrameFields)
   {
      initialize(toolkit.getYoCompositeSearchManager(),
                 toolkit.getReferenceFrameManager(),
                 toolkit.getYoManager().getLinkedRootRegistry(),
                 yoCompositeCollection,
                 setupReferenceFrameFields);
   }

   public void initialize(YoCompositeSearchManager searchManager,
                          ReferenceFrameManager referenceFrameManager,
                          LinkedYoRegistry linkedRootRegistry,
                          YoCompositeCollection yoCompositeCollection,
                          boolean setupReferenceFrameFields)
   {
      yoCompositePattern = yoCompositeCollection.getPattern();
      this.referenceFrameManager = referenceFrameManager;

      numberOfComponents = yoCompositePattern.getComponentIdentifiers() != null ? yoCompositePattern.getComponentIdentifiers().length : 1;

      createLayout();
      if (setupReferenceFrameFields)
         setupReferenceFrameFields();

      yoComponentTextFields = new DoubleSearchField[numberOfComponents];
      compositeSupplierProperty.set(new DoubleProperty[numberOfComponents]);

      for (int i = 0; i < numberOfComponents; i++)
      {
         DoubleSearchField yoComponentTextField = new DoubleSearchField(componentSearchTextFields[i],
                                                                        searchManager,
                                                                        linkedRootRegistry,
                                                                        componentValidImageViews[i]);
         yoComponentTextField.setupAutoCompletion();
         if (inputsValidityProperty == null)
            inputsValidityProperty = BooleanExpression.booleanExpression(yoComponentTextField.getValidityProperty());
         else
            inputsValidityProperty = inputsValidityProperty.and(yoComponentTextField.getValidityProperty());

         int supplierIndex = i;

         yoComponentTextField.supplierProperty().addListener((o, oldValue, newValue) ->
                                                             {
                                                                DoubleProperty[] newSuppliers = Arrays.copyOf(compositeSupplierProperty.get(),
                                                                                                              numberOfComponents);
                                                                newSuppliers[supplierIndex] = newValue;
                                                                compositeSupplierProperty.set(newSuppliers);
                                                             });

         yoComponentTextFields[i] = yoComponentTextField;
      }

      if (setupReferenceFrameFields)
         inputsValidityProperty = inputsValidityProperty.and(yoReferenceFrameTextField.getValidityProperty());

      if (numberOfComponents > 1)
      {
         yoCompositeTextField = new YoCompositeSearchField(searchManager, yoCompositeCollection, searchYoCompositeTextField);
         yoCompositeTextField.setupAutoCompletion();
         yoCompositeTextField.attachIndividualComponentFields(componentSearchTextFields);
      }

      for (int i = 0; i < numberOfComponents; i++)
      {
         componentSearchTextFields[i].setText(Double.toString(0.0));
      }

      compositeNameProperty.addListener((observable, oldValue, newValue) ->
                                        {
                                           if (newValue == null || newValue.isEmpty())
                                           {
                                              compositeNameProperty.set(oldValue);
                                              return;
                                           }

                                           searchYoCompositeLabel.setText(YoGraphicFXControllerTools.replaceAndMatchCase(searchYoCompositeLabel.getText(),
                                                                                                                         oldValue,
                                                                                                                         newValue));
                                        });
   }

   private void createLayout()
   {
      componentLabels = new Label[numberOfComponents];
      componentSearchTextFields = new JFXTextField[numberOfComponents];
      componentValidImageViews = new ImageView[numberOfComponents];

      if (numberOfComponents == 1)
      { // Reshaping the layout for a singleton YoComposite
         mainPane.getChildren().clear();
         mainPane.getRowConstraints().remove(0);
         Label componentLabel = new Label(compositeNameProperty.get());
         mainPane.getChildren().add(componentLabel);
         GridPane.setConstraints(componentLabel, 0, 0, 1, 1, HPos.LEFT, VPos.CENTER);

         JFXTextField componentSearchTextField = new JFXTextField();
         mainPane.getChildren().add(componentSearchTextField);
         GridPane.setConstraints(componentSearchTextField, 1, 0, 1, 1, HPos.CENTER, VPos.CENTER);

         ImageView componentValidImageView = new ImageView(SessionVisualizerIOTools.INVALID_ICON_IMAGE);
         componentValidImageView.setPreserveRatio(true);
         componentValidImageView.setFitHeight(VALID_IMAGE_HEIGHT);
         mainPane.getChildren().add(componentValidImageView);
         GridPane.setConstraints(componentValidImageView, 2, 0, 1, 1, HPos.LEFT, VPos.CENTER);

         componentLabels[0] = componentLabel;
         componentSearchTextFields[0] = componentSearchTextField;
         componentValidImageViews[0] = componentValidImageView;

         searchYoCompositeLabel = componentLabel;
         searchYoCompositeTextField = componentSearchTextField;
      }
      else
      {
         String[] componentIdentifiers = yoCompositePattern.getComponentIdentifiers();

         while (mainPane.getRowConstraints().size() < numberOfComponents + 1)
         {
            mainPane.getRowConstraints().add(mainPane.getRowConstraints().get(1));
         }

         for (int i = 0; i < numberOfComponents; i++)
         {
            String id = componentIdentifiers[i];
            Label componentLabel = new Label(id);
            mainPane.getChildren().add(componentLabel);
            GridPane.setConstraints(componentLabel, 0, i + 1, 1, 1, HPos.RIGHT, VPos.CENTER);

            JFXTextField componentSearchTextField = new JFXTextField();
            mainPane.getChildren().add(componentSearchTextField);
            GridPane.setConstraints(componentSearchTextField, 1, i + 1, 1, 1, HPos.CENTER, VPos.CENTER);

            ImageView componentValidImageView = new ImageView(SessionVisualizerIOTools.INVALID_ICON_IMAGE);
            componentValidImageView.setPreserveRatio(true);
            componentValidImageView.setFitHeight(25.0);
            mainPane.getChildren().add(componentValidImageView);
            GridPane.setConstraints(componentValidImageView, 2, i + 1, 1, 1, HPos.LEFT, VPos.CENTER);

            componentLabels[i] = componentLabel;
            componentSearchTextFields[i] = componentSearchTextField;
            componentValidImageViews[i] = componentValidImageView;
         }
      }
   }

   private void setupReferenceFrameFields()
   {
      referenceFrameLabel = new Label("Reference frame");
      referenceFrameSearchTextField = new JFXTextField();
      referenceFrameValidImageView = new ImageView(SessionVisualizerIOTools.VALID_ICON_IMAGE);
      referenceFrameValidImageView.setPreserveRatio(true);
      referenceFrameValidImageView.setFitHeight(VALID_IMAGE_HEIGHT);

      mainPane.getRowConstraints().add(mainPane.getRowConstraints().get(1));
      mainPane.getChildren().addAll(referenceFrameLabel, referenceFrameSearchTextField, referenceFrameValidImageView);
      GridPane.setConstraints(referenceFrameLabel, 0, numberOfComponents + 1, 1, 1, HPos.LEFT, VPos.CENTER);
      GridPane.setConstraints(referenceFrameSearchTextField, 1, numberOfComponents + 1, 1, 1, HPos.CENTER, VPos.CENTER);
      GridPane.setConstraints(referenceFrameValidImageView, 2, numberOfComponents + 1, 1, 1, HPos.LEFT, VPos.CENTER);

      yoReferenceFrameTextField = new ReferenceFrameSearchField(referenceFrameSearchTextField, referenceFrameManager, referenceFrameValidImageView);
      yoReferenceFrameTextField.setupAutoCompletion();
      referenceFrameSearchTextField.setText(referenceFrameManager.getWorldFrame().getName());
   }

   public void setCompositeName(String compositeName)
   {
      compositeNameProperty.set(compositeName);
   }

   public void clearInput()
   {
      for (int i = 0; i < numberOfComponents; i++)
      {
         componentSearchTextFields[i].setText("0.0");
      }
      if (yoCompositeTextField != null)
         yoCompositeTextField.initializeFieldFromComponents();
      if (referenceFrameSearchTextField != null)
      {
         referenceFrameSearchTextField.setText(referenceFrameManager.getWorldFrame().getName());
      }
   }

   public void setInput(YoComposite input)
   {
      if (yoCompositeTextField != null)
         yoCompositeTextField.setInput(input);
      else
         componentSearchTextFields[0].setText(input.getUniqueShortName());
   }

   public void setInput(CompositeProperty input)
   {
      setInput(CompositePropertyTools.toYoCompositeDefinition(input));
   }

   public void setInput(YoCompositeDefinition input)
   {
      for (int i = 0; i < numberOfComponents; i++)
      {
         componentSearchTextFields[i].setText(input.getComponentValues()[i]);
      }
      if (yoCompositeTextField != null)
         yoCompositeTextField.initializeFieldFromComponents();
      if (referenceFrameSearchTextField != null)
      {
         if (input.getReferenceFrame() == null)
            referenceFrameSearchTextField.setText(referenceFrameManager.getWorldFrame().getName());
         else
            referenceFrameSearchTextField.setText(input.getReferenceFrame());
      }
   }

   public void setInput(String... input)
   {
      for (int i = 0; i < numberOfComponents; i++)
      {
         componentSearchTextFields[i].setText(input[i]);
      }
      if (yoCompositeTextField != null)
         yoCompositeTextField.initializeFieldFromComponents();
   }

   public void setReferenceFrame(ReferenceFrameWrapper referenceFrame)
   {
      if (yoReferenceFrameTextField == null)
         return;
      referenceFrameSearchTextField.setText(referenceFrame.getFullName());
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public ReadOnlyObjectProperty<DoubleProperty[]> compositeSupplierProperty()
   {
      return compositeSupplierProperty;
   }

   public ReadOnlyProperty<Property<ReferenceFrameWrapper>> frameSupplierProperty()
   {
      return yoReferenceFrameTextField.supplierProperty();
   }

   public void addInputNotification(Runnable callback)
   {
      compositeSupplierProperty.addListener((o, oldValue, newValue) -> callback.run());
      if (yoReferenceFrameTextField != null)
         frameSupplierProperty().addListener((o, oldValue, newValue) -> callback.run());
   }

   public void addInputListener(Consumer<DoubleProperty[]> componentsConsumer)
   {
      compositeSupplierProperty.addListener((o, oldValue, newValue) -> componentsConsumer.accept(newValue));
   }

   public void addInputListener(BiConsumer<DoubleProperty[], Property<ReferenceFrameWrapper>> frameComponentsConsumer)
   {
      compositeSupplierProperty.addListener((o, oldValue, newValue) -> frameComponentsConsumer.accept(newValue, yoReferenceFrameTextField.getSupplier()));
      frameSupplierProperty().addListener((o, oldValue, newValue) -> frameComponentsConsumer.accept(compositeSupplierProperty.get(), newValue));
   }

   public void bindYoCompositeDoubleProperty(CompositeProperty propertyToBind)
   {
      if (yoReferenceFrameTextField == null)
         addInputListener(components -> propertyToBind.setComponentValueProperties(components));
      else
         addInputListener((components, frame) -> propertyToBind.set(frame, components));
   }

   public GridPane getMainPane()
   {
      return mainPane;
   }

   public TextField getSearchYoCompositeTextField()
   {
      return searchYoCompositeTextField;
   }
}
