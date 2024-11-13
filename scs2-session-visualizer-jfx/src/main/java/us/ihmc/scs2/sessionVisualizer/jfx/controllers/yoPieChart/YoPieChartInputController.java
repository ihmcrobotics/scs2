package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoPieChart;

import javafx.beans.binding.Bindings;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.scene.Node;
import javafx.scene.chart.PieChart;
import javafx.scene.chart.PieChart.Data;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.Labeled;
import javafx.scene.control.MenuItem;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.Region;
import javafx.scene.text.Text;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableDatabase;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Predicate;

public class YoPieChartInputController
{
   private static final String HIGHLIGHTED_BORDER = "-fx-border-color:green; -fx-border-radius:10;";
   private static final String HIGHLIGHTED_BACKGROUND = "-fx-background-color: #d3d3d3AA;";
   private static final String DEFAULT_BORDER = null;
   private static final String DEFAULT_BACKGROUND = null;

   private final StringProperty backgroundStyle = new SimpleStringProperty(this, "backgroundStyle", DEFAULT_BACKGROUND);
   private final StringProperty borderStyle = new SimpleStringProperty(this, "borderStyle", DEFAULT_BORDER);

   private YoManager yoManager;
   private Region rootPane;
   private Labeled yoVariableDropLabel;
   private YoCompositeSearchManager yoCompositeSearchManager;
   private Predicate<YoVariable> filter;
   private YoVariable yoVariable;

   private final SimpleObjectProperty<ContextMenu> contextMenuProperty = new SimpleObjectProperty<>(this, "buttonContextMenu", null);
   private JavaFXMessager messager;
   private Topic<List<String>> yoCompositeSelectedTopic;
   private AtomicReference<List<String>> yoCompositeSelected;
   private final String defaultText = "Drop YoVariable here";
   private PieChart pieChart;

   private YoVariableDatabase rootRegistryDatabase = null;

   public void initialize(SessionVisualizerToolkit toolkit, Region rootPane, Labeled yoVariableDropLabel, Predicate<YoVariable> filter, PieChart pieChart)
   {
      this.rootPane = rootPane;
      this.yoVariableDropLabel = yoVariableDropLabel;
      this.filter = filter;
      this.pieChart = pieChart;
      this.yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();
      rootRegistryDatabase = toolkit.getYoManager().getRootRegistryDatabase();

      messager = toolkit.getMessager();
      yoCompositeSelectedTopic = toolkit.getTopics().getYoCompositeSelected();
      yoCompositeSelected = messager.createInput(yoCompositeSelectedTopic);

      // These property's get bound so it will update correctly
      rootPane.styleProperty().bind(Bindings.concat(backgroundStyle, "; ", borderStyle));

      rootPane.setOnDragDetected(this::handleDragDetected);
      rootPane.setOnDragOver(this::handleDragOver);
      rootPane.setOnDragDropped(this::handleDragDropped);
      rootPane.setOnDragEntered(this::handleDragEntered);
      rootPane.setOnDragExited(this::handleDragExited);
      rootPane.setOnMousePressed(this::handleMousePressed);
      rootPane.setOnMouseReleased(this::handleMouseReleased);
   }

   public void clear()
   {
      backgroundStyle.set(DEFAULT_BACKGROUND);
      yoVariableDropLabel.setText(defaultText);
   }

   private void handleDragDetected(MouseEvent event)
   {
      if (event == null || yoVariable == null)
         return;

      if (!event.isPrimaryButtonDown())
         return;

      PickResult pickResult = event.getPickResult();

      if (pickResult == null)
         return;

      Node intersectedNode = pickResult.getIntersectedNode();

      if (intersectedNode == null)
         return;

      if (intersectedNode instanceof Text)
      {
         Text legend = (Text) intersectedNode;
         String yoVariableName = legend.getText().split("\\s+")[0];
         if (!yoVariableName.equals(yoVariableDropLabel.toString()))
            return;
         Dragboard dragBoard = legend.startDragAndDrop(TransferMode.COPY);
         ClipboardContent clipboardContent = new ClipboardContent();
         clipboardContent.put(DragAndDropTools.YO_COMPOSITE_REFERENCE, Arrays.asList(YoCompositeTools.YO_VARIABLE, yoVariable.getFullNameString()));
         dragBoard.setContent(clipboardContent);
      }

      event.consume();
   }

   private void handleDragEntered(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
         setSelectionHighlight(true);
      event.consume();
   }

   private void handleDragExited(DragEvent event)
   {
      if (acceptDragEventForDrop(event))
         setSelectionHighlight(false);
      event.consume();
   }

   private void handleDragOver(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
         event.acceptTransferModes(TransferMode.ANY);
      event.consume();
   }

   private void handleDragDropped(DragEvent event)
   {
      // TODO
      //      if (event.isAccepted())
      //         return;

      Dragboard db = event.getDragboard();
      boolean success = false;
      List<YoComposite> yoComposites = DragAndDropTools.retrieveYoCompositesFromDragBoard(db, yoCompositeSearchManager);
      if (yoComposites != null)
      {
         // TODO
         //         for (YoComposite yoComposite : yoComposites)
         setYoVariableInput(yoComposites.get(0).getYoComponents().get(0));
         success = true;
      }
      event.setDropCompleted(success);
      event.consume();

      if (success)
         setSelectionHighlight(false);
   }

   private void handleMousePressed(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY)
      {
         hideContextMenu();
      }
   }

   private void handleMouseReleased(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY)
      {
         if (yoVariableDropLabel != null && event.isStillSincePress())
         {
            messager.submitMessage(yoCompositeSelectedTopic, Arrays.asList(YoCompositeTools.YO_VARIABLE, yoVariableDropLabel.getText()));
         }
      }
      else if (event.getButton() == MouseButton.SECONDARY)
      {
         if (yoVariableDropLabel != null && event.isStillSincePress())
         {
            ContextMenu contextMenu = newGraphContextMenu();
            if (!contextMenu.getItems().isEmpty())
            {
               contextMenuProperty.set(contextMenu);
               contextMenu.show(rootPane, event.getScreenX(), event.getScreenY());
            }
            event.consume();
         }
      }
      else if (event.getButton() == MouseButton.MIDDLE)
      {
         if (yoCompositeSelected.get() != null)
         {
            String type = yoCompositeSelected.get().get(0);
            if (type.equals(YoCompositeTools.YO_VARIABLE))
            {
               String fullname = yoCompositeSelected.get().get(1);
               YoComposite yoComposite = yoCompositeSearchManager.getYoComposite(type, fullname);

               if (yoComposite != null && filter.test(yoComposite.getYoComponents().get(0)))
               {
                  setYoVariableInput(yoComposite.getYoComponents().get(0));
                  messager.submitMessage(yoCompositeSelectedTopic, null);
               }
            }
         }
      }
   }

   private void hideContextMenu()
   {
      if (contextMenuProperty.get() != null)
         contextMenuProperty.set(null);
   }

   private ContextMenu newGraphContextMenu()
   {
      if (yoVariableDropLabel == null)
         return null;

      ContextMenu contextMenu = new ContextMenu();
      MenuItem menuItem = new MenuItem("Remove " + yoVariableDropLabel.getText());
      menuItem.setMnemonicParsing(false);
      menuItem.setOnAction(e ->
                           {
                              for (PieChart.Data data : pieChart.getData())
                              {
                                 if (data.getName().equals(yoVariableDropLabel.getText()))
                                 {
                                    pieChart.getData().remove(data);
                                    break;
                                 }
                              }
                              setYoVariableInput(null);
                           });
      contextMenu.getItems().add(menuItem);
      return contextMenu;
   }

   public void setYoVariableInput(YoVariable yoVariable)
   {
      this.yoVariable = yoVariable;
      backgroundStyle.set(yoVariable == null ? DEFAULT_BACKGROUND : HIGHLIGHTED_BACKGROUND);
      if (yoVariable == null)
      {
         yoVariableDropLabel.setText(defaultText);
      }
      else
      {
         // Check if a variable exists in the pie chart already, and if it does don't add it again
         boolean alreadyExists = pieChart.getData().stream().anyMatch(data -> data.getName().equals(yoVariable.getName()));

         if (!alreadyExists)
         {
            pieChart.getData().add(new Data(yoVariable.getName(), yoVariable.getValueAsDouble()));
         }

         yoVariableDropLabel.setText(yoVariable.getName());
      }
   }

   private boolean acceptDragEventForDrop(DragEvent event)
   {
      if (event.getGestureSource() == yoVariableDropLabel)
         return false;

      Dragboard dragboard = event.getDragboard();
      List<YoComposite> result = DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, yoCompositeSearchManager);
      if (result == null || result.isEmpty())
         return false;
      if (result.get(0).getYoComponents().isEmpty())
         return false;
      return filter.test(result.get(0).getYoComponents().get(0));
   }

   private void setSelectionHighlight(boolean isSelected)
   {
      borderStyle.set(isSelected ? HIGHLIGHTED_BORDER : DEFAULT_BORDER);
   }

   public YoVariable getYoVariable()
   {
      return yoVariable;
   }

   public void setYoVariable(String yoVariableName)
   {
      setYoVariableInput(rootRegistryDatabase.searchExact(yoVariableName));
   }
}
