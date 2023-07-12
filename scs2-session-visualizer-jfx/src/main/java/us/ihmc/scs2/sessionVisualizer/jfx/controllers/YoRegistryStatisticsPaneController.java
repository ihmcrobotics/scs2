package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import java.util.function.Function;

import com.jfoenix.controls.JFXTreeTableColumn;
import com.jfoenix.controls.RecursiveTreeItem;
import com.jfoenix.controls.datamodels.treetable.RecursiveTreeObject;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeTableColumn;
import javafx.scene.control.TreeTableView;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoRegistryStatisticsPaneController
{
   @FXML
   private AnchorPane mainAnchorPane;
   @FXML
   private TreeTableView<YoRegistryInfo> treeTableView;

   private TreeItem<YoRegistryInfo> rootTreeItem;
   private Stage window;

   @SuppressWarnings("unchecked")
   public void initialize(SessionVisualizerToolkit toolkit)
   {
      TreeTableColumn<YoRegistryInfo, String> nameCol;
      TreeTableColumn<YoRegistryInfo, Integer> nVarsShallowCol;
      TreeTableColumn<YoRegistryInfo, Integer> nVarsDeepCol;
      TreeTableColumn<YoRegistryInfo, Integer> nChildShallowCol;

      nameCol = createColumn("Registry", 300, 200, 800, YoRegistryInfo::getName);
      nVarsShallowCol = createColumn("Number of\nvariables\n(shallow)", 100, YoRegistryInfo::getNumberOfVariablesShallow);
      nVarsDeepCol = createColumn("Number of\nvariables\n(deep)", 100, YoRegistryInfo::getNumberOfVariablesDeep);
      nChildShallowCol = createColumn("Number of\nchildren\n(shallow)", 100, YoRegistryInfo::getNumberOfChildrenShallow);

      // TODO Maybe move to CSS?
      nameCol.setStyle("-fx-alignment:center-left");
      nVarsShallowCol.setStyle("-fx-alignment:center-right");
      nVarsDeepCol.setStyle("-fx-alignment:center-right");
      nChildShallowCol.setStyle("-fx-alignment:center-right");

      rootTreeItem = new RecursiveTreeItem<>(FXCollections.observableArrayList(), RecursiveTreeObject<YoRegistryInfo>::getChildren);
      treeTableView.setRoot(rootTreeItem);
      treeTableView.setShowRoot(false);
      treeTableView.getColumns().setAll(nameCol, nVarsShallowCol, nVarsDeepCol, nChildShallowCol);

      window = new Stage();
      window.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ESCAPE)
            close();
      });

      toolkit.getMainWindow().addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            close();
      });
      window.setTitle("YoRegistry statistics");
      SessionVisualizerIOTools.addSCSIconToWindow(window);
      window.setScene(new Scene(mainAnchorPane));
      window.initOwner(toolkit.getMainWindow());
   }

   public void setInput(YoRegistry registry)
   {
      rootTreeItem.getChildren().clear();
      addRegistryRecurvise(rootTreeItem, registry);
      window.setTitle(registry.getName() + " YoRegistry statistics");
   }

   private static void addRegistryRecurvise(TreeItem<YoRegistryInfo> parentTreeItem, YoRegistry registryToAdd)
   {
      TreeItem<YoRegistryInfo> treeItem = new TreeItem<>(new YoRegistryInfo(registryToAdd));

      for (YoRegistry childRegistry : registryToAdd.getChildren())
      {
         addRegistryRecurvise(treeItem, childRegistry);
      }

      parentTreeItem.getChildren().add(treeItem);
   }

   private <T> TreeTableColumn<YoRegistryInfo, T> createColumn(String name, double prefWidth, Function<YoRegistryInfo, Property<T>> fieldProvider)
   {
      return createColumn(name, prefWidth, prefWidth, prefWidth, fieldProvider);
   }

   private <T> TreeTableColumn<YoRegistryInfo, T> createColumn(String name,
                                                                  double prefWidth,
                                                                  double minWidth,
                                                                  double maxWidth,
                                                                  Function<YoRegistryInfo, Property<T>> fieldProvider)
   {
      TreeTableColumn<YoRegistryInfo, T> column = new JFXTreeTableColumn<>(name);
      column.setPrefWidth(prefWidth);
      column.setMinWidth(minWidth);
      column.setMaxWidth(maxWidth);
      column.setCellValueFactory(param ->
      {
         return fieldProvider.apply(param.getValue().getValue());
      });
      return column;
   }

   private static class YoRegistryInfo extends RecursiveTreeObject<YoRegistryInfo>
   {
      private final StringProperty name = new SimpleStringProperty(this, "registryName", null);
      private final Property<Integer> numberOfVariablesShallow = new SimpleObjectProperty<>(this, "numberOfVariablesShallow", -1);
      private final Property<Integer> numberOfVariablesDeep = new SimpleObjectProperty<>(this, "numberOfVariablesDeep", -1);
      private final Property<Integer> numberOfChildrenShallow = new SimpleObjectProperty<>(this, "numberOfChildrenShallow", -1);

      public YoRegistryInfo(YoRegistry registry)
      {
         name.set(registry.getName());
         numberOfVariablesShallow.setValue(registry.getNumberOfVariables());
         numberOfVariablesDeep.setValue(registry.getNumberOfVariablesDeep());
         numberOfChildrenShallow.setValue(registry.getChildren().size());
      }

      public StringProperty getName()
      {
         return name;
      }

      public Property<Integer> getNumberOfVariablesShallow()
      {
         return numberOfVariablesShallow;
      }

      public Property<Integer> getNumberOfVariablesDeep()
      {
         return numberOfVariablesDeep;
      }

      public Property<Integer> getNumberOfChildrenShallow()
      {
         return numberOfChildrenShallow;
      }
   }

   public void close()
   {
      window.close();
   }

   public void showWindow()
   {
      window.setOpacity(0.0);
      window.toFront();
      window.show();
      Timeline timeline = new Timeline();
      KeyFrame key = new KeyFrame(Duration.seconds(0.125), new KeyValue(window.opacityProperty(), 1.0));
      timeline.getKeyFrames().add(key);
      timeline.play();
   }

   public Stage getWindow()
   {
      return window;
   }
}
