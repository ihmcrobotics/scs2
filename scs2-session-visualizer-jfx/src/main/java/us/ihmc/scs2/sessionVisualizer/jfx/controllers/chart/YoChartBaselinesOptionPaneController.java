package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import java.io.IOException;

import javafx.beans.property.SimpleDoubleProperty;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.ListView;
import javafx.scene.control.TitledPane;
import javafx.scene.paint.Color;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartMarker;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartMarkerType;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.ControllerListCell;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoChartBaselinesOptionPaneController
{
   private static final double ROW_HEIGHT = 24 + 12;

   @FXML
   private TitledPane mainPane;
   @FXML
   private ListView<YoChartBaselineEditorPaneController> baselinesListView;

   private ObservableList<YoChartBaselineEditorPaneController> listViewItems;
   private ObservableList<ChartMarker> userMarkers;
   private boolean ignoreChanges = false;

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      baselinesListView.setCellFactory(param -> new ControllerListCell<>());
      baselinesListView.setPrefHeight(ROW_HEIGHT + 2);

      listViewItems = baselinesListView.getItems();
      listViewItems.addListener((ListChangeListener<YoChartBaselineEditorPaneController>) change ->
      {
         int numberOfRows = change.getList().size();
         double prefHeight = numberOfRows * ROW_HEIGHT;
         prefHeight = EuclidCoreTools.clamp(prefHeight, ROW_HEIGHT, 10.0 * ROW_HEIGHT);
         baselinesListView.setPrefHeight(prefHeight + 2);

         if (ignoreChanges)
            return;

         while (change.next())
         {
            if (change.wasAdded())
            {
               for (YoChartBaselineEditorPaneController newController : change.getAddedSubList())
               {
                  userMarkers.add(newController.getMarker());
               }
            }

            if (change.wasRemoved())
            {
               for (YoChartBaselineEditorPaneController oldController : change.getRemoved())
               {
                  userMarkers.remove(oldController.getMarker());
                  oldController.dispose();
               }
            }
         }
      });
   }

   public void setInput(ObservableList<ChartMarker> userMarkers)
   {
      this.userMarkers = userMarkers;

      ignoreChanges = true;

      listViewItems.clear();

      for (ChartMarker marker : userMarkers)
      {
         listViewItems.add(newBaselineEditor(marker));
      }

      ignoreChanges = false;
   }

   @FXML
   private void addBaseline()
   {
      listViewItems.add(newBaselineEditor(null));
   }

   private YoChartBaselineEditorPaneController newBaselineEditor(ChartMarker marker)
   {
      FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CHART_BASELINE_EDITOR_PANE_URL);
      try
      {
         loader.load();
         YoChartBaselineEditorPaneController editor = loader.getController();
         if (marker == null)
         {
            marker = new ChartMarker(ChartMarkerType.HORIZONTAL, new SimpleDoubleProperty(this, "userMarker" + userMarkers.size(), 0.0));
            marker.setStroke(Color.hsb(userMarkers.size() * 360.0 / 4.3, 0.7, 0.8));
         }
         editor.setInput(marker, () -> listViewItems.remove(editor));
         return editor;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public TitledPane getMainPane()
   {
      return mainPane;
   }
}
