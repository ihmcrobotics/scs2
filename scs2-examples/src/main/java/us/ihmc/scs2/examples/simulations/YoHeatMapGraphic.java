package us.ihmc.scs2.examples.simulations;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Scene;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.yoVariables.math.YoMatrix;

public class YoHeatMapGraphic
{
   public static void main(String[] args)
   {
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.doNothingPhysicsEngine());
      YoMatrix matrix = new YoMatrix("matrix", 3, 3, scs.getRootRegistry());
      for (int row = 0; row < matrix.getNumRows(); row++)
      {
         for (int column = 0; column < matrix.getNumCols(); column++)
         {
            matrix.set(row, column, Math.random());
         }
      }

      scs.start(true, false, false);

      Platform.runLater(() ->
                        {
                           Stage window = new Stage();
                           window.setTitle("YoHeatMapGraphic");
                           HeatMapPane heatMapPane = new HeatMapPane();
                           heatMapPane.matrixProperty.setValue(matrix);
                           heatMapPane.start();
                           window.setScene(new Scene(heatMapPane, 600, 400));

                           window.setOnCloseRequest(e -> heatMapPane.stop());
                           scs.addVisualizerShutdownListener(window::close);
                           window.show();
                        });
   }

   private static class HeatMapPane extends GridPane
   {

      private StackPane[][] cells;
      private final Property<YoMatrix> matrixProperty = new SimpleObjectProperty<>(this, "matrix", null);
      private final AnimationTimer animationTimer;

      public HeatMapPane()
      {
         setVgap(2.0);
         setHgap(2.0);
         setMaxSize(Double.MAX_VALUE, Double.MAX_VALUE);

         setStyle("-fx-background-color: grey;");

         animationTimer = new AnimationTimer()
         {
            @Override
            public void handle(long now)
            {
               YoMatrix matrix = matrixProperty.getValue();
               if (matrix == null)
               {
                  getChildren().clear();
                  cells = null;
                  return;
               }

               double max = Double.NEGATIVE_INFINITY;
               double min = Double.POSITIVE_INFINITY;

               for (int row = 0; row < matrix.getNumRows(); row++)
               {
                  for (int column = 0; column < matrix.getNumCols(); column++)
                  {
                     double value = matrix.get(row, column);
                     max = Math.max(max, value);
                     min = Math.min(min, value);
                  }
               }

               if (cells == null || cells.length != matrix.getNumRows() || cells[0].length != matrix.getNumCols())
               {
                  getChildren().clear();
                  cells = new StackPane[matrix.getNumRows()][matrix.getNumCols()];

                  for (int row = 0; row < matrix.getNumRows(); row++)
                  {
                     for (int column = 0; column < matrix.getNumCols(); column++)
                     {
                        StackPane cell = new StackPane();
                        cell.setMinSize(1.0, 1.0);
                        cell.setPrefSize(10.0, 10.0);
                        cell.setMaxSize(Double.MAX_VALUE, Double.MAX_VALUE);
                        cells[row][column] = cell;
                        add(cell, column, row);
                        GridPane.setHgrow(cell, Priority.ALWAYS);
                        GridPane.setVgrow(cell, Priority.ALWAYS);
                     }
                  }
               }

               for (int row = 0; row < matrix.getNumRows(); row++)
               {
                  for (int column = 0; column < matrix.getNumCols(); column++)
                  {
                     double value = matrix.get(row, column);
                     double normalizedValue = (value - min) / (max - min);
                     // Render the value at (row, column) with normalizedValue
                     double hue = 240.0 * normalizedValue;
                     cells[row][column].setStyle("-fx-background-color: hsb(" + hue + ", 100%, 100%);");
                  }
               }
            }
         };
      }

      public void start()
      {
         animationTimer.start();
      }

      public void stop()
      {
         animationTimer.stop();
      }
   }
}