package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class MultiViewport3DManager
{
   private final GridPane container;
   private final Group mainView3DRoot;

   private MainViewport3DManager mainViewport;
   private final ObservableList<SingleViewport3DManager> allViewports = FXCollections.observableArrayList();
   private final IntegerProperty numberOfColumns = new SimpleIntegerProperty(this, "numberOfColumns", 2);

   public MultiViewport3DManager(Group mainView3DRoot)
   {
      this.mainView3DRoot = mainView3DRoot;
      container = new GridPane();

      allViewports.addListener((ListChangeListener<SingleViewport3DManager>) change -> refreshLayout());
      numberOfColumns.addListener((o, oldValue, newValue) -> refreshLayout());
   }

   public void refreshLayout()
   {
      container.getChildren().clear();

      int numCols = numberOfColumns.get();

      int col = 0;
      int row = 0;

      for (int i = 0; i < allViewports.size(); i++)
      {
         Pane pane = allViewports.get(i).getPane();
         container.add(pane, col, row);
         GridPane.setHgrow(pane, Priority.SOMETIMES);
         GridPane.setVgrow(pane, Priority.SOMETIMES);
         col++;
         if (col >= numCols)
         {
            col = 0;
            row++;
         }
      }
   }

   public void createMainViewport()
   {
      if (mainViewport != null)
         throw new IllegalOperationException("Can only have 1 main viewport");

      mainViewport = new MainViewport3DManager(mainView3DRoot);
      allViewports.add(mainViewport);
   }

   public void addSecondaryViewport()
   {
      allViewports.add(new SecondaryViewport3DManager(mainView3DRoot));
   }

   public MainViewport3DManager getMainViewport()
   {
      return mainViewport;
   }

   public Pane getPane()
   {
      return container;
   }

   public void dispose()
   {
      allViewports.forEach(viewport -> viewport.dispose());
      allViewports.clear();
   }
}
