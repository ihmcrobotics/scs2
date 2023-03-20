package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.awt.Dimension;
import java.awt.Toolkit;
import java.io.File;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javax.imageio.ImageIO;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.embed.swing.SwingFXUtils;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.SnapshotParameters;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;

public class SnapshotManager
{
   private final ObjectProperty<Stage> snapshotSelectionWindowProperty = new SimpleObjectProperty<>(this, "snapshotWindowSelection", null);
   private final Map<Object, Recordable> recordables = new LinkedHashMap<>();
   private Stage primaryStage;

   public SnapshotManager(Stage mainWindow, JavaFXMessager messager, SessionVisualizerTopics topics)
   {
      this.primaryStage = mainWindow;
      messager.addTopicListener(topics.getTakeSnapshot(), message -> takeSnapshot());
      messager.addTopicListener(topics.getRegisterRecordable(), this::registerRecordable);
      messager.addTopicListener(topics.getForgetRecordable(), this::forgetRecordable);

      primaryStage.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed() && snapshotSelectionWindowProperty.get() != null)
            snapshotSelectionWindowProperty.get().close();
      });
   }

   public void registerRecordable(Object recordable)
   {
      if (recordable == null)
         return;
      if (recordable instanceof Node)
         recordables.put(recordable, (params) -> ((Node) recordable).snapshot(params, null));
      else if (recordable instanceof Scene)
         recordables.put(recordable, (params) -> ((Scene) recordable).snapshot(null));
      else
         LogTools.warn("Can only record Node or Scen, received: " + recordable.getClass().getSimpleName());
   }

   public void forgetRecordable(Object recordable)
   {
      if (recordable == null)
         return;
      recordables.remove(recordable);
   }

   public void takeSnapshot()
   {
      List<Image> snapshots = recordables.values().stream().map(node -> node.snapshot(null)).collect(Collectors.toList());

      Stage window = snapshotSelectionWindowProperty.get();

      if (window == null)
      {
         window = new Stage();
         window.setTitle("Select snapshot to save...");
         snapshotSelectionWindowProperty.set(window);
      }

      int nCols = Math.min(4, snapshots.size());
      int nRows = snapshots.size() / nCols + 1;

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
      System.out.println(screenSize);
      double width = 0.5 * screenSize.getWidth() / nCols;
      double height = 0.5 * screenSize.getHeight() / nRows;

      List<ImageView> previews = snapshots.stream().map(snapshot -> newSnapshotPreview(snapshot, width, height)).collect(Collectors.toList());

      GridPane pane = new GridPane();
      pane.setPadding(new Insets(20, 20, 20, 20));
      int col = 0, row = 0;

      for (ImageView preview : previews)
      {
         pane.add(preview, col, row);
         col++;
         if (col == nCols)
         {
            col = 0;
            row++;
         }
      }

      window.setScene(new Scene(pane));
      window.centerOnScreen();
      window.show();
      window.toFront();
      window.setOnCloseRequest(e -> snapshotSelectionWindowProperty.set(null));
   }

   private ImageView newSnapshotPreview(Image snapshot, double width, double height)
   {
      ImageView imageView = new ImageView(snapshot);
      imageView.setPreserveRatio(true);
      imageView.setFitHeight(height);
      imageView.setFitWidth(width);
      imageView.setOnMouseEntered(new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            imageView.setScaleX(1.1);
            imageView.setScaleY(1.1);
            imageView.setStyle("-fx-border-color:darkred; -fx-border-width:10;");
            imageView.toFront();
         }
      });
      imageView.setOnMouseExited(new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            imageView.setStyle("");
            imageView.setScaleX(1.0);
            imageView.setScaleY(1.0);
         }
      });
      imageView.setOnMouseClicked(new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() == MouseButton.PRIMARY)
               saveSnapshot(imageView.getImage());
         }
      });

      return imageView;
   }

   private void saveSnapshot(Image snapshot)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setTitle("Save snapshot as...");
      fileChooser.getExtensionFilters().add(new ExtensionFilter("PNG files (*.png)", "*.png"));
      // JPG outputs the image with bad colors.
      // fileChooser.getExtensionFilters().add(new ExtensionFilter("JPG files (*.jpg)", "*.jpg"));
      File fileToSaveTo = fileChooser.showSaveDialog(primaryStage);

      if (fileToSaveTo == null)
         return;

      String fileName = fileToSaveTo.getName();
      String fileExtension = fileName.substring(fileName.lastIndexOf(".") + 1, fileToSaveTo.getName().length());

      try
      {
         ImageIO.write(SwingFXUtils.fromFXImage(snapshot, null), fileExtension, fileToSaveTo);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      if (snapshotSelectionWindowProperty.get() != null)
         snapshotSelectionWindowProperty.get().close();
   }

   private static interface Recordable
   {
      Image snapshot(SnapshotParameters params);
   }
}
