package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator;

import com.jfoenix.controls.JFXListView;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.ListCell;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import javafx.scene.text.FontPosture;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.symbolic.parser.EquationAliasManager;
import us.ihmc.scs2.symbolic.parser.EquationAliasManager.EquationAlias;
import us.ihmc.scs2.symbolic.parser.EquationOperationLibrary;
import us.ihmc.scs2.symbolic.parser.EquationSymbol;

import java.util.Map.Entry;

public class YoEquationEditorHelpPaneController
{
   @FXML
   public VBox mainPane;
   @FXML
   public JFXListView<Pair<String, String>> constantsListView;
   @FXML
   public JFXListView<Pair<String, String>> symbolsListView;
   @FXML
   public JFXListView<Pair<String, String>> functionsListView;

   private final IntegerProperty nameLabelPrefWidthProperty = new SimpleIntegerProperty(this, "nameLabelPrefWidth", -1);

   public void show(Window owner)
   {
      constantsListView.setCellFactory(param -> new NameDescriptionListCell());
      symbolsListView.setCellFactory(param -> new NameDescriptionListCell());
      functionsListView.setCellFactory(param -> new NameDescriptionListCell());

      for (Entry<String, EquationAlias> aliasEntry : EquationAliasManager.defaultAliases.entrySet())
      {
         constantsListView.getItems().add(new Pair<>(aliasEntry.getKey(), aliasEntry.getValue().input().valueAsString()));
      }

      for (EquationSymbol symbol : EquationSymbol.getSupportedSymbols())
      {
         symbolsListView.getItems().add(new Pair<>(symbol.getSymbolString(), symbol.getDescription()));
      }

      for (String name : EquationOperationLibrary.getOperationNames())
      {
         functionsListView.getItems().add(new Pair<>(name, EquationOperationLibrary.getOperationDescription(name)));
      }

      Stage stage = new Stage();
      stage.initStyle(StageStyle.UTILITY);
      stage.initOwner(owner);
      stage.setTitle("Equation Editor Help");
      SessionVisualizerIOTools.addSCSIconToWindow(stage);

      Scene scene = new Scene(mainPane);
      stage.setScene(scene);
      stage.show();
   }

   private class NameDescriptionListCell extends ListCell<Pair<String, String>>
   {
      @Override
      protected void updateItem(Pair<String, String> item, boolean empty)
      {
         super.updateItem(item, empty);
         if (empty || item == null)
         {
            setText(null);
            setGraphic(null);
         }
         else
         {
            setText(null);
            HBox graphic = new HBox(5);
            Label nameLabel = new Label(item.getKey());
            nameLabel.prefWidthProperty().bind(nameLabelPrefWidthProperty);

            Font nameFont = Font.font(Font.getDefault().getFamily(), FontPosture.ITALIC, 12);
            Text sample = new Text(item.getKey());
            sample.setFont(nameFont);
            nameLabelPrefWidthProperty.set((int) Math.max(nameLabelPrefWidthProperty.get(), sample.getLayoutBounds().getWidth() + 10));
            nameLabel.setFont(nameFont);
            graphic.getChildren().add(nameLabel);
            Label descriptionLabel = new Label(item.getValue());
            descriptionLabel.setFont(Font.font(Font.getDefault().getFamily(), 12));
            graphic.getChildren().add(descriptionLabel);
            setGraphic(graphic);
         }
      }
   }
}
