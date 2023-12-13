package us.ihmc.scs2.sessionVisualizer.jfx;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Hyperlink;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.version.SCS2VersionChecker;

public class AboutWindowController implements VisualizerController
{

   @FXML
   private Stage stage;
   @FXML
   private Text nameAndVersionText;
   @FXML
   private Hyperlink repositoryLink;
   @FXML
   private Text lastestVersionText;
   @FXML
   private Hyperlink downloadLink;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      SessionVisualizerIOTools.addSCSIconToWindow(stage);
      stage.initOwner(toolkit.getWindow());

      nameAndVersionText.setText(nameAndVersionText.getText().replace("VERSION", SCS2VersionChecker.getCurrentBaseVersion()));

      repositoryLink.setText(SCS2VersionChecker.REPOSITORY_URL.toString());

      if (SCS2VersionChecker.getLatestBaseVersion() == null)
         lastestVersionText.setText("unknown - check your internet connection");
      else
         lastestVersionText.setText(SCS2VersionChecker.getLatestBaseVersion());

      downloadLink.setText(SCS2VersionChecker.DOWNLOAD_URL.toString());

      stage.show();
      JavaFXMissingTools.centerWindowInOwner(stage, toolkit.getWindow());
   }

   @FXML
   public void openDownloadPageURL()
   {
      SessionVisualizerIOTools.openWebpage(SCS2VersionChecker.DOWNLOAD_URL);
   }

   public void openRepositoryURL(ActionEvent actionEvent)
   {
      SessionVisualizerIOTools.openWebpage(SCS2VersionChecker.REPOSITORY_URL);
   }
}
