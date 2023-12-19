package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import us.ihmc.scs2.sessionVisualizer.jfx.AboutWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

import java.io.IOException;

public class HelpMenuController implements VisualizerController
{
   private SessionVisualizerWindowToolkit toolkit;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
   }

   @FXML
   public void openAboutDialog()
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.ABOUT_WINDOW_URL);
         loader.load();
         AboutWindowController controller = loader.getController();
         controller.initialize(toolkit);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
