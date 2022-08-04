package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;
import java.util.prefs.Preferences;

import javafx.fxml.FXMLLoader;
import javafx.scene.control.Dialog;
import javafx.scene.image.Image;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.Window;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.scs2.session.SessionIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;

public class SessionVisualizerIOTools
{
   private static final ClassLoader classLoader = SessionVisualizerIOTools.class.getClassLoader();

   public static final Path SCS2_HOME = SessionIOTools.SCS2_HOME;
   public static final Path SCS2_CONFIGURATION_DEFAULT_PATH = SCS2_HOME.resolve("Configurations");
   public static final String SCS2_CONFIGURATION_FOLDER_KEY = "scsConfigFolderPath";

   static
   {
      try
      {
         FileTools.ensureDirectoryExists(SCS2_CONFIGURATION_DEFAULT_PATH);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static final String scsConfigurationFileExtension = ".scs2";
   public static final String scsMainConfigurationFileExtension = ".scs2.main";
   public static final String yoChartGroupConfigurationFileExtension = ".scs2.chart";
   public static final String yoGraphicConfigurationFileExtension = SessionIOTools.yoGraphicConfigurationFileExtension;
   public static final String yoCompositeConfigurationFileExtension = ".scs2.yoComposite";
   public static final String yoEntryConfigurationFileExtension = ".scs2.yoEntry";
   public static final String yoSliderboardConfigurationFileExtension = ".scs2.yoSliderboard";
   public static final String videoFileExtension = ".mp4";
   public static final ExtensionFilter scs2InfoFilter = new ExtensionFilter("SCS2 Info File", "*" + SessionIOTools.infoFileExtension);
   public static final ExtensionFilter scs2ConfigurationFilter = new ExtensionFilter("SCS2 Config File", "*" + scsConfigurationFileExtension);
   public static final ExtensionFilter yoChartGroupConfigurationFilter = new ExtensionFilter("SCS2 YoChartGroup File",
                                                                                             "*" + yoChartGroupConfigurationFileExtension);
   public static final ExtensionFilter yoGraphicConfigurationFilter = new ExtensionFilter("SCS2 YoGraphic File", "*" + yoGraphicConfigurationFileExtension);
   public static final ExtensionFilter yoCompositeConfigurationFilter = new ExtensionFilter("SCS2 YoComposite File",
                                                                                            "*" + yoCompositeConfigurationFileExtension);
   public static final ExtensionFilter yoEntryConfigurationFilter = new ExtensionFilter("SCS2 YoEntry File", "*" + yoEntryConfigurationFileExtension);
   public static final ExtensionFilter yoSliderboardConfigurationFilter = new ExtensionFilter("SCS2 YoSliderboard File",
                                                                                              "*" + yoSliderboardConfigurationFileExtension);

   public static final ExtensionFilter videoExtensionFilter = new ExtensionFilter("MP4", "*" + videoFileExtension);

   private static final String CSS_FOLDER = "css/";
   private static final String FXML_FOLDER = "fxml/";
   private static final String ICONS_FOLDER = "icons/";
   private static final String IMAGES_FOLDER = "images/";
   private static final String SKYBOX_FOLDER = "skybox/";
   private static final String CONFIGURATION_FOLDER = "configuration/";
   private static final String YO_GRAPHIC_FOLDER = "yoGraphic/";
   private static final String SESSION_FOLDER = "session/";

   // CSS
   public static final URL GENERAL_STYLESHEET = getCSSResource("GeneralStylesheet");

   // Icon list:
   public static final Image SCS_ICON_IMAGE = loadIcon("scs-icon.png");
   public static final Image INVALID_ICON_IMAGE = loadIcon("invalid-icon.png");
   public static final Image VALID_ICON_IMAGE = loadIcon("valid-icon.png");
   public static final Image REMOTE_SESSION_IMAGE = loadIcon("viz-icon.png");
   public static final Image LOG_SESSION_IMAGE = loadIcon("log-icon.png");

   // FXML list:
   private static final String CHART = "chart/";
   private static final String YO_COMPOSITE = "yoComposite/";
   private static final String YO_COMPOSITE_PATTERN = YO_COMPOSITE + "pattern/";
   private static final String YO_COMPOSITE_SEARCH = YO_COMPOSITE + "search/";
   private static final String YO_COMPOSITE_ENTRY = YO_COMPOSITE + "entry/";
   private static final String YO_GRAPHIC = "yoGraphic/";
   private static final String YO_GRAPHIC_2D = YO_GRAPHIC + "graphic2D/";
   private static final String YO_GRAPHIC_3D = YO_GRAPHIC + "graphic3D/";
   private static final String YO_GRAPHIC_GROUP = YO_GRAPHIC + "group/";
   private static final String YO_GRAPHIC_SUB_EDITOR = YO_GRAPHIC + "editor/";
   private static final String YO_SLIDERBOARD = "yoSliderboard/";
   private static final String YO_SLIDERBOARD_BCF2000 = YO_SLIDERBOARD + "bcf2000/";

   // YoComposite resources:
   public static final String DEFAULT_YO_COMPOSITE_PATTERNS_FILE = "DefaultYoCompositePatterns" + yoCompositeConfigurationFileExtension;
   public static final URL YO_COMPOSITE_PATTERN_EDITOR_PANE_URL = getFXMLResource(YO_COMPOSITE_PATTERN, "YoCompositePatternEditorPane");
   public static final URL YO_COMPOSITE_PATTERN_PROPERTY_WINDOW_URL = getFXMLResource(YO_COMPOSITE_PATTERN, "YoCompositePatternPropertyWindow");
   public static final URL YO_COMPOSITE_SEARCH_PANEL_URL = getFXMLResource(YO_COMPOSITE_SEARCH, "YoCompositeSearchPane");
   public static final URL YO_SEARCH_OPTIONS_PANE_URL = getFXMLResource(YO_COMPOSITE_SEARCH, "YoSearchOptionsPane");
   public static final URL YO_SEARCH_TAB_PANE_URL = getFXMLResource(YO_COMPOSITE_SEARCH, "YoSearchTabPane");
   public static final URL YO_ENTRY_LIST_VIEW_URL = getFXMLResource(YO_COMPOSITE_ENTRY, "YoEntryListView");
   public static final URL YO_REGISTRY_STATISTICS_URL = getFXMLResource("YoRegistryStatisticsPane");
   // YoGraphic resources:
   public static final URL GRAPHIC_2D_CROSS_URL = getYoGraphicResource("cross.svg");
   public static final URL GRAPHIC_2D_PLUS_URL = getYoGraphicResource("plus.svg");
   public static final URL GRAPHIC_2D_CIRCLE_URL = getYoGraphicResource("circle.svg");
   public static final URL GRAPHIC_2D_CIRCLE_CROSS_URL = getYoGraphicResource("circle_cross.svg");
   public static final URL GRAPHIC_2D_CIRCLE_PLUS_URL = getYoGraphicResource("circle_plus.svg");
   public static final URL GRAPHIC_2D_DIAMOND_URL = getYoGraphicResource("diamond.svg");
   public static final URL GRAPHIC_2D_DIAMOND_PLUS_URL = getYoGraphicResource("diamond_plus.svg");
   public static final URL GRAPHIC_2D_SQUARE_URL = getYoGraphicResource("square.svg");
   public static final URL GRAPHIC_2D_SQUARE_CROSS_URL = getYoGraphicResource("square_cross.svg");
   public static final Path GRAPHIC_2D_CUSTOM_GRAPHICS = Paths.get(SCS2_HOME.toString(), "yoGraphic");
   public static final URL GRAPHIC_3D_SPHERE_URL = getYoGraphicResource("sphere.stl");
   public static final URL GRAPHIC_3D_CUBE_URL = getYoGraphicResource("cube.stl");
   public static final URL GRAPHIC_3D_TETRAHEDRON_URL = getYoGraphicResource("tetrahedron.stl");
   public static final URL GRAPHIC_3D_ICOSAHEDRON_URL = getYoGraphicResource("icosahedron.stl");
   public static final Path GRAPHIC_3D_CUSTOM_GRAPHICS = Paths.get(SCS2_HOME.toString(), "yoGraphic");
   // YoSliderboard resources:
   public static final URL YO_SLIDERBOARD_BCF2000_WINDOW_URL = getFXMLResource(YO_SLIDERBOARD_BCF2000, "YoBCF2000SliderboardWindow");

   public static final URL MAIN_WINDOW_URL = getFXMLResource("MainWindow");
   public static final URL SECONDARY_WINDOW_URL = getFXMLResource("SecondaryWindow");

   public static final URL SIDE_PANE_URL = getFXMLResource("SidePane");
   public static final URL USER_SIDE_PANE_URL = getFXMLResource("UserSidePane");
   public static final URL VIDEO_PREVIEW_PANE_URL = getFXMLResource("VideoRecordingPreviewPane");
   public static final URL SESSION_DATA_EXPORT_STAGE_URL = getFXMLResource("SessionDataExportStage");

   public static final URL CHART_PANEL_FXML_URL = getFXMLResource(CHART, "YoChartPanel");
   public static final URL CHART_GROUP_PANEL_URL = getFXMLResource(CHART, "YoChartGroupPanel");
   public static final URL CHART_GROUP_MODEL_EDITOR_PANE_URL = getFXMLResource(CHART, "YoChartGroupModelEditorPane");
   public static final URL CHART_IDENTIFIER_EDITOR_PANE_URL = getFXMLResource(CHART, "YoChartIdentifierEditorPane");
   public static final URL CHART_OPTION_DIALOG_URL = getFXMLResource(CHART, "YoChartOptionDialog");
   public static final URL CHART_VARIABLE_OPTION_PANE_URL = getFXMLResource(CHART, "YoChartVariableOptionPane");
   public static final URL CHART_BASELINE_EDITOR_PANE_URL = getFXMLResource(CHART, "YoChartBaselineEditorPane");

   public static final URL YO_GRAPHIC_ITEM_CREATOR_URL = getFXMLResource(YO_GRAPHIC, "YoGraphicItemCreatorDialog");
   public static final URL YO_GRAPHIC_PROPERTY_URL = getFXMLResource(YO_GRAPHIC, "YoGraphicPropertyWindow");
   public static final URL YO_COMPOSITE_EDITOR_URL = getFXMLResource(YO_GRAPHIC_SUB_EDITOR, "YoCompositeEditorPane");
   public static final URL YO_GRAPHIC_ROBOT_COLLISIONS_BUTTON_URL = getFXMLResource(YO_GRAPHIC, "YoGraphicRobotCollisionsToggleButton");
   public static final URL YO_GRAPHIC_TERRAIN_COLLISIONS_BUTTON_URL = getFXMLResource(YO_GRAPHIC, "YoGraphicTerrainCollisionsToggleButton");
   public static final URL YO_GRAPHIC_ROBOT_MASS_PROPERTIES_BUTTON_URL = getFXMLResource(YO_GRAPHIC, "YoGraphicRobotMassPropertiesToggleButton");

   // Session resources:
   public static final URL REMOTE_SESSION_MANAGER_PANE_FXML_URL = getFXMLResource(SESSION_FOLDER, "RemoteSessionManagerPane");
   public static final URL REMOTE_SESSION_INFO_PANE_FXML_URL = getFXMLResource(SESSION_FOLDER, "YoClientInformationPane");
   public static final URL LOG_SESSION_MANAGER_PANE_FXML_URL = getFXMLResource(SESSION_FOLDER, "LogSessionManagerPane");
   public static final URL LOG_CROP_PROGRESS_PANE_FXML_URL = getFXMLResource(SESSION_FOLDER, "LogCropProgressPane");

   // Cloudy Crown Skybox
   public static final String SKYBOX_CLOUDY_FOLDER = "cloudy/";
   public static final Image SKYBOX_TOP_IMAGE = createImage(getSkyboxResource(SKYBOX_CLOUDY_FOLDER + "Up.png"));
   public static final Image SKYBOX_BOTTOM_IMAGE = createImage(getSkyboxResource(SKYBOX_CLOUDY_FOLDER + "Down.png"));
   public static final Image SKYBOX_LEFT_IMAGE = createImage(getSkyboxResource(SKYBOX_CLOUDY_FOLDER + "Left.png"));
   public static final Image SKYBOX_RIGHT_IMAGE = createImage(getSkyboxResource(SKYBOX_CLOUDY_FOLDER + "Right.png"));
   public static final Image SKYBOX_FRONT_IMAGE = createImage(getSkyboxResource(SKYBOX_CLOUDY_FOLDER + "Front.png"));
   public static final Image SKYBOX_BACK_IMAGE = createImage(getSkyboxResource(SKYBOX_CLOUDY_FOLDER + "Back.png"));

   // SCS 1 Skybox
   public static final String SKYBOX_SCS1_FOLDER = "brightSky/";
   public static final Image SCS1_SKYBOX_TOP_IMAGE = createImage(getSkyboxResource(SKYBOX_SCS1_FOLDER + "up.bmp"));
   public static final Image SCS1_SKYBOX_BOTTOM_IMAGE = createImage(getSkyboxResource(SKYBOX_SCS1_FOLDER + "down.bmp"));
   public static final Image SCS1_SKYBOX_LEFT_IMAGE = createImage(getSkyboxResource(SKYBOX_SCS1_FOLDER + "west.bmp"));
   public static final Image SCS1_SKYBOX_RIGHT_IMAGE = createImage(getSkyboxResource(SKYBOX_SCS1_FOLDER + "east.bmp"));
   public static final Image SCS1_SKYBOX_FRONT_IMAGE = createImage(getSkyboxResource(SKYBOX_SCS1_FOLDER + "north.bmp"));
   public static final Image SCS1_SKYBOX_BACK_IMAGE = createImage(getSkyboxResource(SKYBOX_SCS1_FOLDER + "south.bmp"));

   public static void addSCSIconToDialog(Dialog<?> dialog)
   {
      addSCSIconToWindow(dialog.getDialogPane().getScene().getWindow());
   }

   public static void addSCSIconToWindow(Window window)
   {
      if (window instanceof Stage)
      {
         ((Stage) window).getIcons().add(SCS_ICON_IMAGE);
      }
   }

   public static URL getCSSResource(String filename)
   {
      return classLoader.getResource(CSS_FOLDER + filename + ".css");
   }

   public static URL getFXMLResource(String filename)
   {
      return classLoader.getResource(FXML_FOLDER + filename + ".fxml");
   }

   public static URL getFXMLResource(String location, String filename)
   {
      return classLoader.getResource(FXML_FOLDER + location + filename + ".fxml");
   }

   public static InputStream getYoGraphicFXIconResource(Class<? extends YoGraphicFXItem> yoGraphicFXType)
   {
      if (yoGraphicFXType == null)
         return null;

      return getIconResource(YO_GRAPHIC_FOLDER + yoGraphicFXType.getSimpleName() + ".png");
   }

   public static FXMLLoader getYoGraphicFXEditorFXMLLoader(Class<? extends YoGraphicFXItem> yoGraphicFXType)
   {
      String location = null;
      if (YoGraphicFX2D.class.isAssignableFrom(yoGraphicFXType))
         location = YO_GRAPHIC_2D;
      else if (YoGraphicFX3D.class.isAssignableFrom(yoGraphicFXType))
         location = YO_GRAPHIC_3D;
      else
         throw new IllegalArgumentException("Unhandled graphic type: " + yoGraphicFXType.getSimpleName());

      URL fxmlResource = getFXMLResource(location, yoGraphicFXType.getSimpleName() + "EditorPane");
      Objects.requireNonNull(fxmlResource, "Could not find FXML resource for " + yoGraphicFXType.getSimpleName());
      return new FXMLLoader(fxmlResource);
   }

   public static FXMLLoader getYoGraphicFXGroupEditorFXMLLoader(Class<? extends YoGraphicFXItem> yoGraphicFXType)
   {
      String location = null;
      if (YoGraphicFX2D.class.isAssignableFrom(yoGraphicFXType))
         location = YO_GRAPHIC_GROUP + "graphic2D/";
      else if (YoGraphicFX3D.class.isAssignableFrom(yoGraphicFXType))
         location = YO_GRAPHIC_GROUP + "graphic3D/";
      else
         throw new IllegalArgumentException("Unhandled graphic type: " + yoGraphicFXType.getSimpleName());

      URL fxmlResource = getFXMLResource(location, yoGraphicFXType.getSimpleName() + "GroupEditorPane");
      Objects.requireNonNull(fxmlResource, "Could not find FXML resource for grouped " + yoGraphicFXType.getSimpleName());
      return new FXMLLoader(fxmlResource);
   }

   public static InputStream getIconResource(String iconNameWithExtension)
   {
      return classLoader.getResourceAsStream(ICONS_FOLDER + iconNameWithExtension);
   }

   public static InputStream getImageResource(String imageNameWithExtension)
   {
      return classLoader.getResourceAsStream(IMAGES_FOLDER + imageNameWithExtension);
   }

   public static InputStream getSkyboxResource(String skyboxNameWithExtension)
   {
      return classLoader.getResourceAsStream(SKYBOX_FOLDER + skyboxNameWithExtension);
   }

   public static InputStream getConfigurationResource(String filenameWithExtension)
   {
      return classLoader.getResourceAsStream(CONFIGURATION_FOLDER + filenameWithExtension);
   }

   public static URL getYoGraphicResource(String filenameWithExtension)
   {
      return classLoader.getResource(YO_GRAPHIC_FOLDER + filenameWithExtension);
   }

   public static URL getYoGraphicResource(String location, String filenameWithExtension)
   {
      return classLoader.getResource(YO_GRAPHIC_FOLDER + location + filenameWithExtension);
   }

   public static Image loadIcon(String iconNameWithExtension)
   {
      return createImage(getIconResource(iconNameWithExtension));
   }

   public static Image loadImage(String imageNameWithExtension)
   {
      return createImage(getImageResource(imageNameWithExtension));
   }

   public static Image createImage(InputStream is)
   {
      Image image = new Image(is);
      try
      {
         is.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      return image;
   }

   public static File scs2ConfigurationOpenFileDialog(Window owner)
   {
      return showOpenDialog(owner, "Load SCS2 Configuration", scs2ConfigurationFilter);
   }

   public static File scs2ConfigurationSaveFileDialog(Window owner)
   {
      return showSaveDialog(owner, "Save SCS2 Configuration", scs2ConfigurationFilter);
   }

   public static File yoChartConfigurationOpenFileDialog(Window owner)
   {
      return showOpenDialog(owner, "Load YoChartGroup", yoChartGroupConfigurationFilter);
   }

   public static File yoChartConfigurationSaveFileDialog(Window owner)
   {
      return showSaveDialog(owner, "Save YoChartGroup", yoChartGroupConfigurationFilter);
   }

   public static File yoGraphicConfigurationOpenFileDialog(Window owner)
   {
      return showOpenDialog(owner, "Load YoGraphic", yoGraphicConfigurationFilter);
   }

   public static File yoGraphicConfigurationSaveFileDialog(Window owner)
   {
      return showSaveDialog(owner, "Save YoGraphic", yoGraphicConfigurationFilter);
   }

   public static File yoCompositeConfigurationOpenFileDialog(Window owner)
   {
      return showOpenDialog(owner, "Load YoComposite", yoCompositeConfigurationFilter);
   }

   public static File yoCompositeConfigurationSaveFileDialog(Window owner)
   {
      return showSaveDialog(owner, "Save YoComposite", yoCompositeConfigurationFilter);
   }

   public static File yoEntryConfigurationOpenFileDialog(Window owner)
   {
      return showOpenDialog(owner, "Load YoEntry", yoEntryConfigurationFilter);
   }

   public static File yoEntryConfigurationSaveFileDialog(Window owner)
   {
      return showSaveDialog(owner, "Save YoEntry", yoEntryConfigurationFilter);
   }

   public static File yoSliderboardConfigurationOpenFileDialog(Window owner)
   {
      return showOpenDialog(owner, "Load YoSliderboard", yoSliderboardConfigurationFilter);
   }

   public static File yoSliderboardConfigurationSaveFileDialog(Window owner)
   {
      return showSaveDialog(owner, "Save YoSliderboard", yoSliderboardConfigurationFilter);
   }

   private static File showSaveDialog(Window owner, String title, ExtensionFilter extensionFilter)
   {
      return showSaveDialog(owner, title, extensionFilter, "filePath");
   }

   private static File showSaveDialog(Window owner, String title, ExtensionFilter extensionFilter, String pathKey)
   {
      FileChooser fileChooser = fileChooser(title, extensionFilter);
      File result = fileChooser.showSaveDialog(owner);

      if (result != null)
      {
         // FIXME: This is to address what seems to be a bug with the FileChooser on Windows.
         // When saving, if you select a file with the same extension, then modify the name, the FileChooser will append the file extension another time.
         if (extensionFilter != null && !extensionFilter.getExtensions().isEmpty())
         {
            String filename = result.getName();

            for (String extension : extensionFilter.getExtensions())
            {
               if (extension.charAt(0) == '*')
                  extension = extension.substring(1);

               int firstIndexOfExtension = filename.indexOf(extension);

               if (firstIndexOfExtension == -1)
               {
                  continue;
               }
               else if (firstIndexOfExtension == filename.length() - extension.length())
               {
                  break;
               }
               else
               {
                  String newFilename = filename.substring(0, firstIndexOfExtension) + extension;
                  result = new File(result.getParentFile(), newFilename);
                  // No need to worry if the file already exists, it would have caused a prompt in the FileChooser.
                  break;
               }
            }
         }
         setDefaultFilePath(result);
      }

      return result;
   }

   private static File showOpenDialog(Window owner, String title, ExtensionFilter extensionFilter)
   {
      return showOpenDialog(owner, title, extensionFilter, "filePath");
   }

   private static File showOpenDialog(Window owner, String title, ExtensionFilter extensionFilter, String pathKey)
   {
      FileChooser fileChooser = fileChooser(title, extensionFilter);
      File result = fileChooser.showOpenDialog(owner);
      if (result != null)
         setDefaultFilePath(result);
      return result;
   }

   private static FileChooser fileChooser(String title, ExtensionFilter extensionFilter)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setTitle(title);
      fileChooser.setInitialDirectory(getDefaultFilePath());
      fileChooser.getExtensionFilters().add(extensionFilter);
      return fileChooser;
   }

   /**
    * Returns the file that was last opened or saved to.
    *
    * @return the most-recently-used file.
    */
   public static File getDefaultFilePath()
   {
      return getDefaultFilePath("filePath");
   }

   public static File getDefaultFilePath(String key)
   {
      Preferences prefs = Preferences.userNodeForPackage(SessionVisualizerIOTools.class);
      String filePath = prefs.get(key, null);

      if (filePath != null && Files.isDirectory(Paths.get(filePath)))
         return new File(filePath);
      else
         return null;
   }

   /**
    * Stores the given file's path as the most-recently-used path. The path is persisted across program
    * runs.
    *
    * @param file the file
    */
   public static void setDefaultFilePath(File file)
   {
      setDefaultFilePath("filePath", file);
   }

   public static void setDefaultFilePath(String key, File file)
   {
      Preferences prefs = Preferences.userNodeForPackage(SessionVisualizerIOTools.class);
      if (file != null)
      {
         if (!file.isDirectory())
            file = file.getParentFile();

         prefs.put(key, file.getAbsolutePath());
      }
   }
}
