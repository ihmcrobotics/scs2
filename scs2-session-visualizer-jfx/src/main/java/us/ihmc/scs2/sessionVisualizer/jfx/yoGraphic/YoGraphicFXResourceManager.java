package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commons.nio.FileTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;

public class YoGraphicFXResourceManager
{
   public static final double SVG_SCALE = 0.001;
   public static final YoGraphicFXResource DEFAULT_POINT2D_GRAPHIC_RESOURCE = new YoGraphicFXResource(graphicName(SessionVisualizerIOTools.GRAPHIC_2D_CIRCLE_URL),
                                                                                                      SessionVisualizerIOTools.GRAPHIC_2D_CIRCLE_URL);
   public static final YoGraphicFXResource DEFAULT_POINT3D_GRAPHIC_RESOURCE = new YoGraphicFXResource(graphicName(SessionVisualizerIOTools.GRAPHIC_3D_SPHERE_URL),
                                                                                                      SessionVisualizerIOTools.GRAPHIC_3D_SPHERE_URL);

   private final Map<String, URL> graphic2DResourceMap = new LinkedHashMap<>();
   private final Map<String, URL> graphic3DResourceMap = new LinkedHashMap<>();

   public YoGraphicFXResourceManager()
   {
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_CROSS_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_PLUS_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_CIRCLE_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_CIRCLE_CROSS_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_CIRCLE_PLUS_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_DIAMOND_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_DIAMOND_PLUS_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_SQUARE_URL);
      registerNewGraphic2D(SessionVisualizerIOTools.GRAPHIC_2D_SQUARE_CROSS_URL);
      registerNewGraphic3D(SessionVisualizerIOTools.GRAPHIC_3D_SPHERE_URL);
      registerNewGraphic3D(SessionVisualizerIOTools.GRAPHIC_3D_CUBE_URL);
      registerNewGraphic3D(SessionVisualizerIOTools.GRAPHIC_3D_TETRAHEDRON_URL);
      registerNewGraphic3D(SessionVisualizerIOTools.GRAPHIC_3D_ICOSAHEDRON_URL);
      registerCustomGraphics(SessionVisualizerIOTools.GRAPHIC_2D_CUSTOM_GRAPHICS, graphic2DResourceMap, Collections.singleton(".svg"));
      registerCustomGraphics(SessionVisualizerIOTools.GRAPHIC_3D_CUSTOM_GRAPHICS, graphic3DResourceMap, Arrays.asList(".stl", ".obj", ".dae"));
   }

   private void registerCustomGraphics(Path customGraphicsFolder, Map<String, URL> resourceMap, Collection<String> acceptableFileExtensions)
   {
      try
      {
         FileTools.ensureDirectoryExists(customGraphicsFolder);
         File customGraphicFolder = customGraphicsFolder.toFile();
         File[] customGraphics = customGraphicFolder.listFiles(file -> file.isFile()
               && acceptableFileExtensions.contains(file.getName().substring(file.getName().lastIndexOf("."), file.getName().length())));

         for (File customGraphic : customGraphics)
         {
            URL graphicURL = customGraphic.toURI().toURL();
            resourceMap.putIfAbsent(graphicName(graphicURL), graphicURL);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public Collection<String> getGraphic2DNameList()
   {
      return graphic2DResourceMap.keySet();
   }

   public Collection<String> getGraphic3DNameList()
   {
      return graphic3DResourceMap.keySet();
   }

   public void registerNewGraphic2D(URL graphicURL)
   {
      graphic2DResourceMap.putIfAbsent(graphicName(graphicURL), graphicURL);
   }

   public void registerNewGraphic3D(URL graphicURL)
   {
      graphic3DResourceMap.putIfAbsent(graphicName(graphicURL), graphicURL);
   }

   public YoGraphicFXResource loadGraphic2DResource(String resourceName)
   {
      return new YoGraphicFXResource(resourceName, graphic2DResourceMap.get(resourceName));
   }

   public YoGraphicFXResource loadGraphic3DResource(String resourceName)
   {
      return new YoGraphicFXResource(resourceName, graphic3DResourceMap.get(resourceName));
   }

   public static String graphicName(URL graphicURL)
   {
      String graphicName;
      FileSystem fs = null;

      try
      {
         Path path;
         URI graphicURI = graphicURL.toURI();

         if (graphicURI.toString().contains("!"))
         { // The resource in inside a Jar.
            Map<String, String> env = new HashMap<>();
            String[] array = graphicURI.toString().split("!");
            fs = FileSystems.newFileSystem(URI.create(array[0]), env);
            path = fs.getPath(array[1]);
         }
         else
         {
            path = Paths.get(graphicURI);
         }
         graphicName = path.getFileName().toString();
      }
      catch (URISyntaxException | IOException e)
      {
         throw new RuntimeException(e);
      }
      finally
      {
         try
         {
            if (fs != null)
               fs.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      if (graphicName.contains("."))
         graphicName = graphicName.substring(0, graphicName.lastIndexOf("."));

      graphicName = graphicName.replace("_", " ");
      graphicName = Character.toUpperCase(graphicName.charAt(0)) + graphicName.substring(1);
      return graphicName;
   }
}
