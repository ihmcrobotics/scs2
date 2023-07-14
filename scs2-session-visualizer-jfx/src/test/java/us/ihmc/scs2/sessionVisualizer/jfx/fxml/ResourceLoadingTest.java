package us.ihmc.scs2.sessionVisualizer.jfx.fxml;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.image.Image;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class ResourceLoadingTest
{
   private static final boolean VERBOSE = true;

   @BeforeAll
   public static void startJavaFXThread()
   {
      try
      {
         Platform.startup(() ->
         {
         });
      }
      catch (Exception e)
      {
         // The toolkit may have been started already
      }
   }

   // Sanity check: verify all the FXML files are loadable 
   @Test
   public void testFXMLLoading() throws Throwable
   {
      Throwable t = JavaFXMissingTools.runAndWait(ResourceLoadingTest.class, () ->
      {
         try
         {
            List<File> fxmlFiles = findResourceFiles("fxml", file -> file.getName().endsWith(".fxml"));

            for (File fxmlFile : fxmlFiles)
            {
               FXMLLoader loader = new FXMLLoader(fxmlFile.toURI().toURL());
               Object rootPane = assertDoesNotThrow(() -> loader.load());
               assertNotNull(rootPane);

               Object controller = loader.getController();

               if (controller != null)
               {
                  // TODO Check the @FXML are initialized properly
               }
            }
            return null;
         }
         catch (Throwable e)
         {
            return e;
         }
      });

      if (t != null)
      {
         if (VERBOSE)
            t.printStackTrace();
         throw t;
      }
   }

   @Test
   public void testURLResources() throws Exception
   {
      for (Field field : SessionVisualizerIOTools.class.getFields())
      {
         if (!Modifier.isStatic(field.getModifiers()))
            continue;

         if (field.getType() != URL.class)
            continue;

         assertDoesNotThrow(() ->
         {
            URL urlResource = (URL) field.get(null);
            File fileResource = Paths.get(urlResource.toURI()).toFile();
            assertTrue(fileResource.exists());
            assertTrue(fileResource.isFile());
         }, "Test failed for field: " + field.getName());
      }
   }

   @Test
   public void testImageResources() throws Exception
   {
      for (Field field : SessionVisualizerIOTools.class.getFields())
      {
         if (!Modifier.isStatic(field.getModifiers()))
            continue;

         if (field.getType() != Image.class)
            continue;

         assertDoesNotThrow(() ->
         {
            Image imageResource = (Image) field.get(null);
            assertNotNull(imageResource);
         }, "Test failed for field: " + field.getName());
      }
   }

   private static List<File> findResourceFiles(String folder, Predicate<File> filter) throws URISyntaxException
   {
      ClassLoader loader = Thread.currentThread().getContextClassLoader();
      URL url = loader.getResource(folder);
      return collectFilesDeep(Paths.get(url.toURI()).toFile(), filter);
   }

   public static List<File> collectFilesDeep(File start, Predicate<File> filter)
   {
      return collectFilesDeep(start, filter, new ArrayList<>());
   }

   public static List<File> collectFilesDeep(File start, Predicate<File> filter, List<File> filesToPack)
   {
      if (start.isFile())
      {
         if (filter.test(start))
            filesToPack.add(start);
      }
      else
      {
         for (File child : start.listFiles())
            collectFilesDeep(child, filter, filesToPack);
      }

      return filesToPack;
   }
}
