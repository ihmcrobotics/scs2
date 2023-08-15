package us.ihmc.scs2.sessionVisualizer.jfx.fxml;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.MalformedURLException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
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
            List<URL> fxmlResources = findResources("fxml", path -> path.getFileName().toString().endsWith(".fxml"));

            for (URL fxmlResource : fxmlResources)
            {
               System.out.println(fxmlResource);
               FXMLLoader loader = new FXMLLoader(fxmlResource);
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

         assertNotNull(field.get(null), "Test failed for field: " + field.getName());
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

   private static List<URL> findResources(String folder, Predicate<Path> filter) throws URISyntaxException, IOException
   {
      ClassLoader loader = Thread.currentThread().getContextClassLoader();
      URL url = loader.getResource(folder);
      URI uri = url.toURI();

      Path myPath;
      if (uri.getScheme().equals("jar"))
      {
         FileSystem fileSystem = FileSystems.newFileSystem(uri, Collections.emptyMap());
         myPath = fileSystem.getPath(folder);
      }
      else
      {
         myPath = Paths.get(uri);
      }
      return Files.walk(myPath, Integer.MAX_VALUE).filter(filter).map(path ->
      {
         try
         {
            return path.toUri().toURL();
         }
         catch (MalformedURLException e)
         {
            throw new RuntimeException(e);
         }
      }).toList();
   }
}
