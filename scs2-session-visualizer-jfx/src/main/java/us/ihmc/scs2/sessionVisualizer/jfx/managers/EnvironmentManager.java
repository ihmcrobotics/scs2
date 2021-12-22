package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.List;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.SubScene;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.Skybox;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class EnvironmentManager implements Manager
{
   private final Group rootNode = new Group();
   private final Group terrainObjectGraphics = new Group();
   private Group staticVisualsRoot;
   private Skybox skybox;

   private final BackgroundExecutorManager backgroundExecutorManager;

   public EnvironmentManager(BackgroundExecutorManager backgroundExecutorManager)
   {
      this.backgroundExecutorManager = backgroundExecutorManager;
      rootNode.getChildren().add(terrainObjectGraphics);
   }

   public void addWorldCoordinateSystem(double size)
   {
      backgroundExecutorManager.executeInBackground(() ->
      {
         Node node = new JavaFXCoordinateSystem(size);
         JavaFXMissingTools.runLater(getClass(), () -> rootNode.getChildren().add(node));
      });
   }

   public void addSkybox(SubScene subScene)
   {
      if (skybox != null)
         return;

      skybox = new Skybox();
      skybox.setupCloudyCrown();
      skybox.setupCamera(subScene.getCamera());

      backgroundExecutorManager.executeInBackground(() ->
      {
         JavaFXMissingTools.runLater(getClass(), () -> rootNode.getChildren().add(skybox));
      });
   }

   public void addStaticVisual(VisualDefinition visualDefinition)
   {
      Node node = JavaFXVisualTools.toNode(visualDefinition, null);

      if (staticVisualsRoot == null)
      {
         staticVisualsRoot = new Group();
         JavaFXMissingTools.runLater(getClass(), () ->
         {
            staticVisualsRoot.getChildren().add(node);
            rootNode.getChildren().add(staticVisualsRoot);
         });
      }
      else
      {
         JavaFXMissingTools.runLater(getClass(), () ->
         {
            staticVisualsRoot.getChildren().add(node);
         });
      }
   }

   @Override
   public void startSession(Session session)
   {
      List<TerrainObjectDefinition> terrainObjectDefinitions = session.getTerrainObjectDefinitions();
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         for (TerrainObjectDefinition definition : terrainObjectDefinitions)
         {
            Node nodes = JavaFXVisualTools.collectNodes(definition.getVisualDefinitions(), definition.getResourceClassLoader());
            if (nodes != null)
               terrainObjectGraphics.getChildren().add(nodes);
         }
      });
   }

   @Override
   public void stopSession()
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> terrainObjectGraphics.getChildren().clear());
   }

   @Override
   public boolean isSessionLoaded()
   {
      return true;
   }

   public Group getRootNode()
   {
      return rootNode;
   }

   public void dispose()
   {
      if (skybox != null)
      {
         skybox.dispose();
         skybox = null;
      }

      rootNode.getChildren().clear();
      terrainObjectGraphics.getChildren().clear();

      if (staticVisualsRoot != null)
      {
         staticVisualsRoot.getChildren().clear();
         staticVisualsRoot = null;
      }
   }
}
