package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javafx.scene.Camera;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.Skybox;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class EnvironmentManager implements Manager
{
   private static final int LARGE_TRIANGLE_MESH_THRESHOLD = 1000000;

   private final Group rootNode = new Group();
   private final Group terrainObjectGraphics = new Group();
   private Group staticVisualsRoot;
   private Map<VisualDefinition, Node> staticVisualDefinitionToNodeMap;
   private Skybox skybox;

   private final BackgroundExecutorManager backgroundExecutorManager;

   public EnvironmentManager(JavaFXMessager messager, SessionVisualizerTopics topics, BackgroundExecutorManager backgroundExecutorManager)
   {
      this.backgroundExecutorManager = backgroundExecutorManager;
      rootNode.getChildren().add(terrainObjectGraphics);
      messager.registerTopicListener(topics.getTerrainVisualRequest(), this::handleTerrainVisualRequest);
   }

   private void handleTerrainVisualRequest(NewTerrainVisualRequest request)
   {
      if (request.getRequestedVisible() != null)
         terrainObjectGraphics.setVisible(request.getRequestedVisible());

      if (request.getRequestedDrawMode() != null)
         setTerrainDrawMode(request.getRequestedDrawMode());
   }

   public void addWorldCoordinateSystem(double size)
   {
      backgroundExecutorManager.executeInBackground(() ->
      {
         Node node = new JavaFXCoordinateSystem(size);
         JavaFXMissingTools.runLater(getClass(), () -> rootNode.getChildren().add(node));
      });
   }

   public void addSkybox(Camera mainCamera)
   {
      if (skybox != null)
         return;

      skybox = new Skybox();

      if (Skybox.USE_CLOUDY_CROWN_SKY_BOX)
         skybox.setupCloudyCrown();
      else
         skybox.setupSCS1Skybox();

      skybox.setupCamera(mainCamera);

      backgroundExecutorManager.executeInBackground(() ->
      {
         JavaFXMissingTools.runLater(getClass(), () -> rootNode.getChildren().add(skybox));
      });
   }

   public void addStaticVisual(VisualDefinition visualDefinition)
   {
      if (staticVisualDefinitionToNodeMap == null)
      {
         staticVisualDefinitionToNodeMap = new HashMap<>();
         if (staticVisualDefinitionToNodeMap.containsKey(visualDefinition))
            return; // This visual was already added
      }

      Node nodeToAdd = JavaFXVisualTools.toNode(visualDefinition, null);

      // Test if the new mesh is a large triangle mesh, if so, we make mouse transparent to improve performance.
      if (nodeToAdd instanceof MeshView)
      {
         Mesh mesh = ((MeshView) nodeToAdd).getMesh();
         if (mesh instanceof TriangleMesh)
         {
            if (((TriangleMesh) mesh).getPoints().size() > LARGE_TRIANGLE_MESH_THRESHOLD)
               nodeToAdd.setMouseTransparent(true);
         }
      }

      staticVisualDefinitionToNodeMap.put(visualDefinition, nodeToAdd);

      if (staticVisualsRoot == null)
      {
         staticVisualsRoot = new Group();

         JavaFXMissingTools.runLater(getClass(), () ->
         {
            staticVisualsRoot.getChildren().add(nodeToAdd);
            rootNode.getChildren().add(staticVisualsRoot);
         });
      }
      else
      {
         JavaFXMissingTools.runLater(getClass(), () ->
         {
            staticVisualsRoot.getChildren().add(nodeToAdd);
         });
      }
   }

   public void removeStaticVisual(VisualDefinition visualDefinition)
   {
      if (staticVisualDefinitionToNodeMap == null)
         return;

      if (staticVisualsRoot == null)
         return;

      Node nodeToRemove = staticVisualDefinitionToNodeMap.remove(visualDefinition);

      if (nodeToRemove == null)
         return;

      JavaFXMissingTools.runLater(getClass(), () ->
      {
         staticVisualsRoot.getChildren().remove(nodeToRemove);
      });
   }

   public void setTerrainDrawMode(DrawMode drawMode)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> JavaFXMissingTools.setDrawModeRecursive(terrainObjectGraphics, drawMode));
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
