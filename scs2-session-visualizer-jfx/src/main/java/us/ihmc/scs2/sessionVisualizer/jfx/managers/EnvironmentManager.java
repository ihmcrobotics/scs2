package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.io.File;
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
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.Skybox;
import us.ihmc.scs2.sessionVisualizer.jfx.Skybox.SkyboxTheme;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class EnvironmentManager implements Manager
{
   public static final SkyboxTheme SKYBOX_THEME = SessionPropertiesHelper.loadEnumPropertyOrEnvironment("scs2.session.gui.skybox.theme",
                                                                                                        "SCS2_SKYBOX_THEME",
                                                                                                        SkyboxTheme.class,
                                                                                                        SkyboxTheme.CLOUDY_CROWN_MIDDAY);
   public static final String SKYBOX_CUSTOM_PATH = SessionPropertiesHelper.loadStringPropertyOrEnvironment("scs2.session.gui.skybox.custompath",
                                                                                                           "SCS2_SKYBOX_CUSTOM_PATH",
                                                                                                           null);

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
      messager.addTopicListener(topics.getTerrainVisualRequest(), this::handleTerrainVisualRequest);
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

      switch (SKYBOX_THEME)
      {
         case CLOUDY_CROWN_MIDDAY:
            skybox.setupCloudyCrown();
            break;
         case SCS1:
            skybox.setupSCS1Skybox();
            break;
         case CUSTOM:
            if (SKYBOX_CUSTOM_PATH == null)
            {
               LogTools.warn("Could not load custom skybox, needs to set the path.");
               skybox.setupCloudyCrown();
            }
            else
            {
               if (!skybox.loadSkyboxFlexible(new File(SKYBOX_CUSTOM_PATH)))
                  skybox.setupCloudyCrown();
            }
            break;
         default:
            throw new IllegalArgumentException("Unexpected value: " + SKYBOX_THEME);
      }

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
