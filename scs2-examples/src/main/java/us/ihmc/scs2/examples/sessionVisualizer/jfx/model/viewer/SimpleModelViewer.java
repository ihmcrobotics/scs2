package us.ihmc.scs2.examples.sessionVisualizer.jfx.model.viewer;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.paint.Color;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFModel;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.ApplicationRunner;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.Scene3DBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.FrameNode;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.RigidBodyFrameNodeFactories;

import javax.xml.bind.JAXBException;
import java.io.File;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public class SimpleModelViewer
{
   private static final ReferenceFrame rootFrame = ReferenceFrame.getWorldFrame();
   private static final String MODEL_FILE_KEY = "ROBOT_MODEL_FILE_SIMPLE_VIEWER";
   private File modelFile = null;

   public SimpleModelViewer(Stage primaryStage)
   {
      if (modelFile == null)
      {
         FileChooser fileChooser = new FileChooser();
         fileChooser.getExtensionFilters()
                    .addAll(new ExtensionFilter("All Model Files", "*.urdf", "*.sdf"),
                            new ExtensionFilter("URDF Files", "*.urdf"),
                            new ExtensionFilter("SDF Files", "*.sdf"));
         fileChooser.setInitialDirectory(SessionVisualizerIOTools.getDefaultFilePath(MODEL_FILE_KEY));
         modelFile = fileChooser.showOpenDialog(primaryStage);
      }

      if (modelFile == null)
      {
         Platform.exit();
         return;
      }

      SessionVisualizerIOTools.setDefaultFilePath(MODEL_FILE_KEY, modelFile);

      RobotDefinition robotDefinition;

      if (modelFile.getName().toLowerCase().endsWith("urdf"))
      {
         URDFModel urdfModel;
         try
         {
            urdfModel = URDFTools.loadURDFModel(modelFile, Collections.singletonList(modelFile.getParent()));
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
         robotDefinition = URDFTools.toRobotDefinition(urdfModel);
      }
      else if (modelFile.getName().toLowerCase().endsWith("sdf"))
      {
         SDFModel sdfModel;
         try
         {
            sdfModel = SDFTools.loadSDFRoot(modelFile).getModels().get(0);
         }
         catch (JAXBException e)
         {
            throw new RuntimeException(e);
         }
         robotDefinition = SDFTools.toFloatingRobotDefinition(sdfModel);
      }
      else
      {
         LogTools.error("Unhandled file type: " + modelFile.getName());
         return;
      }

      RigidBodyBasics rigidBody = robotDefinition.newInstance(rootFrame);
      Map<String, FrameNode> frameNodes = new LinkedHashMap<>();
      RigidBodyFrameNodeFactories.createRobotFrameNodeMap(rigidBody, robotDefinition, null, frameNodes);
      rigidBody.updateFramesRecursively();
      frameNodes.values().forEach(FrameNode::updatePose);

      Scene3DBuilder scene3DBuilder = new Scene3DBuilder();
      Scene scene = new Scene(scene3DBuilder.getRoot(), 1024, 768, true, SceneAntialiasing.BALANCED);
      scene.setFill(Color.GREY);
      scene3DBuilder.addCoordinateSystem(0.2);
      frameNodes.values().forEach(frameNode -> scene3DBuilder.addNodeToView(frameNode.getNode()));
      Simple3DViewer.setupCamera(scene, scene3DBuilder.getRoot());

      primaryStage.setTitle("Robot Model Viewer");
      primaryStage.setScene(scene);
      primaryStage.show();
   }

   public void stop()
   {
      System.out.println(getClass().getSimpleName() + " is going down.");
   }

   public static void main(String[] args)
   {
      ApplicationRunner.runApplication(SimpleModelViewer::new);
   }
}
