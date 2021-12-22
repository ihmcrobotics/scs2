package us.ihmc.scs2.examples.sessionVisualizer.jfx.model.viewer;

import java.io.File;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.xml.bind.JAXBException;

import javafx.application.Platform;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFModel;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.FrameNode;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.RigidBodyFrameNodeFactories;

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
         fileChooser.getExtensionFilters().addAll(new ExtensionFilter("All Model Files", "*.urdf", "*.sdf"),
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
            urdfModel = URDFTools.loadURDFModel(modelFile);
         }
         catch (JAXBException e)
         {
            throw new RuntimeException(e);
         }
         robotDefinition = URDFTools.toFloatingRobotDefinition(urdfModel);
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


      View3DFactory view3dFactory = new View3DFactory(1024, 768);
      view3dFactory.addWorldCoordinateSystem(0.2);
      frameNodes.values().forEach(frameNode -> view3dFactory.addNodeToView(frameNode.getNode()));
      view3dFactory.addCameraController();

      primaryStage.setTitle("Robot Model Viewer");
      primaryStage.setScene(view3dFactory.getScene());
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
