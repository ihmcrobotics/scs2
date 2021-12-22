package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import javax.xml.bind.JAXBException;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.jointState.OneDoFState;
import us.ihmc.robotDataLogger.jointState.SixDoFState;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RobotModelLoader
{
   private final static String resourceDirectoryLocation = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "resources";

   private static final TLongObjectHashMap<RobotDefinition> cachedImportedModels = new TLongObjectHashMap<>();

   public static Runnable setupRobotUpdater(RobotDefinition robotDefinition, YoVariableHandshakeParser handshakeParser, YoRegistry rootRegistry, ReferenceFrame inertialFrame)
   {
      if (robotDefinition == null)
         return null;

      RobotInterface robot = new Robot(robotDefinition, inertialFrame);

      Map<String, JointState> jointNameToState = handshakeParser.getJointStates().stream().collect(Collectors.toMap(JointState::getName, Function.identity()));

      List<Runnable> jointStateUpdaters = new ArrayList<>();

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, robot.getRootBody()).forEach(oneDoFJoint ->
      {
         OneDoFState jointState = (OneDoFState) jointNameToState.get(oneDoFJoint.getName());
         jointStateUpdaters.add(() -> {
            oneDoFJoint.setQ(jointState.getQ());
            oneDoFJoint.setQd(jointState.getQd());
         });
      });

      SixDoFJointBasics floatingJoint = (SixDoFJointBasics) robot.getRootBody().getChildrenJoints().get(0);
      SixDoFState jointState = (SixDoFState) jointNameToState.get(floatingJoint.getSuccessor().getName());

      jointStateUpdaters.add(() ->
      {
         floatingJoint.getJointPose().set(jointState.getTranslation(), jointState.getRotation());
         floatingJoint.getJointTwist().set(jointState.getTwistAngularPart(), jointState.getTwistLinearPart());
      });

      rootRegistry.addChild(robot.getRegistry());

      return () -> jointStateUpdaters.forEach(updater -> updater.run());
   }

   public static RobotDefinition loadModel(String modelName, String[] resourceDirectories, byte[] model, byte[] resourceZip)
   {
      if (model == null)
         return null;

      long modelHashCode = computeModelHashCode(modelName, resourceDirectories, model, resourceZip);
      RobotDefinition robotDefinition = cachedImportedModels.get(modelHashCode);

      if (robotDefinition != null)
         return robotDefinition;

      ClassLoader resourceClassLoader = unpackResources(modelName, resourceZip);
      try
      {
         SDFRoot sdfRoot = SDFTools.loadSDFRoot(new ByteArrayInputStream(model), Arrays.asList(resourceDirectories), resourceClassLoader);
         robotDefinition = SDFTools.toFloatingRobotDefinition(sdfRoot.getModels().get(0));
         robotDefinition.setResourceClassLoader(resourceClassLoader);
         cachedImportedModels.put(modelHashCode, robotDefinition);
         return robotDefinition;
      }
      catch (JAXBException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private static ClassLoader unpackResources(String modelName, byte[] resourceZip)
   {
      // Some of the model importers do not immediately release the files, triggering GC seems to help ensuring that these files are released.
      System.gc();

      if (resourceZip == null)
         return null;

      Path resourceDirectory = Paths.get(resourceDirectoryLocation, modelName);
      try
      {
         Files.createDirectories(resourceDirectory);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      ByteArrayInputStream is = new ByteArrayInputStream(resourceZip);
      ZipInputStream zip = new ZipInputStream(is);
      ZipEntry ze = null;

      try
      {
         while ((ze = zip.getNextEntry()) != null)
         {
            Path target = resourceDirectory.resolve(ze.getName());
            Files.createDirectories(target.getParent());
            Files.copy(zip, target, StandardCopyOption.REPLACE_EXISTING);
         }
      }
      catch (IOException e)
      {
         System.err.println("SDFModelLoader: Cannot load model zip file. Not unpacking robot model.");
         e.printStackTrace();
      }
      finally
      {
         try
         {
            zip.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         try
         {
            is.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      try
      {
         return new URLClassLoader(new URL[] {resourceDirectory.toUri().toURL()});
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static long computeModelHashCode(String modelName, String[] resourceDirectories, byte[] model, byte[] resourceZip)
   {
      long hash = modelName == null ? 1L : modelName.hashCode();
      hash = EuclidHashCodeTools.combineHashCode(hash, Arrays.hashCode(resourceDirectories));
      hash = EuclidHashCodeTools.combineHashCode(hash, Arrays.hashCode(model));
      hash = EuclidHashCodeTools.combineHashCode(hash, Arrays.hashCode(resourceZip));
      return hash;
   }
}
