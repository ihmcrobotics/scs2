package us.ihmc.scs2.definition.robot.sdf;

import java.io.File;
import java.io.InputStream;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.FixedJointDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.LidarSensorDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PlanarJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.robot.sdf.items.SDFCollision;
import us.ihmc.scs2.definition.robot.sdf.items.SDFGeometry;
import us.ihmc.scs2.definition.robot.sdf.items.SDFInertia;
import us.ihmc.scs2.definition.robot.sdf.items.SDFJoint;
import us.ihmc.scs2.definition.robot.sdf.items.SDFJoint.SDFAxis;
import us.ihmc.scs2.definition.robot.sdf.items.SDFJoint.SDFAxis.SDFDynamics;
import us.ihmc.scs2.definition.robot.sdf.items.SDFJoint.SDFAxis.SDFLimit;
import us.ihmc.scs2.definition.robot.sdf.items.SDFLink;
import us.ihmc.scs2.definition.robot.sdf.items.SDFLink.SDFInertial;
import us.ihmc.scs2.definition.robot.sdf.items.SDFModel;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFCamera;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFIMU;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFIMU.SDFIMUNoise;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFIMU.SDFIMUNoise.SDFNoiseParameters;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFRay;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFRay.SDFNoise;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFRay.SDFRange;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFRay.SDFScan.SDFHorizontalScan;
import us.ihmc.scs2.definition.robot.sdf.items.SDFSensor.SDFRay.SDFScan.SDFVerticalScan;
import us.ihmc.scs2.definition.robot.sdf.items.SDFURIHolder;
import us.ihmc.scs2.definition.robot.sdf.items.SDFVisual;
import us.ihmc.scs2.definition.robot.sdf.items.SDFVisual.SDFMaterial;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class SDFTools
{
   private static final double DEFAULT_MASS = 0.0;
   private static final double DEFAULT_IXX = 0.0;
   private static final double DEFAULT_IYY = 0.0;
   private static final double DEFAULT_IZZ = 0.0;
   private static final double DEFAULT_IXY = 0.0;
   private static final double DEFAULT_IXZ = 0.0;
   private static final double DEFAULT_IYZ = 0.0;

   private static final Vector3DReadOnly DEFAULT_AXIS = new Vector3D(1.0, 0.0, 0.0);
   private static final double DEFAULT_LOWER_LIMIT = Double.NEGATIVE_INFINITY;
   private static final double DEFAULT_UPPER_LIMIT = Double.POSITIVE_INFINITY;
   private static final double DEFAULT_EFFORT_LIMIT = Double.POSITIVE_INFINITY;
   private static final double DEFAULT_VELOCITY_LIMIT = Double.POSITIVE_INFINITY;

   public static SDFRoot loadSDFRoot(File sdfFile) throws JAXBException
   {
      return loadSDFRoot(sdfFile, Collections.emptyList());
   }

   public static SDFRoot loadSDFRoot(File sdfFile, Collection<String> resourceDirectories) throws JAXBException
   {
      Set<String> allResourceDirectories = new HashSet<>(resourceDirectories);
      File parentFile = sdfFile.getParentFile();

      if (parentFile != null)
      {
         allResourceDirectories.add(parentFile.getAbsolutePath() + File.separator);
         Stream.of(parentFile.listFiles(File::isDirectory)).map(file -> file.getAbsolutePath() + File.separator).forEach(allResourceDirectories::add);
      }

      JAXBContext context = JAXBContext.newInstance(SDFRoot.class);
      Unmarshaller um = context.createUnmarshaller();
      SDFRoot sdfRoot = (SDFRoot) um.unmarshal(sdfFile);

      resolvePaths(sdfRoot, allResourceDirectories);

      return sdfRoot;
   }

   public static SDFRoot loadSDFRoot(InputStream inputStream, Collection<String> resourceDirectories, ClassLoader resourceClassLoader) throws JAXBException
   {
      JAXBContext context = JAXBContext.newInstance(SDFRoot.class);
      Unmarshaller um = context.createUnmarshaller();
      SDFRoot sdfRoot = (SDFRoot) um.unmarshal(inputStream);

      resolvePaths(sdfRoot, resourceDirectories, resourceClassLoader);

      return sdfRoot;
   }

   public static void resolvePaths(SDFRoot sdfRoot, Collection<String> resourceDirectories)
   {
      resolvePaths(sdfRoot, resourceDirectories, null);
   }

   public static void resolvePaths(SDFRoot sdfRoot, Collection<String> resourceDirectories, ClassLoader resourceClassLoader)
   {
      if (resourceClassLoader == null)
         resourceClassLoader = SDFTools.class.getClassLoader();

      List<SDFURIHolder> uriHolders = sdfRoot.getURIHolders();

      for (SDFURIHolder sdfURIHolder : uriHolders)
      {
         sdfURIHolder.setUri(tryToConvertToPath(sdfURIHolder.getUri(), resourceDirectories, resourceClassLoader));
      }
   }

   public static String tryToConvertToPath(String filename, Collection<String> resourceDirectories, ClassLoader resourceClassLoader)
   {
      try
      {
         URI uri = new URI(filename);

         String authority = uri.getAuthority() == null ? "" : uri.getAuthority();

         for (String resourceDirectory : resourceDirectories)
         {
            String fullname = resourceDirectory + authority + uri.getPath();
            // Path relative to class root
            if (resourceClassLoader.getResource(fullname) != null)
            {
               return fullname;
            }
            // Absolute path
            if (new File(fullname).exists())
            {
               return fullname;
            }
         }

         // Let's look in the parent directories of the resources if we can find a match to authority
         String resourceContainingAuthority = null;

         for (String resourceDirectory : resourceDirectories)
         {
            if (resourceDirectory.contains(authority))
            {
               resourceContainingAuthority = resourceDirectory;
               break;
            }
         }

         if (resourceContainingAuthority != null)
         {
            int lastIndexOf = resourceContainingAuthority.lastIndexOf(authority, resourceContainingAuthority.length());
            String newResource = resourceContainingAuthority.substring(0, lastIndexOf);

            if (!resourceDirectories.contains(newResource))
            {
               resourceDirectories.add(newResource);
               return tryToConvertToPath(filename, resourceDirectories, resourceClassLoader);
            }
         }
      }
      catch (URISyntaxException e)
      {
         System.err.println(SDFTools.class.getSimpleName() + ": Malformed resource path in SDF file for path: " + filename);
      }

      System.err.println(SDFTools.class.getSimpleName() + ": Unable to resolve the path: " + filename);

      return null;
   }

   public static RobotDefinition toFloatingRobotDefinition(SDFRoot sdfRoot, String modelName)
   {
      return toFloatingRobotDefinition(sdfRoot.getModels().stream().filter(model -> model.getName().equals(modelName)).findFirst().get());
   }

   public static RobotDefinition toFloatingRobotDefinition(SDFModel sdfModel)
   {
      return toRobotDefinition(new SixDoFJointDefinition(), sdfModel);
   }

   public static RobotDefinition toRobotDefinition(JointDefinition rootJointDefinition, SDFRoot sdfRoot, String modelName)
   {
      return toRobotDefinition(rootJointDefinition, sdfRoot.getModels().stream().filter(model -> model.getName().equals(modelName)).findFirst().get());
   }

   public static RobotDefinition toRobotDefinition(JointDefinition rootJointDefinition, SDFModel sdfModel)
   {
      List<SDFLink> sdfLinks = sdfModel.getLinks();
      List<SDFJoint> sdfJoints = sdfModel.getJoints();

      List<RigidBodyDefinition> rigidBodyDefinitions = sdfLinks.stream().map(SDFTools::toRigidBodyDefinition).collect(Collectors.toList());
      List<JointDefinition> jointDefinitions;
      if (sdfJoints == null)
         jointDefinitions = Collections.emptyList();
      else
         jointDefinitions = sdfJoints.stream().map(SDFTools::toJointDefinition).collect(Collectors.toList());
      RigidBodyDefinition startBodyDefinition = connectKinematics(rigidBodyDefinitions, jointDefinitions, sdfJoints, sdfLinks);
      if (rootJointDefinition.getName() == null)
         rootJointDefinition.setName(startBodyDefinition.getName());
      rootJointDefinition.setSuccessor(startBodyDefinition);
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");
      rootBodyDefinition.addChildJoint(rootJointDefinition);

      addSensors(sdfLinks, rigidBodyDefinitions);
      correctTransforms(sdfJoints, sdfLinks, jointDefinitions);

      RobotDefinition robotDefinition = new RobotDefinition(sdfModel.getName());
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      return robotDefinition;
   }

   public static RigidBodyDefinition connectKinematics(List<RigidBodyDefinition> rigidBodyDefinitions,
                                                       List<JointDefinition> jointDefinitions,
                                                       List<SDFJoint> sdfJoints,
                                                       List<SDFLink> sdfLinks)
   {
      if (sdfJoints == null)
         return rigidBodyDefinitions.get(0);

      Map<String, RigidBodyDefinition> rigidBodyDefinitionMap = rigidBodyDefinitions.stream().collect(Collectors.toMap(RigidBodyDefinition::getName,
                                                                                                                       Function.identity()));
      Map<String, JointDefinition> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDefinition::getName, Function.identity()));

      for (SDFJoint sdfJoint : sdfJoints)
      {
         String parent = sdfJoint.getParent();
         String child = sdfJoint.getChild();
         RigidBodyDefinition parentRigidBodyDefinition = rigidBodyDefinitionMap.get(parent);
         RigidBodyDefinition childRigidBodyDefinition = rigidBodyDefinitionMap.get(child);
         JointDefinition jointDefinition = jointDefinitionMap.get(sdfJoint.getName());

         jointDefinition.setSuccessor(childRigidBodyDefinition);
         parentRigidBodyDefinition.addChildJoint(jointDefinition);
      }

      RigidBodyDefinition rootBody = jointDefinitions.get(0).getPredecessor();

      while (rootBody.getParentJoint() != null)
         rootBody = rootBody.getParentJoint().getPredecessor();

      return rootBody;
   }

   public static void addSensors(List<SDFLink> sdfLinks, List<RigidBodyDefinition> rigidBodyDefinitions)
   {
      Map<String, RigidBodyDefinition> rigidBodyDefinitionMap = rigidBodyDefinitions.stream().collect(Collectors.toMap(RigidBodyDefinition::getName,
                                                                                                                       Function.identity()));

      for (SDFLink sdfLink : sdfLinks)
      {
         if (sdfLink.getSensors() == null)
            continue;

         RigidBodyDefinition rigidBodyDefinition = rigidBodyDefinitionMap.get(sdfLink.getName());
         JointDefinition parentJoint = rigidBodyDefinition.getParentJoint();

         for (SDFSensor sdfSensor : sdfLink.getSensors())
         {
            List<SensorDefinition> sensorDefinitions = toSensorDefinition(sdfSensor);
            if (sensorDefinitions != null)
               sensorDefinitions.forEach(parentJoint::addSensorDefinition);
         }
      }
   }

   public static void correctTransforms(List<SDFJoint> sdfJoints, List<SDFLink> sdfLinks, List<JointDefinition> jointDefinitions)
   {
      Map<String, SDFLink> sdfLinkMap = sdfLinks.stream().collect(Collectors.toMap(SDFLink::getName, Function.identity()));
      Map<String, JointDefinition> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDefinition::getName, Function.identity()));

      for (SDFJoint sdfJoint : sdfJoints)
      {
         String jointName = sdfJoint.getName();
         JointDefinition jointDefinition = jointDefinitionMap.get(jointName);
         RigidBodyDefinition childDefinition = jointDefinition.getSuccessor();

         String parentLinkName = sdfJoint.getParent();
         String childLinkName = sdfJoint.getChild();
         SDFLink parentSDFLink = sdfLinkMap.get(parentLinkName);
         SDFLink childSDFLink = sdfLinkMap.get(childLinkName);

         RigidBodyTransform parentLinkPose = parsePose(parentSDFLink.getPose());
         RigidBodyTransform childLinkPose = parsePose(childSDFLink.getPose());

         // Correct joint transform
         YawPitchRollTransformDefinition transformToParentJoint = jointDefinition.getTransformToParent();

         transformToParentJoint.setAndInvert(parentLinkPose);
         transformToParentJoint.multiply(childLinkPose);
         transformToParentJoint.getRotation().setToZero();
         parentLinkPose.transform(transformToParentJoint.getTranslation());

         // Correct link inertia pose
         YawPitchRollTransformDefinition inertiaPose = childDefinition.getInertiaPose();
         inertiaPose.prependOrientation(childLinkPose.getRotation());
         inertiaPose.transform(childDefinition.getMomentOfInertia());
         inertiaPose.getRotation().setToZero();

         // Correct visual transform
         for (VisualDefinition visualDescription : childDefinition.getVisualDefinitions())
         {
            AffineTransformDefinition visualPose = visualDescription.getOriginPose();
            visualPose.prependOrientation(childLinkPose.getRotation());
         }

         for (SensorDefinition sensorDefinition : jointDefinition.getSensorDefinitions())
         {
            sensorDefinition.getTransformToJoint().prependOrientation(childLinkPose.getRotation());
         }
      }
   }

   public static RigidBodyDefinition toRigidBodyDefinition(SDFLink sdfLink)
   {
      RigidBodyDefinition definition = new RigidBodyDefinition(sdfLink.getName());

      SDFInertial sdfInertial = sdfLink.getInertial();

      if (sdfInertial == null)
      {
         definition.setMass(parseDouble(null, DEFAULT_MASS));
         definition.getMomentOfInertia().set(parseMomentOfInertia(null));
         definition.getInertiaPose().set(parsePose(null));
      }
      else
      {
         definition.setMass(parseDouble(sdfInertial.getMass(), DEFAULT_MASS));
         definition.getMomentOfInertia().set(parseMomentOfInertia(sdfInertial.getInertia()));
         definition.getInertiaPose().set(parsePose(sdfInertial.getPose()));
      }

      if (sdfLink.getVisuals() != null)
         sdfLink.getVisuals().stream().map(SDFTools::toVisualDefinition).forEach(definition::addVisualDefinition);
      if (sdfLink.getCollisions() != null)
         sdfLink.getCollisions().stream().map(SDFTools::toCollisionShapeDefinition).forEach(definition::addCollisionShapeDefinition);

      return definition;
   }

   public static JointDefinition toJointDefinition(SDFJoint sdfJoint)
   {
      switch (sdfJoint.getType())
      {
         case "continuous":
            return toRevoluteJointDefinition(sdfJoint, true);
         case "revolute":
            return toRevoluteJointDefinition(sdfJoint, false);
         case "prismatic":
            return toPrismaticJointDefinition(sdfJoint);
         case "fixed":
            return toFixedJoint(sdfJoint);
         case "floating":
            return toSixDoFJointDefinition(sdfJoint);
         case "planar":
            return toPlanarJointDefinition(sdfJoint);
         default:
            throw new RuntimeException("Unexpected value for the joint type: " + sdfJoint.getType());
      }
   }

   public static RevoluteJointDefinition toRevoluteJointDefinition(SDFJoint sdfJoint, boolean ignorePositionLimits)
   {
      RevoluteJointDefinition definition = new RevoluteJointDefinition(sdfJoint.getName());

      definition.getTransformToParent().set(parsePose(sdfJoint.getPose()));
      definition.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), definition, ignorePositionLimits);
      parseDynamics(sdfJoint.getAxis().getDynamics(), definition);

      return definition;
   }

   public static PrismaticJointDefinition toPrismaticJointDefinition(SDFJoint sdfJoint)
   {
      PrismaticJointDefinition definition = new PrismaticJointDefinition(sdfJoint.getName());

      definition.getTransformToParent().set(parsePose(sdfJoint.getPose()));
      definition.getAxis().set(parseAxis(sdfJoint.getAxis()));
      parseLimit(sdfJoint.getAxis().getLimit(), definition, false);
      parseDynamics(sdfJoint.getAxis().getDynamics(), definition);

      return definition;
   }

   public static FixedJointDefinition toFixedJoint(SDFJoint sdfJoint)
   {
      FixedJointDefinition definition = new FixedJointDefinition(sdfJoint.getName());

      RigidBodyTransform parseRigidBodyTransform = parsePose(sdfJoint.getPose());
      definition.getTransformToParent().set(parseRigidBodyTransform);

      return definition;
   }

   public static SixDoFJointDefinition toSixDoFJointDefinition(SDFJoint sdfJoint)
   {
      SixDoFJointDefinition definition = new SixDoFJointDefinition(sdfJoint.getName());

      definition.getTransformToParent().set(parsePose(sdfJoint.getPose()));

      return definition;
   }

   public static PlanarJointDefinition toPlanarJointDefinition(SDFJoint sdfJoint)
   {
      PlanarJointDefinition definition = new PlanarJointDefinition(sdfJoint.getName());

      definition.getTransformToParent().set(parsePose(sdfJoint.getPose()));

      Vector3D surfaceNormal = parseAxis(sdfJoint.getAxis());

      if (!surfaceNormal.geometricallyEquals(Axis3D.Y, 1.0e-5))
         throw new UnsupportedOperationException("Planar joint are supported only with a surface normal equal to: "
               + EuclidCoreIOTools.getTuple3DString(Axis3D.Y) + ", received:" + surfaceNormal);

      return definition;
   }

   public static List<SensorDefinition> toSensorDefinition(SDFSensor sdfSensor)
   {
      List<SensorDefinition> definitions = new ArrayList<>();

      switch (sdfSensor.getType())
      {
         case "camera":
         case "multicamera":
         case "depth":
            definitions.addAll(toCameraSensorDefinition(sdfSensor.getCamera()));
            break;
         case "imu":
            definitions.add(toIMUSensorDefinition(sdfSensor.getImu()));
            break;
         case "gpu_ray":
         case "ray":
            definitions.add(toLidarSensorDefinition(sdfSensor.getRay()));
            break;
         default:
            LogTools.error("Unsupport sensor type: " + sdfSensor.getType());
            return null;
      }

      int updatePeriod = sdfSensor.getUpdateRate() == null ? -1 : (int) (1000.0 / parseDouble(sdfSensor.getUpdateRate(), 1000.0));

      for (SensorDefinition definition : definitions)
      {
         if (definition.getName() != null && !definition.getName().isEmpty())
            definition.setName(sdfSensor.getName() + "_" + definition.getName());
         else
            definition.setName(sdfSensor.getName());
         definition.getTransformToJoint().preMultiply(parsePose(sdfSensor.getPose()));
         definition.setUpdatePeriod(updatePeriod);
      }

      return definitions;
   }

   public static List<CameraSensorDefinition> toCameraSensorDefinition(List<SDFCamera> sdfCameras)
   {
      return sdfCameras.stream().map(SDFTools::toCameraSensorDefinition).collect(Collectors.toList());
   }

   public static CameraSensorDefinition toCameraSensorDefinition(SDFCamera sdfCamera)
   {
      CameraSensorDefinition definition = new CameraSensorDefinition();
      definition.setName(sdfCamera.getName());
      definition.getTransformToJoint().set(parsePose(sdfCamera.getPose()));
      definition.setFieldOfView(parseDouble(sdfCamera.getHorizontalFov(), Double.NaN));
      definition.setClipNear(parseDouble(sdfCamera.getClip().getNear(), Double.NaN));
      definition.setClipFar(parseDouble(sdfCamera.getClip().getFar(), Double.NaN));
      definition.setImageWidth(parseInteger(sdfCamera.getImage().getWidth(), -1));
      definition.setImageHeight(parseInteger(sdfCamera.getImage().getHeight(), -1));
      return definition;
   }

   public static LidarSensorDefinition toLidarSensorDefinition(SDFRay sdfRay)
   {
      LidarSensorDefinition definition = new LidarSensorDefinition();

      SDFRange sdfRange = sdfRay.getRange();
      double sdfRangeMax = parseDouble(sdfRange.getMax(), Double.NaN);
      double sdfRangeMin = parseDouble(sdfRange.getMin(), Double.NaN);
      double sdfRangeResolution = parseDouble(sdfRange.getResolution(), Double.NaN);

      SDFHorizontalScan sdfHorizontalScan = sdfRay.getScan().getHorizontal();
      SDFVerticalScan sdfVerticalScan = sdfRay.getScan().getVertical();
      double maxSweepAngle = parseDouble(sdfHorizontalScan.getMaxAngle(), 0.0);
      double minSweepAngle = parseDouble(sdfHorizontalScan.getMinAngle(), 0.0);
      double maxHeightAngle = sdfVerticalScan == null ? 0.0 : parseDouble(sdfVerticalScan.getMaxAngle(), 0.0);
      double minHeightAngle = sdfVerticalScan == null ? 0.0 : parseDouble(sdfVerticalScan.getMinAngle(), 0.0);

      int samples = parseInteger(sdfHorizontalScan.getSamples(), -1) / 3 * 3;
      int scanHeight = sdfVerticalScan == null ? 1 : parseInteger(sdfVerticalScan.getSamples(), 1);

      SDFNoise sdfNoise = sdfRay.getNoise();
      if (sdfNoise != null)
      {
         if ("gaussian".equals(sdfNoise.getType()))
         {
            definition.setGaussianNoiseMean(parseDouble(sdfNoise.getMean(), 0.0));
            definition.setGaussianNoiseStandardDeviation(parseDouble(sdfNoise.getStddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown noise model: {}.", sdfNoise.getType());
         }
      }

      definition.getTransformToJoint().set(parsePose(sdfRay.getPose()));
      definition.setPointsPerSweep(samples);
      definition.setSweepYawLimits(minSweepAngle, maxSweepAngle);
      definition.setHeightPitchLimits(minHeightAngle, maxHeightAngle);
      definition.setRangeLimits(sdfRangeMin, sdfRangeMax);
      definition.setRangeResolution(sdfRangeResolution);
      definition.setScanHeight(scanHeight);
      return definition;
   }

   public static IMUSensorDefinition toIMUSensorDefinition(SDFIMU sdfIMU)
   {
      IMUSensorDefinition definition = new IMUSensorDefinition();

      SDFIMUNoise sdfNoise = sdfIMU.getNoise();
      if (sdfNoise != null)
      {
         if ("gaussian".equals(sdfNoise.getType()))
         {
            SDFNoiseParameters accelerationNoise = sdfNoise.getAccel();
            SDFNoiseParameters angularVelocityNoise = sdfNoise.getRate();

            definition.setAccelerationNoiseParameters(parseDouble(accelerationNoise.getMean(), 0.0), parseDouble(accelerationNoise.getStddev(), 0.0));
            definition.setAccelerationBiasParameters(parseDouble(accelerationNoise.getBias_mean(), 0.0), parseDouble(accelerationNoise.getBias_stddev(), 0.0));

            definition.setAngularVelocityNoiseParameters(parseDouble(angularVelocityNoise.getMean(), 0.0), parseDouble(angularVelocityNoise.getStddev(), 0.0));
            definition.setAngularVelocityBiasParameters(parseDouble(angularVelocityNoise.getBias_mean(), 0.0),
                                                        parseDouble(angularVelocityNoise.getBias_stddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown IMU noise model: {}.", sdfNoise.getType());
         }
      }

      return definition;
   }

   public static VisualDefinition toVisualDefinition(SDFVisual sdfVisual)
   {
      if (sdfVisual == null)
         return null;

      VisualDefinition visualDefinition = new VisualDefinition();
      visualDefinition.setName(sdfVisual.getName());
      visualDefinition.setOriginPose(parsePose(sdfVisual.getPose()));
      visualDefinition.setMaterialDefinition(toMaterialDefinition(sdfVisual.getMaterial()));
      visualDefinition.setGeometryDefinition(toGeometryDefinition(sdfVisual.getGeometry()));
      return visualDefinition;
   }

   public static CollisionShapeDefinition toCollisionShapeDefinition(SDFCollision sdfCollision)
   {
      if (sdfCollision == null)
         return null;

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      collisionShapeDefinition.setName(sdfCollision.getName());
      collisionShapeDefinition.setOriginPose(parsePose(sdfCollision.getPose()));
      collisionShapeDefinition.setGeometryDefinition(toGeometryDefinition(sdfCollision.getGeometry()));
      return collisionShapeDefinition;
   }

   public static GeometryDefinition toGeometryDefinition(SDFGeometry sdfGeometry)
   {
      return toGeometryDefinition(sdfGeometry, Collections.emptyList());
   }

   public static GeometryDefinition toGeometryDefinition(SDFGeometry sdfGeometry, List<String> resourceDirectories)
   {
      if (sdfGeometry.getBox() != null)
      {
         Box3DDefinition boxGeometryDefinition = new Box3DDefinition();
         boxGeometryDefinition.setSize(parseVector3D(sdfGeometry.getBox().getSize(), null));
         return boxGeometryDefinition;
      }
      if (sdfGeometry.getCylinder() != null)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = new Cylinder3DDefinition();
         cylinderGeometryDefinition.setRadius(parseDouble(sdfGeometry.getCylinder().getRadius(), 0.0));
         cylinderGeometryDefinition.setLength(parseDouble(sdfGeometry.getCylinder().getLength(), 0.0));
         return cylinderGeometryDefinition;
      }
      if (sdfGeometry.getSphere() != null)
      {
         Sphere3DDefinition sphereGeometryDefinition = new Sphere3DDefinition();
         sphereGeometryDefinition.setRadius(parseDouble(sdfGeometry.getSphere().getRadius(), 0.0));
         return sphereGeometryDefinition;
      }
      if (sdfGeometry.getMesh() != null)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition();
         modelFileGeometryDefinition.setResourceDirectories(resourceDirectories);
         modelFileGeometryDefinition.setFileName(sdfGeometry.getMesh().getUri());
         modelFileGeometryDefinition.setScale(parseVector3D(sdfGeometry.getMesh().getScale(), new Vector3D(1, 1, 1)));
         return modelFileGeometryDefinition;
      }
      throw new IllegalArgumentException("The given SDF Geometry is empty.");
   }

   public static MaterialDefinition toMaterialDefinition(SDFMaterial sdfMaterial)
   {
      if (sdfMaterial == null)
         return null;

      MaterialDefinition materialDefinition = new MaterialDefinition();
      materialDefinition.setShininess(parseDouble(sdfMaterial.getLighting(), Double.NaN));
      materialDefinition.setAmbientColor(toColorDefinition(sdfMaterial.getAmbient()));
      materialDefinition.setDiffuseColor(toColorDefinition(sdfMaterial.getDiffuse()));
      materialDefinition.setSpecularColor(toColorDefinition(sdfMaterial.getSpecular()));
      materialDefinition.setEmissiveColor(toColorDefinition(sdfMaterial.getEmissive()));
      // TODO handle the script
      return materialDefinition;
   }

   public static ColorDefinition toColorDefinition(String sdfColor)
   {
      if (sdfColor == null)
         return null;

      double[] colorArray = parseArray(sdfColor, null);
      if (colorArray == null)
         return null;
      else if (colorArray.length < 4)
         return ColorDefinitions.rgb(colorArray);
      else
         return ColorDefinitions.rgba(colorArray);
   }

   public static RigidBodyTransform parsePose(String pose)
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();

      if (pose != null)
      {
         String[] split = pose.split("\\s+");
         Vector3D position = new Vector3D(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]));
         YawPitchRoll orientation = new YawPitchRoll(Double.parseDouble(split[5]), Double.parseDouble(split[4]), Double.parseDouble(split[3]));
         rigidBodyTransform.set(orientation, position);
      }
      return rigidBodyTransform;
   }

   public static Matrix3D parseMomentOfInertia(SDFInertia inertia)
   {
      if (inertia == null)
         inertia = new SDFInertia();

      Matrix3D momentOfInertia = new Matrix3D();

      double ixx = parseDouble(inertia.getIxx(), DEFAULT_IXX);
      double iyy = parseDouble(inertia.getIyy(), DEFAULT_IYY);
      double izz = parseDouble(inertia.getIzz(), DEFAULT_IZZ);

      double ixy = parseDouble(inertia.getIxy(), DEFAULT_IXY);
      double ixz = parseDouble(inertia.getIxz(), DEFAULT_IXZ);
      double iyz = parseDouble(inertia.getIyz(), DEFAULT_IYZ);

      momentOfInertia.setM00(ixx);
      momentOfInertia.setM11(iyy);
      momentOfInertia.setM22(izz);

      momentOfInertia.setM01(ixy);
      momentOfInertia.setM02(ixz);
      momentOfInertia.setM12(iyz);

      momentOfInertia.setM10(ixy);
      momentOfInertia.setM20(ixz);
      momentOfInertia.setM21(iyz);

      return momentOfInertia;
   }

   public static void parseLimit(SDFLimit sdfLimit, OneDoFJointDefinition jointDefinitionToParseLimitInto, boolean ignorePositionLimits)
   {
      jointDefinitionToParseLimitInto.setPositionLimits(DEFAULT_LOWER_LIMIT, DEFAULT_UPPER_LIMIT);
      jointDefinitionToParseLimitInto.setEffortLimits(DEFAULT_EFFORT_LIMIT);
      jointDefinitionToParseLimitInto.setVelocityLimits(DEFAULT_VELOCITY_LIMIT);

      if (sdfLimit != null)
      {
         if (!ignorePositionLimits)
         {
            double positionLowerLimit = parseDouble(sdfLimit.getLower(), DEFAULT_LOWER_LIMIT);
            double positionUpperLimit = parseDouble(sdfLimit.getUpper(), DEFAULT_UPPER_LIMIT);
            if (positionLowerLimit < positionUpperLimit)
               jointDefinitionToParseLimitInto.setPositionLimits(positionLowerLimit, positionUpperLimit);
         }
         double effortLimit = parseDouble(sdfLimit.getEffort(), DEFAULT_EFFORT_LIMIT);
         if (Double.isFinite(effortLimit) && effortLimit >= 0)
            jointDefinitionToParseLimitInto.setEffortLimits(effortLimit);
         double velocityLimit = parseDouble(sdfLimit.getVelocity(), DEFAULT_VELOCITY_LIMIT);
         if (Double.isFinite(velocityLimit) && velocityLimit >= 0)
            jointDefinitionToParseLimitInto.setVelocityLimits(velocityLimit);
      }
   }

   public static void parseDynamics(SDFDynamics sdfDynamics, OneDoFJointDefinition jointDefinitionToParseDynamicsInto)
   {
      double damping = 0.0;
      double stiction = 0.0;

      if (sdfDynamics != null)
      {
         damping = parseDouble(sdfDynamics.getDamping(), 0.0);
         stiction = parseDouble(sdfDynamics.getFriction(), 0.0);
      }

      jointDefinitionToParseDynamicsInto.setDamping(damping);
      jointDefinitionToParseDynamicsInto.setStiction(stiction);
   }

   public static Vector3D parseAxis(SDFAxis axis)
   {
      if (axis == null)
         return new Vector3D(DEFAULT_AXIS);
      Vector3D parsedAxis = parseVector3D(axis.getXYZ(), new Vector3D(DEFAULT_AXIS));
      parsedAxis.normalize();
      return parsedAxis;
   }

   public static double parseDouble(String value, double defaultValue)
   {
      if (value == null)
         return defaultValue;
      return Double.parseDouble(value);
   }

   public static int parseInteger(String value, int defaultValue)
   {
      if (value == null)
         return defaultValue;
      return Integer.parseInt(value);
   }

   public static Vector3D parseVector3D(String value, Vector3D defaultValue)
   {
      if (value == null)
         return defaultValue;

      String[] split = value.split("\\s+");
      Vector3D vector = new Vector3D();
      vector.setX(Double.parseDouble(split[0]));
      vector.setY(Double.parseDouble(split[1]));
      vector.setZ(Double.parseDouble(split[2]));
      return vector;
   }

   public static double[] parseArray(String value, double[] defaultValue)
   {
      if (value == null)
         return defaultValue;

      String[] split = value.split("\\s+");
      double[] array = new double[split.length];

      for (int i = 0; i < split.length; i++)
         array[i] = Double.parseDouble(split[i]);

      return array;
   }
}
