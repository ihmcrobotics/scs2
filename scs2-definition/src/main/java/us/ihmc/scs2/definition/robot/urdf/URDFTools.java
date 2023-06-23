package us.ihmc.scs2.definition.robot.urdf;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParserFactory;
import javax.xml.transform.sax.SAXSource;

import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.XMLReader;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
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
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PlanarJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFAxis;
import us.ihmc.scs2.definition.robot.urdf.items.URDFBox;
import us.ihmc.scs2.definition.robot.urdf.items.URDFCollision;
import us.ihmc.scs2.definition.robot.urdf.items.URDFColor;
import us.ihmc.scs2.definition.robot.urdf.items.URDFCylinder;
import us.ihmc.scs2.definition.robot.urdf.items.URDFDynamics;
import us.ihmc.scs2.definition.robot.urdf.items.URDFFilenameHolder;
import us.ihmc.scs2.definition.robot.urdf.items.URDFGazebo;
import us.ihmc.scs2.definition.robot.urdf.items.URDFGeometry;
import us.ihmc.scs2.definition.robot.urdf.items.URDFInertia;
import us.ihmc.scs2.definition.robot.urdf.items.URDFInertial;
import us.ihmc.scs2.definition.robot.urdf.items.URDFItem;
import us.ihmc.scs2.definition.robot.urdf.items.URDFJoint;
import us.ihmc.scs2.definition.robot.urdf.items.URDFJoint.URDFJointType;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLimit;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLink;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLinkReference;
import us.ihmc.scs2.definition.robot.urdf.items.URDFMass;
import us.ihmc.scs2.definition.robot.urdf.items.URDFMaterial;
import us.ihmc.scs2.definition.robot.urdf.items.URDFMesh;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.robot.urdf.items.URDFOrigin;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFCamera;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFCamera.URDFClip;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFCamera.URDFSensorImage;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFIMU;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFIMU.URDFIMUNoise;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFIMU.URDFIMUNoise.URDFIMUNoiseType;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFIMU.URDFIMUNoise.URDFNoiseParameters;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFNoise;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFNoise.URDFNoiseType;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFRange;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFScan;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFScan.URDFHorizontalScan;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFScan.URDFVerticalScan;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFSensorType;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSphere;
import us.ihmc.scs2.definition.robot.urdf.items.URDFTexture;
import us.ihmc.scs2.definition.robot.urdf.items.URDFVisual;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

/**
 * This class gathers tools for parsing a URDF file and for converting a parsed {@code URDFModel}
 * into a {@link RobotDefinition}.
 * <p>
 * {@link RobotDefinition} is a template for creating a robot to be simulated in SCS2.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class URDFTools
{
   private static final Vector3D DEFAULT_ORIGIN_XYZ = new Vector3D();
   private static final Vector3D DEFAULT_ORIGIN_RPY = new Vector3D();

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

   public static final URDFFormatter DEFAULT_URDF_FORMATTER = new URDFFormatter();

   /**
    * Parse a {@link URDFModel} from the given URDF file.
    * 
    * @param urdfFile the URDF file to be loaded.
    * @return the model.
    * @throws JAXBException
    */
   public static URDFModel loadURDFModel(File urdfFile) throws JAXBException
   {
      return loadURDFModel(urdfFile, Collections.emptyList());
   }

   /**
    * Parse a {@link URDFModel} from the given URDF file.
    * 
    * @param urdfFile            the URDF file to be loaded.
    * @param resourceDirectories paths to resource directories. This allows to search for resources
    *                            that are not in the same directory as the {@code urdfFile}'s parent
    *                            directory. Paths can either be relative to the {@code urdfFile}'s
    *                            parent directory or absolute.
    * @return the model.
    * @throws JAXBException
    */
   public static URDFModel loadURDFModel(File urdfFile, Collection<String> resourceDirectories) throws JAXBException
   {
      Set<String> allResourceDirectories = new HashSet<>(resourceDirectories);
      File parentFile = urdfFile.getParentFile();

      if (parentFile != null)
      {
         allResourceDirectories.add(parentFile.getAbsolutePath() + File.separator);
         Stream.of(parentFile.listFiles(File::isDirectory)).map(file -> file.getAbsolutePath() + File.separator).forEach(allResourceDirectories::add);
      }

      JAXBContext context = JAXBContext.newInstance(URDFModel.class);
      Unmarshaller um = context.createUnmarshaller();
      URDFModel urdfModel = (URDFModel) um.unmarshal(urdfFile);

      resolvePaths(urdfModel, allResourceDirectories);

      return urdfModel;
   }

   /**
    * Parse a {@link URDFModel} from the given input stream.
    * 
    * @param inputStream         the stream to be loaded.
    * @param resourceDirectories paths to resource directories. This allows to search for resources
    *                            that are defined outside the {@code inputStream}.
    * @param resourceClassLoader the class loader is used to retrieve the resources. If the resources
    *                            are located in the class path, e.g. in the <tt>resources</tt> folder,
    *                            simply use {@code CallerClass.getClassLoader()}. If the resources are
    *                            located outside the scope of the class path, see
    *                            {@link URLClassLoader} that allows to point to a directory among other
    *                            options.
    * @return the model.
    * @throws JAXBException
    */
   public static URDFModel loadURDFModel(InputStream inputStream, Collection<String> resourceDirectories, ClassLoader resourceClassLoader) throws JAXBException
   {
      return loadURDFModel(inputStream, resourceDirectories, resourceClassLoader, false);
   }

   /**
    * Parse a {@link URDFModel} from the given input stream.
    * 
    * @param inputStream         the stream to be loaded.
    * @param resourceDirectories paths to resource directories. This allows to search for resources
    *                            that are defined outside the {@code inputStream}.
    * @param resourceClassLoader the class loader is used to retrieve the resources. If the resources
    *                            are located in the class path, e.g. in the <tt>resources</tt> folder,
    *                            simply use {@code CallerClass.getClassLoader()}. If the resources are
    *                            located outside the scope of the class path, see
    *                            {@link URLClassLoader} that allows to point to a directory among other
    *                            options.
    * @param ignoreNamespace     whether XML namespaces should be ignored. Note that should always be
    *                            false, it is only to handle malformed XML file that do not properly
    *                            declare their namespaces.
    * @return the model.
    * @throws JAXBException
    */
   public static URDFModel loadURDFModel(InputStream inputStream,
                                         Collection<String> resourceDirectories,
                                         ClassLoader resourceClassLoader,
                                         boolean ignoreNamespace)
         throws JAXBException
   {
      try
      {
         Set<String> allResourceDirectories = new HashSet<>(resourceDirectories);
         URDFModel urdfModel;
         JAXBContext context = JAXBContext.newInstance(URDFModel.class);
         Unmarshaller um = context.createUnmarshaller();

         if (!ignoreNamespace)
         {
            urdfModel = (URDFModel) um.unmarshal(inputStream);
         }
         else
         {
            InputSource is = new InputSource(inputStream);
            SAXParserFactory sax = SAXParserFactory.newInstance();
            sax.setNamespaceAware(false);
            XMLReader reader;

            try
            {
               reader = sax.newSAXParser().getXMLReader();
            }
            catch (SAXException | ParserConfigurationException e)
            {
               throw new JAXBException(e);
            }

            SAXSource source = new SAXSource(reader, is);
            urdfModel = (URDFModel) um.unmarshal(source);
         }
         resolvePaths(urdfModel, allResourceDirectories, resourceClassLoader);
         return urdfModel;
      }
      finally
      {
         try
         {
            inputStream.close();
         }
         catch (IOException e)
         {
            LogTools.error(e.getMessage());
         }
      }
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Tests and resolve all filename referenced in the {@code urdfModel} such that they can later be
    * easily loaded.
    * </p>
    * <p>
    * This method attempt to locate every single file reference in the URDF model using the given
    * {@code resourceDirectories}.
    * </p>
    * 
    * @param urdfModel           the model to resolve internal references to files.
    * @param resourceDirectories the paths the resources could be located in.
    * @see SDFTools#tryToConvertToPath(String, Collection, ClassLoader)
    */
   public static void resolvePaths(URDFModel urdfModel, Collection<String> resourceDirectories)
   {
      resolvePaths(urdfModel, resourceDirectories, null);
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Tests and resolve all filename referenced in the {@code urdfModel} such that they can later be
    * easily loaded.
    * </p>
    * <p>
    * This method attempt to locate every single file reference in the URDF model using the given
    * {@code resourceDirectories}.
    * </p>
    * 
    * @param urdfModel           the model to resolve internal references to files.
    * @param resourceDirectories the paths the resources could be located in.
    * @param resourceClassLoader the class loader is used to retrieve the resources. If the resources
    *                            are located in the class path, e.g. in the <tt>resources</tt> folder,
    *                            simply use {@code CallerClass.getClassLoader()}. If the resources are
    *                            located outside the scope of the class path, see
    *                            {@link URLClassLoader} that allows to point to a directory among other
    *                            options.
    * @see SDFTools#tryToConvertToPath(String, Collection, ClassLoader)
    */
   public static void resolvePaths(URDFModel urdfModel, Collection<String> resourceDirectories, ClassLoader resourceClassLoader)
   {
      if (resourceClassLoader == null)
         resourceClassLoader = URDFTools.class.getClassLoader();

      List<URDFFilenameHolder> filenameHolders = urdfModel.getFilenameHolders();

      for (URDFFilenameHolder urdfFilenameHolder : filenameHolders)
      {
         urdfFilenameHolder.setFilename(tryToConvertToPath(urdfFilenameHolder.getFilename(), resourceDirectories, resourceClassLoader));
      }
   }

   /**
    * Redirection to {@link SDFTools#tryToConvertToPath(String, Collection, ClassLoader)}.
    */
   public static String tryToConvertToPath(String filename, Collection<String> resourceDirectories, ClassLoader resourceClassLoader)
   {
      return SDFTools.tryToConvertToPath(filename, resourceDirectories, resourceClassLoader);
   }

   /**
    * Converts the given URDF model into a {@code RobotDefinition} which root joint is a 6-DoF floating
    * joint.
    * <p>
    * Note that the root joint type is not explicitly defined in the URDF model and thus has to be
    * specified here.
    * </p>
    * 
    * @param urdfModel the URDF model to convert.
    * @return the robot definition which can be used to create a robot to be simulated in SCS2.
    */
   public static RobotDefinition toFloatingRobotDefinition(URDFModel urdfModel)
   {
      return toFloatingRobotDefinition(urdfModel, null);
   }

   /**
    * Converts the given URDF model into a {@code RobotDefinition} which root joint is a 6-DoF floating
    * joint.
    * <p>
    * Note that the root joint type is not explicitly defined in the URDF model and thus has to be
    * specified here.
    * </p>
    * 
    * @param urdfModel the URDF model to convert.
    * @return the robot definition which can be used to create a robot to be simulated in SCS2.
    */
   public static RobotDefinition toFloatingRobotDefinition(URDFModel urdfModel, Predicate<FixedJointDefinition> simplifyKinematicsFilter)
   {
      return toRobotDefinition(new SixDoFJointDefinition(), urdfModel);
   }

   /**
    * Converts the given URDF model into a {@code RobotDefinition} while specifying the root joint
    * type.
    * <p>
    * Note that the root joint type is not explicitly defined in the URDF model and thus has to be
    * specified here.
    * </p>
    * 
    * @param rootJointDefinition      the definition to use for the robot's root joint.
    * @param urdfModel                the URDF model to convert.
    * @param simplifyKinematicsFilter filter for controlling the robot kinematics simplification.
    * @return the robot definition which can be used to create a robot to be simulated in SCS2.
    * @see RobotDefinition#simplifyKinematics(JointDefinition, Predicate)
    */
   public static RobotDefinition toRobotDefinition(JointDefinition rootJointDefinition, URDFModel urdfModel)
   {
      return toRobotDefinition(rootJointDefinition, urdfModel, null);
   }

   /**
    * Converts the given URDF model into a {@code RobotDefinition} while specifying the root joint
    * type.
    * <p>
    * Note that the root joint type is not explicitly defined in the URDF model and thus has to be
    * specified here.
    * </p>
    * 
    * @param rootJointDefinition      the definition to use for the robot's root joint.
    * @param urdfModel                the URDF model to convert.
    * @param simplifyKinematicsFilter filter for controlling the robot kinematics simplification.
    * @return the robot definition which can be used to create a robot to be simulated in SCS2.
    * @see RobotDefinition#simplifyKinematics(JointDefinition, Predicate)
    */
   public static RobotDefinition toRobotDefinition(JointDefinition rootJointDefinition,
                                                   URDFModel urdfModel,
                                                   Predicate<FixedJointDefinition> simplifyKinematicsFilter)
   {
      List<URDFLink> urdfLinks = urdfModel.getLinks();
      List<URDFJoint> urdfJoints = urdfModel.getJoints();
      List<URDFGazebo> urdfGazebos = urdfModel.getGazebos();

      List<RigidBodyDefinition> rigidBodyDefinitions = urdfLinks.stream().map(URDFTools::toRigidBodyDefinition).collect(Collectors.toList());
      List<JointDefinition> jointDefinitions;
      if (urdfJoints == null)
         jointDefinitions = Collections.emptyList();
      else
         jointDefinitions = urdfJoints.stream().map(URDFTools::toJointDefinition).collect(Collectors.toList());
      RigidBodyDefinition startBodyDefinition = connectKinematics(rigidBodyDefinitions, jointDefinitions, urdfJoints);
      if (rootJointDefinition.getName() == null)
         rootJointDefinition.setName(startBodyDefinition.getName());
      rootJointDefinition.setSuccessor(startBodyDefinition);
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      jointDefinitions.add(rootJointDefinition); // This is required for sensors that are attached to the successor of the root joint.
      addSensors(urdfGazebos, jointDefinitions);

      RobotDefinition robotDefinition = new RobotDefinition(urdfModel.getName());
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      return robotDefinition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Searches the URDF Gazebo references for sensors, for instance IMUs and cameras, to convert into
    * {@link SensorDefinition} and add to the corresponding joints.
    * </p>
    * 
    * @param urdfGazebos      the list of parsed Gazebo references from a URDF file.
    * @param jointDefinitions the list of parsed and converted joints from the same URDF file.
    */
   public static void addSensors(List<URDFGazebo> urdfGazebos, List<JointDefinition> jointDefinitions)
   {
      if (urdfGazebos == null || urdfGazebos.isEmpty())
         return;

      Map<String, JointDefinition> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDefinition::getName, Function.identity()));
      Map<String, JointDefinition> linkNameToJointDefinitionMap = jointDefinitions.stream()
                                                                                  .collect(Collectors.toMap(joint -> joint.getSuccessor().getName(),
                                                                                                            Function.identity()));

      for (URDFGazebo urdfGazebo : urdfGazebos)
      {
         if (urdfGazebo.getSensor() == null)
            continue;

         List<SensorDefinition> sensorDefinitions = toSensorDefinition(urdfGazebo.getSensor());
         JointDefinition jointDefinition = jointDefinitionMap.get(urdfGazebo.getReference());
         if (jointDefinition == null)
            jointDefinition = linkNameToJointDefinitionMap.get(urdfGazebo.getReference());

         if (jointDefinition == null)
         {
            LogTools.error("Could not find reference: " + urdfGazebo.getReference());
            continue;
         }

         if (sensorDefinitions != null)
            sensorDefinitions.forEach(jointDefinition::addSensorDefinition);
      }
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Connects the kinematics of rigid-bodies and joints that were just converted.
    * </p>
    * <p>
    * This method essentially retrieves the parents ({@link RigidBodyDefinition#getParentJoint()}),
    * children {@link RigidBodyDefinition#getChildrenJoints()}, predecessors
    * ({@link JointDefinition#getPredecessor()}), and successors
    * ({@link JointDefinition#getSuccessor()}).
    * </p>
    * 
    * @param rigidBodyDefinitions the rigid-bodies to retrieve the parent and children joints for.
    * @param jointDefinitions     the joints to retrieve the predecessors and successors for.
    * @param urdfJoints           the parsed URDF joints used to identify relationship between joints
    *                             and rigid-bodies.
    * @return the root body, i.e. the only rigid-body without a parent joint.
    */
   public static RigidBodyDefinition connectKinematics(List<RigidBodyDefinition> rigidBodyDefinitions,
                                                       List<JointDefinition> jointDefinitions,
                                                       List<URDFJoint> urdfJoints)
   {
      Map<String, RigidBodyDefinition> rigidBodyDefinitionMap = rigidBodyDefinitions.stream()
                                                                                    .collect(Collectors.toMap(RigidBodyDefinition::getName,
                                                                                                              Function.identity()));
      Map<String, JointDefinition> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDefinition::getName, Function.identity()));

      if (urdfJoints != null)
      {
         for (URDFJoint urdfJoint : urdfJoints)
         {
            URDFLinkReference parent = urdfJoint.getParent();
            URDFLinkReference child = urdfJoint.getChild();
            RigidBodyDefinition parentRigidBodyDefinition = rigidBodyDefinitionMap.get(parent.getLink());
            Objects.requireNonNull(parentRigidBodyDefinition,
                                   "Could not find parent rigid-body (%s) for joint (%s)".formatted(parent.getLink(), urdfJoint.getName()));
            RigidBodyDefinition childRigidBodyDefinition = rigidBodyDefinitionMap.get(child.getLink());
            Objects.requireNonNull(parentRigidBodyDefinition,
                                   "Could not find child rigid-body (%s) for joint (%s)".formatted(child.getLink(), urdfJoint.getName()));
            JointDefinition jointDefinition = jointDefinitionMap.get(urdfJoint.getName());

            jointDefinition.setSuccessor(childRigidBodyDefinition);
            parentRigidBodyDefinition.addChildJoint(jointDefinition);
         }
      }

      if (urdfJoints == null)
      {
         return rigidBodyDefinitions.get(0);
      }
      else
      {
         Map<String, URDFJoint> childToParentJoint = urdfJoints.stream()
                                                               .collect(Collectors.toMap(urdfJoint -> urdfJoint.getChild().getLink(), Function.identity()));

         String rootBodyName = urdfJoints.iterator().next().getParent().getLink();
         URDFJoint parentJoint = childToParentJoint.get(rootBodyName);

         while (parentJoint != null)
         {
            rootBodyName = parentJoint.getParent().getLink();
            parentJoint = childToParentJoint.get(rootBodyName);
         }

         return rigidBodyDefinitionMap.get(rootBodyName);
      }
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF link into a {@link RigidBodyDefinition}.
    * </p>
    * <p>
    * The parent and children joints are not configured at this stage.
    * </p>
    * 
    * @param urdfLink the parsed URDF link to convert.
    * @return the rigid-body definition.
    * @see #connectKinematics(List, List, List)
    */
   public static RigidBodyDefinition toRigidBodyDefinition(URDFLink urdfLink)
   {
      RigidBodyDefinition definition = new RigidBodyDefinition(urdfLink.getName());

      URDFInertial urdfInertial = urdfLink.getInertial();

      if (urdfInertial == null)
      {
         definition.setMass(parseMass(null));
         definition.getMomentOfInertia().set(parseMomentOfInertia(null));
         definition.getInertiaPose().set(parseRigidBodyTransform(null));
      }
      else
      {
         definition.setMass(parseMass(urdfInertial.getMass()));
         definition.getMomentOfInertia().set(parseMomentOfInertia(urdfInertial.getInertia()));
         definition.getInertiaPose().set(parseRigidBodyTransform(urdfInertial.getOrigin()));
      }

      if (urdfLink.getVisual() != null)
      {
         List<URDFVisual> urdfVisuals = urdfLink.getVisual();

         for (int i = 0; i < urdfVisuals.size(); i++)
         {
            URDFVisual urdfVisual = urdfVisuals.get(i);
            VisualDefinition visual = toVisualDefinition(urdfVisual);
            if (visual == null)
               continue;
            if (visual.getName() == null)
            {
               if (i == 0)
                  visual.setName(urdfLink.getName() + "_visual");
               else // This seems to be the Gazebo SDF converter default naming convention
                  visual.setName(urdfLink.getName() + "_visual_" + urdfLink.getName() + "_" + i);
            }
            definition.addVisualDefinition(visual);
         }
      }

      if (urdfLink.getCollision() != null)
      {
         List<URDFCollision> urdfCollisions = urdfLink.getCollision();

         for (int i = 0; i < urdfCollisions.size(); i++)
         {
            URDFCollision urdfCollision = urdfCollisions.get(i);
            CollisionShapeDefinition collision = toCollisionShapeDefinition(urdfCollision);
            if (collision == null)
               continue;
            if (collision.getName() == null)
            {
               if (i == 0)
                  collision.setName(urdfLink.getName() + "_collision");
               else // This seems to be the Gazebo SDF converter default naming convention
                  collision.setName(urdfLink.getName() + "_collision_" + urdfLink.getName() + "_" + i);
            }
            definition.addCollisionShapeDefinition(collision);
         }
      }

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF joint into a {@link JointDefinition}.
    * </p>
    * <p>
    * The predecessor, successor, and sensors are not configured at this stage.
    * </p>
    * 
    * @param urdfJoint the parsed URDF joint to convert.
    * @return the joint definition.
    * @see #addSensors(List, List)
    * @see #connectKinematics(List, List, List)
    */
   public static JointDefinition toJointDefinition(URDFJoint urdfJoint)
   {
      switch (URDFJointType.parse(urdfJoint.getType()))
      {
         case continuous:
            return toRevoluteJointDefinition(urdfJoint, true);
         case revolute:
            return toRevoluteJointDefinition(urdfJoint, false);
         case prismatic:
            return toPrismaticJointDefinition(urdfJoint);
         case fixed:
            return toFixedJointDefinition(urdfJoint);
         case floating:
            return toSixDoFJointDefinition(urdfJoint);
         case planar:
            return toPlanarJointDefinition(urdfJoint);
         default:
            throw new RuntimeException("Unexpected value for the joint type: " + urdfJoint.getType());
      }
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toJointDefinition(URDFJoint)
    */
   public static RevoluteJointDefinition toRevoluteJointDefinition(URDFJoint urdfJoint, boolean ignorePositionLimits)
   {
      RevoluteJointDefinition definition = new RevoluteJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis()));
      parseLimit(urdfJoint.getLimit(), definition, ignorePositionLimits);
      parseDynamics(urdfJoint.getDynamics(), definition);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toJointDefinition(URDFJoint)
    */
   public static PrismaticJointDefinition toPrismaticJointDefinition(URDFJoint urdfJoint)
   {
      PrismaticJointDefinition definition = new PrismaticJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin()));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis()));
      parseLimit(urdfJoint.getLimit(), definition, false);
      parseDynamics(urdfJoint.getDynamics(), definition);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toJointDefinition(URDFJoint)
    */
   public static FixedJointDefinition toFixedJointDefinition(URDFJoint urdfJoint)
   {
      FixedJointDefinition definition = new FixedJointDefinition(urdfJoint.getName());

      RigidBodyTransform parseRigidBodyTransform = parseRigidBodyTransform(urdfJoint.getOrigin());
      definition.getTransformToParent().set(parseRigidBodyTransform);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toJointDefinition(URDFJoint)
    */
   public static SixDoFJointDefinition toSixDoFJointDefinition(URDFJoint urdfJoint)
   {
      SixDoFJointDefinition definition = new SixDoFJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin()));

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toJointDefinition(URDFJoint)
    */
   public static PlanarJointDefinition toPlanarJointDefinition(URDFJoint urdfJoint)
   {
      PlanarJointDefinition definition = new PlanarJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin()));

      Vector3D surfaceNormal = parseAxis(urdfJoint.getAxis());

      if (!surfaceNormal.geometricallyEquals(Axis3D.Y, 1.0e-5))
         throw new UnsupportedOperationException("Planar joint are supported only with a surface normal equal to: "
                                                 + EuclidCoreIOTools.getTuple3DString(Axis3D.Y) + ", received:" + surfaceNormal);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF sensor into a {@link SensorDefinition}.
    * </p>
    * 
    * @param urdfSensor the parsed URDF sensor to convert.
    * @return the sensor definition.
    */
   public static List<SensorDefinition> toSensorDefinition(URDFSensor urdfSensor)
   {
      List<SensorDefinition> definitions = new ArrayList<>();

      switch (URDFSensorType.parse(urdfSensor.getType()))
      {
         case camera:
         case multicamera:
         case depth:
            definitions.addAll(toCameraSensorDefinition(urdfSensor.getCamera()));
            break;
         case imu:
            definitions.add(toIMUSensorDefinition(urdfSensor.getImu()));
            break;
         case gpu_ray:
         case ray:
            definitions.add(toLidarSensorDefinition(urdfSensor.getRay()));
            break;
         case force_torque:
            definitions.add(new WrenchSensorDefinition());
            break;
         default:
            LogTools.error("Unsupported sensor type: " + urdfSensor.getType());
            return null;
      }

      int updatePeriod = urdfSensor.getUpdateRate() == null ? -1 : (int) (1000.0 / parseDouble(urdfSensor.getUpdateRate(), 1000.0));

      for (SensorDefinition definition : definitions)
      {
         if (definition.getName() != null && !definition.getName().isEmpty())
            definition.setName(urdfSensor.getName() + "_" + definition.getName());
         else
            definition.setName(urdfSensor.getName());
         definition.getTransformToJoint().preMultiply(parsePose(urdfSensor.getPose()));
         definition.setUpdatePeriod(updatePeriod);
      }

      return definitions;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toSensorDefinition(URDFSensor)
    */
   public static List<CameraSensorDefinition> toCameraSensorDefinition(List<URDFCamera> urdfCameras)
   {
      return urdfCameras.stream().map(URDFTools::toCameraSensorDefinition).collect(Collectors.toList());
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toSensorDefinition(URDFSensor)
    */
   public static CameraSensorDefinition toCameraSensorDefinition(URDFCamera urdfCamera)
   {
      CameraSensorDefinition definition = new CameraSensorDefinition();
      definition.setName(urdfCamera.getName());
      definition.getTransformToJoint().set(parsePose(urdfCamera.getPose()));
      definition.setFieldOfView(parseDouble(urdfCamera.getHorizontalFov(), Double.NaN));
      definition.setClipNear(parseDouble(urdfCamera.getClip().getNear(), Double.NaN));
      definition.setClipFar(parseDouble(urdfCamera.getClip().getFar(), Double.NaN));
      definition.setImageWidth(parseInteger(urdfCamera.getImage().getWidth(), -1));
      definition.setImageHeight(parseInteger(urdfCamera.getImage().getHeight(), -1));
      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toSensorDefinition(URDFSensor)
    */
   public static LidarSensorDefinition toLidarSensorDefinition(URDFRay urdfRay)
   {
      LidarSensorDefinition definition = new LidarSensorDefinition();

      URDFRange urdfRange = urdfRay.getRange();
      double urdfRangeMax = parseDouble(urdfRange.getMax(), Double.NaN);
      double urdfRangeMin = parseDouble(urdfRange.getMin(), Double.NaN);
      double urdfRangeResolution = parseDouble(urdfRange.getResolution(), Double.NaN);

      URDFHorizontalScan urdfHorizontalScan = urdfRay.getScan().getHorizontal();
      URDFVerticalScan urdfVerticalScan = urdfRay.getScan().getVertical();
      double maxSweepAngle = parseDouble(urdfHorizontalScan.getMaxAngle(), 0.0);
      double minSweepAngle = parseDouble(urdfHorizontalScan.getMinAngle(), 0.0);
      double maxHeightAngle = urdfVerticalScan == null ? 0.0 : parseDouble(urdfVerticalScan.getMaxAngle(), 0.0);
      double minHeightAngle = urdfVerticalScan == null ? 0.0 : parseDouble(urdfVerticalScan.getMinAngle(), 0.0);

      int samples = parseInteger(urdfHorizontalScan.getSamples(), -1) / 3 * 3;
      int scanHeight = urdfVerticalScan == null ? 1 : parseInteger(urdfVerticalScan.getSamples(), 1);

      URDFNoise urdfNoise = urdfRay.getNoise();
      if (urdfNoise != null)
      {
         if (URDFNoiseType.gaussian.equals(URDFNoiseType.parse(urdfNoise.getType())))
         {
            definition.setGaussianNoiseMean(parseDouble(urdfNoise.getMean(), 0.0));
            definition.setGaussianNoiseStandardDeviation(parseDouble(urdfNoise.getStddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown noise model: {}.", urdfNoise.getType());
         }
      }

      definition.getTransformToJoint().set(parsePose(urdfRay.getPose()));
      definition.setPointsPerSweep(samples);
      definition.setSweepYawLimits(minSweepAngle, maxSweepAngle);
      definition.setHeightPitchLimits(minHeightAngle, maxHeightAngle);
      definition.setRangeLimits(urdfRangeMin, urdfRangeMax);
      definition.setRangeResolution(urdfRangeResolution);
      definition.setScanHeight(scanHeight);
      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * 
    * @see #toSensorDefinition(URDFSensor)
    */
   public static IMUSensorDefinition toIMUSensorDefinition(URDFIMU urdfIMU)
   {
      IMUSensorDefinition definition = new IMUSensorDefinition();

      URDFIMUNoise urdfNoise = urdfIMU.getNoise();
      if (urdfNoise != null)
      {
         if (URDFIMUNoiseType.gaussian.equals(URDFIMUNoiseType.parse(urdfNoise.getType())))
         {
            URDFNoiseParameters accelerationNoise = urdfNoise.getAccel();
            URDFNoiseParameters angularVelocityNoise = urdfNoise.getRate();

            definition.setAccelerationNoiseParameters(parseDouble(accelerationNoise.getMean(), 0.0), parseDouble(accelerationNoise.getStddev(), 0.0));
            definition.setAccelerationBiasParameters(parseDouble(accelerationNoise.getBias_mean(), 0.0), parseDouble(accelerationNoise.getBias_stddev(), 0.0));

            definition.setAngularVelocityNoiseParameters(parseDouble(angularVelocityNoise.getMean(), 0.0), parseDouble(angularVelocityNoise.getStddev(), 0.0));
            definition.setAngularVelocityBiasParameters(parseDouble(angularVelocityNoise.getBias_mean(), 0.0),
                                                        parseDouble(angularVelocityNoise.getBias_stddev(), 0.0));
         }
         else
         {
            LogTools.error("Unknown IMU noise model: {}.", urdfNoise.getType());
         }
      }

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF visual into a {@link VisualDefinition}.
    * </p>
    * 
    * @param urdfVisual the parsed URDF visual to convert.
    * @return the visual definition.
    */
   public static VisualDefinition toVisualDefinition(URDFVisual urdfVisual)
   {
      if (urdfVisual == null)
         return null;

      VisualDefinition visualDefinition = new VisualDefinition();
      visualDefinition.setName(urdfVisual.getName());
      visualDefinition.setOriginPose(parseRigidBodyTransform(urdfVisual.getOrigin()));
      visualDefinition.setMaterialDefinition(toMaterialDefinition(urdfVisual.getMaterial()));
      visualDefinition.setGeometryDefinition(toGeometryDefinition(urdfVisual.getGeometry()));
      return visualDefinition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF collision into a {@link CollisionShapeDefinition}.
    * </p>
    * 
    * @param urdfCollision the parsed URDF collision to convert.
    * @return the collision shape definition.
    */
   public static CollisionShapeDefinition toCollisionShapeDefinition(URDFCollision urdfCollision)
   {
      if (urdfCollision == null)
         return null;

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      collisionShapeDefinition.setName(urdfCollision.getName());
      collisionShapeDefinition.setOriginPose(parseRigidBodyTransform(urdfCollision.getOrigin()));
      collisionShapeDefinition.setGeometryDefinition(toGeometryDefinition(urdfCollision.getGeometry()));
      return collisionShapeDefinition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF geometry into a {@link GeometryDefinition}.
    * </p>
    * 
    * @param urdfGeometry the parsed URDF geometry to convert.
    * @return the geometry definition.
    */
   public static GeometryDefinition toGeometryDefinition(URDFGeometry urdfGeometry)
   {
      return toGeometryDefinition(urdfGeometry, Collections.emptyList());
   }

   public static GeometryDefinition toGeometryDefinition(URDFGeometry urdfGeometry, List<String> resourceDirectories)
   {
      if (urdfGeometry.getBox() != null)
      {
         Box3DDefinition boxGeometryDefinition = new Box3DDefinition();
         boxGeometryDefinition.setSize(parseVector3D(urdfGeometry.getBox().getSize(), null));
         return boxGeometryDefinition;
      }
      if (urdfGeometry.getCylinder() != null)
      {
         Cylinder3DDefinition cylinderGeometryDefinition = new Cylinder3DDefinition();
         cylinderGeometryDefinition.setRadius(parseDouble(urdfGeometry.getCylinder().getRadius(), 0.0));
         cylinderGeometryDefinition.setLength(parseDouble(urdfGeometry.getCylinder().getLength(), 0.0));
         return cylinderGeometryDefinition;
      }
      if (urdfGeometry.getSphere() != null)
      {
         Sphere3DDefinition sphereGeometryDefinition = new Sphere3DDefinition();
         sphereGeometryDefinition.setRadius(parseDouble(urdfGeometry.getSphere().getRadius(), 0.0));
         return sphereGeometryDefinition;
      }
      if (urdfGeometry.getMesh() != null)
      {
         ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition();
         modelFileGeometryDefinition.setResourceDirectories(resourceDirectories);
         modelFileGeometryDefinition.setFileName(urdfGeometry.getMesh().getFilename());
         modelFileGeometryDefinition.setScale(parseVector3D(urdfGeometry.getMesh().getScale(), new Vector3D(1, 1, 1)));
         return modelFileGeometryDefinition;
      }
      return null;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF material into a {@link MaterialDefinition}.
    * </p>
    * 
    * @param urdfMaterial the parsed URDF material to convert.
    * @return the material definition.
    */
   public static MaterialDefinition toMaterialDefinition(URDFMaterial urdfMaterial)
   {
      if (urdfMaterial == null)
         return null;

      MaterialDefinition materialDefinition = new MaterialDefinition();
      materialDefinition.setName(urdfMaterial.getName());
      materialDefinition.setDiffuseColor(toColorDefinition(urdfMaterial.getColor()));
      materialDefinition.setDiffuseMap(toTextureDefinition(urdfMaterial.getTexture()));
      return materialDefinition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF texture into a {@link TextureDefinition}.
    * </p>
    * 
    * @param urdfTexture the parsed URDF texture to convert.
    * @return the texture definition.
    */
   public static TextureDefinition toTextureDefinition(URDFTexture urdfTexture)
   {
      if (urdfTexture == null)
         return null;

      TextureDefinition textureDefinition = new TextureDefinition();
      textureDefinition.setFilename(urdfTexture.getFilename());
      return textureDefinition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Converts the given URDF color into a {@link ColorDefinition}.
    * </p>
    * 
    * @param urdfColor the parsed URDF color to convert.
    * @return the color definition.
    */
   public static ColorDefinition toColorDefinition(URDFColor urdfColor)
   {
      if (urdfColor == null)
         return null;

      double[] colorArray = parseArray(urdfColor.getRGBA(), null);
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
         String[] split = pose.strip().split("\\s+");
         Vector3D position = new Vector3D(Double.parseDouble(split[0]), Double.parseDouble(split[1]), Double.parseDouble(split[2]));
         YawPitchRoll orientation = new YawPitchRoll(Double.parseDouble(split[5]), Double.parseDouble(split[4]), Double.parseDouble(split[3]));
         rigidBodyTransform.set(orientation, position);
      }
      return rigidBodyTransform;
   }

   public static RigidBodyTransform parseRigidBodyTransform(URDFOrigin origin)
   {
      if (origin == null)
         origin = new URDFOrigin();

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.getTranslation().set(parseVector3D(origin.getXYZ(), DEFAULT_ORIGIN_XYZ));
      rigidBodyTransform.getRotation().setEuler(parseVector3D(origin.getRPY(), DEFAULT_ORIGIN_RPY));
      return rigidBodyTransform;
   }

   public static Matrix3D parseMomentOfInertia(URDFInertia inertia)
   {
      if (inertia == null)
         inertia = new URDFInertia();

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

   public static double parseMass(URDFMass urdfMass)
   {
      if (urdfMass == null)
         return DEFAULT_MASS;
      return parseDouble(urdfMass.getValue(), DEFAULT_MASS);
   }

   public static void parseLimit(URDFLimit urdfLimit, OneDoFJointDefinition jointDefinitionToParseLimitInto, boolean ignorePositionLimits)
   {
      jointDefinitionToParseLimitInto.setPositionLimits(DEFAULT_LOWER_LIMIT, DEFAULT_UPPER_LIMIT);
      jointDefinitionToParseLimitInto.setEffortLimits(DEFAULT_EFFORT_LIMIT);
      jointDefinitionToParseLimitInto.setVelocityLimits(DEFAULT_VELOCITY_LIMIT);

      if (urdfLimit != null)
      {
         if (!ignorePositionLimits)
         {
            double positionLowerLimit = parseDouble(urdfLimit.getLower(), DEFAULT_LOWER_LIMIT);
            double positionUpperLimit = parseDouble(urdfLimit.getUpper(), DEFAULT_UPPER_LIMIT);
            if (positionLowerLimit < positionUpperLimit)
               jointDefinitionToParseLimitInto.setPositionLimits(positionLowerLimit, positionUpperLimit);
         }
         double effortLimit = parseDouble(urdfLimit.getEffort(), DEFAULT_EFFORT_LIMIT);
         if (Double.isFinite(effortLimit) && effortLimit >= 0)
            jointDefinitionToParseLimitInto.setEffortLimits(effortLimit);
         double velocityLimit = parseDouble(urdfLimit.getVelocity(), DEFAULT_VELOCITY_LIMIT);
         if (Double.isFinite(velocityLimit) && velocityLimit >= 0)
            jointDefinitionToParseLimitInto.setVelocityLimits(velocityLimit);
      }
   }

   public static void parseDynamics(URDFDynamics urdfDynamics, OneDoFJointDefinition jointDefinitionToParseDynamicsInto)
   {
      double damping = 0.0;
      double stiction = 0.0;

      if (urdfDynamics != null)
      {
         damping = parseDouble(urdfDynamics.getDamping(), 0.0);
         stiction = parseDouble(urdfDynamics.getFriction(), 0.0);
      }

      jointDefinitionToParseDynamicsInto.setDamping(damping);
      jointDefinitionToParseDynamicsInto.setStiction(stiction);
   }

   public static Vector3D parseAxis(URDFAxis axis)
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

      String[] split = value.strip().split("\\s+");
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

      String[] split = value.strip().split("\\s+");
      double[] array = new double[split.length];

      for (int i = 0; i < split.length; i++)
         array[i] = Double.parseDouble(split[i]);

      return array;
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////// Following are tools for converting RobotDefinition to URDF ////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   public static void saveURDFModel(OutputStream outputStream, URDFModel urdfModel) throws JAXBException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(URDFModel.class);
         Marshaller m = context.createMarshaller();
         m.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         m.marshal(urdfModel, outputStream);
      }
      finally
      {
         try
         {
            outputStream.close();
         }
         catch (IOException e)
         {
            LogTools.error(e.getMessage());
         }
      }
   }

   public static URDFModel toURDFModel(RobotDefinition robotDefinition)
   {
      return toURDFModel(robotDefinition, DEFAULT_URDF_FORMATTER);
   }

   public static URDFModel toURDFModel(RobotDefinition robotDefinition, URDFFormatter urdfFormatter)
   {
      URDFModel urdfModel = new URDFModel();

      urdfModel.setName(robotDefinition.getName());
      urdfModel.setLinks(toURDFLinks(robotDefinition.getAllRigidBodies(), urdfFormatter));
      urdfModel.setJoints(toURDFJoints(robotDefinition.getAllJoints(), urdfFormatter));
      urdfModel.setGazebos(toURDFGazebos(robotDefinition.getAllJoints(), urdfFormatter));

      return urdfModel;
   }

   public static List<URDFLink> toURDFLinks(List<RigidBodyDefinition> rigidBodyDefinitions, URDFFormatter urdfFormatter)
   {
      if (rigidBodyDefinitions == null || rigidBodyDefinitions.isEmpty())
         return null;

      List<URDFLink> urdfLinks = new ArrayList<>();

      for (RigidBodyDefinition rigidBodyDefinition : rigidBodyDefinitions)
      {
         URDFLink urdfLink = toURDFLink(rigidBodyDefinition, urdfFormatter);
         if (urdfLink != null)
            urdfLinks.add(urdfLink);
      }

      return urdfLinks;
   }

   public static URDFLink toURDFLink(RigidBodyDefinition rigidBodyDefinition, URDFFormatter urdfFormatter)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFLink urdfLink = new URDFLink();
      urdfLink.setName(rigidBodyDefinition.getName());
      urdfLink.setInertial(toURDFInterial(rigidBodyDefinition, urdfFormatter));
      urdfLink.setVisual(toURDFVisuals(rigidBodyDefinition.getVisualDefinitions(), urdfFormatter));
      urdfLink.setCollision(toURDFCollisions(rigidBodyDefinition.getCollisionShapeDefinitions(), urdfFormatter));
      return urdfLink;
   }

   public static URDFInertial toURDFInterial(RigidBodyDefinition rigidBodyDefinition, URDFFormatter urdfFormatter)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFInertial urdfInertial = new URDFInertial();
      urdfInertial.setOrigin(toURDFOrigin(rigidBodyDefinition.getInertiaPose(), urdfFormatter));
      urdfInertial.setMass(toURDFMass(rigidBodyDefinition.getMass(), urdfFormatter));
      urdfInertial.setInertia(toURDFInertia(rigidBodyDefinition.getMomentOfInertia(), urdfFormatter));
      return urdfInertial;
   }

   public static URDFMass toURDFMass(double mass, URDFFormatter urdfFormatter)
   {
      URDFMass urdfMass = new URDFMass();
      urdfMass.setValue(urdfFormatter.toString(URDFMass.class, "value", mass));
      return urdfMass;
   }

   public static URDFInertia toURDFInertia(MomentOfInertiaDefinition momentOfInertiaDefinition, URDFFormatter urdfFormatter)
   {
      if (momentOfInertiaDefinition == null)
         return null;

      URDFInertia urdfInertia = new URDFInertia();
      urdfInertia.setIxx(urdfFormatter.toString(URDFInertia.class, "ixx", momentOfInertiaDefinition.getIxx()));
      urdfInertia.setIyy(urdfFormatter.toString(URDFInertia.class, "iyy", momentOfInertiaDefinition.getIyy()));
      urdfInertia.setIzz(urdfFormatter.toString(URDFInertia.class, "izz", momentOfInertiaDefinition.getIzz()));
      urdfInertia.setIxy(urdfFormatter.toString(URDFInertia.class, "ixy", momentOfInertiaDefinition.getIxy()));
      urdfInertia.setIxz(urdfFormatter.toString(URDFInertia.class, "ixz", momentOfInertiaDefinition.getIxz()));
      urdfInertia.setIyz(urdfFormatter.toString(URDFInertia.class, "iyz", momentOfInertiaDefinition.getIyz()));
      return urdfInertia;
   }

   public static List<URDFVisual> toURDFVisuals(List<VisualDefinition> visualDefinitions, URDFFormatter urdfFormatter)
   {
      if (visualDefinitions == null || visualDefinitions.isEmpty())
         return null;

      List<URDFVisual> urdfVisuals = new ArrayList<>();

      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         URDFVisual urdfVisual = toURDFVisual(visualDefinition, urdfFormatter);
         if (urdfVisual != null)
            urdfVisuals.add(urdfVisual);
      }

      return urdfVisuals;
   }

   public static URDFVisual toURDFVisual(VisualDefinition visualDefinition, URDFFormatter urdfFormatter)
   {
      if (visualDefinition == null)
         return null;

      URDFVisual urdfVisual = new URDFVisual();
      urdfVisual.setName(visualDefinition.getName());
      urdfVisual.setOrigin(toURDFOrigin(visualDefinition.getOriginPose(), urdfFormatter));
      urdfVisual.setGeometry(toURDFGeometry(visualDefinition.getGeometryDefinition(), urdfFormatter));
      urdfVisual.setMaterial(toURDFMaterial(visualDefinition.getMaterialDefinition(), urdfFormatter));
      return urdfVisual;
   }

   public static List<URDFCollision> toURDFCollisions(List<CollisionShapeDefinition> collisionShapeDefinitions, URDFFormatter urdfFormatter)
   {
      if (collisionShapeDefinitions == null || collisionShapeDefinitions.isEmpty())
         return null;

      List<URDFCollision> urdfCollisions = new ArrayList<>();

      for (CollisionShapeDefinition collisionShapeDefinition : collisionShapeDefinitions)
      {
         URDFCollision urdfCollision = toURDFCollision(collisionShapeDefinition, urdfFormatter);
         if (urdfCollision != null)
            urdfCollisions.add(urdfCollision);
      }

      return urdfCollisions;
   }

   public static URDFCollision toURDFCollision(CollisionShapeDefinition collisionShapeDefinition, URDFFormatter urdfFormatter)
   {
      if (collisionShapeDefinition == null)
         return null;

      URDFCollision urdfCollision = new URDFCollision();
      urdfCollision.setName(collisionShapeDefinition.getName());
      urdfCollision.setOrigin(toURDFOrigin(collisionShapeDefinition.getOriginPose(), urdfFormatter));
      urdfCollision.setGeometry(toURDFGeometry(collisionShapeDefinition.getGeometryDefinition(), urdfFormatter));
      return urdfCollision;
   }

   public static URDFGeometry toURDFGeometry(GeometryDefinition geometryDefinition, URDFFormatter urdfFormatter)
   {
      if (geometryDefinition == null)
         return null;

      URDFGeometry urdfGeometry = new URDFGeometry();

      if (geometryDefinition instanceof Box3DDefinition box3DGeometry)
      {
         urdfGeometry.setBox(toURDFBox(box3DGeometry, urdfFormatter));
      }
      else if (geometryDefinition instanceof Cylinder3DDefinition cylinder3DDefinition)
      {
         urdfGeometry.setCylinder(toURDFCylinder(cylinder3DDefinition, urdfFormatter));
      }
      else if (geometryDefinition instanceof Sphere3DDefinition sphere3DDefinition)
      {
         urdfGeometry.setSphere(toURDFSphere(sphere3DDefinition, urdfFormatter));
      }
      else if (geometryDefinition instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
      {
         urdfGeometry.setMesh(toURDFMesh(modelFileGeometryDefinition, urdfFormatter));
      }
      else
      {
         LogTools.warn("Unhandled geometry: {}", geometryDefinition);
      }

      return urdfGeometry;
   }

   public static URDFBox toURDFBox(Box3DDefinition box3DDefinition, URDFFormatter urdfFormatter)
   {
      if (box3DDefinition == null)
         return null;

      URDFBox urdfBox = new URDFBox();
      String sizeX = urdfFormatter.toString(URDFBox.class, "size", box3DDefinition.getSizeX());
      String sizeY = urdfFormatter.toString(URDFBox.class, "size", box3DDefinition.getSizeY());
      String sizeZ = urdfFormatter.toString(URDFBox.class, "size", box3DDefinition.getSizeZ());
      urdfBox.setSize("%s %s %s".formatted(sizeX, sizeY, sizeZ));
      return urdfBox;
   }

   public static URDFCylinder toURDFCylinder(Cylinder3DDefinition cylinder3DDefinition, URDFFormatter urdfFormatter)
   {
      if (cylinder3DDefinition == null)
         return null;

      URDFCylinder urdfCylinder = new URDFCylinder();
      urdfCylinder.setRadius(urdfFormatter.toString(URDFCylinder.class, "radius", cylinder3DDefinition.getRadius()));
      urdfCylinder.setLength(urdfFormatter.toString(URDFCylinder.class, "length", cylinder3DDefinition.getLength()));
      return urdfCylinder;
   }

   public static URDFSphere toURDFSphere(Sphere3DDefinition sphere3DDefinition, URDFFormatter urdfFormatter)
   {
      if (sphere3DDefinition == null)
         return null;

      URDFSphere urdfSphere = new URDFSphere();
      urdfSphere.setRadius(urdfFormatter.toString(URDFSphere.class, "radius", sphere3DDefinition.getRadius()));
      return urdfSphere;
   }

   public static URDFMesh toURDFMesh(ModelFileGeometryDefinition modelFileGeometryDefinition, URDFFormatter urdfFormatter)
   {
      if (modelFileGeometryDefinition == null)
         return null;

      URDFMesh urdfMesh = new URDFMesh();
      urdfMesh.setFilename(modelFileGeometryDefinition.getFileName());
      Vector3D scale = modelFileGeometryDefinition.getScale();
      if (scale != null && !scale.equals(new Vector3D(1, 1, 1)))
      {
         String scaleX = urdfFormatter.toString(URDFMesh.class, "scale", scale.getX());
         String scaleY = urdfFormatter.toString(URDFMesh.class, "scale", scale.getY());
         String scaleZ = urdfFormatter.toString(URDFMesh.class, "scale", scale.getZ());
         urdfMesh.setScale("%s %s %s".formatted(scaleX, scaleY, scaleZ));
      }
      return urdfMesh;
   }

   public static URDFMaterial toURDFMaterial(MaterialDefinition materialDefinition, URDFFormatter urdfFormatter)
   {
      if (materialDefinition == null)
         return null;

      URDFMaterial urdfMaterial = new URDFMaterial();
      urdfMaterial.setName(materialDefinition.getName());
      urdfMaterial.setColor(toURDFColor(materialDefinition.getDiffuseColor(), urdfFormatter));
      urdfMaterial.setTexture(toURDFTexture(materialDefinition.getDiffuseMap(), urdfFormatter));
      return urdfMaterial;
   }

   public static URDFColor toURDFColor(ColorDefinition colorDefinition, URDFFormatter urdfFormatter)
   {
      if (colorDefinition == null)
         return null;

      URDFColor urdfColor = new URDFColor();
      String r = urdfFormatter.toString(URDFColor.class, "rgba", colorDefinition.getRed());
      String g = urdfFormatter.toString(URDFColor.class, "rgba", colorDefinition.getGreen());
      String b = urdfFormatter.toString(URDFColor.class, "rgba", colorDefinition.getBlue());
      String a = urdfFormatter.toString(URDFColor.class, "rgba", colorDefinition.getAlpha());
      urdfColor.setRGBA("%s %s %s %s".formatted(r, g, b, a));
      return urdfColor;
   }

   public static URDFTexture toURDFTexture(TextureDefinition diffuseMap, URDFFormatter urdfFormatter)
   {
      if (diffuseMap == null)
         return null;

      URDFTexture urdfTexture = new URDFTexture();
      urdfTexture.setFilename(diffuseMap.getFilename());
      return urdfTexture;
   }

   public static List<URDFJoint> toURDFJoints(List<JointDefinition> jointDefinitions, URDFFormatter urdfFormatter)
   {
      if (jointDefinitions == null || jointDefinitions.isEmpty())
         return null;

      List<URDFJoint> urdfJoints = new ArrayList<>();

      for (JointDefinition jointDefinition : jointDefinitions)
      {
         URDFJoint urdfJoint = toURDFJoint(jointDefinition, urdfFormatter);
         if (urdfJoint != null)
            urdfJoints.add(urdfJoint);
      }

      return urdfJoints;
   }

   public static URDFJoint toURDFJoint(JointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      if (jointDefinition instanceof RevoluteJointDefinition revoluteJointDefinition)
         return toURDFJoint(revoluteJointDefinition, urdfFormatter);
      if (jointDefinition instanceof PrismaticJointDefinition prismaticJointDefinition)
         return toURDFJoint(prismaticJointDefinition, urdfFormatter);
      if (jointDefinition instanceof FixedJointDefinition fixedJointDefinition)
         return toURDFJoint(fixedJointDefinition, urdfFormatter);
      if (jointDefinition instanceof SixDoFJointDefinition sixDoFJointDefinition)
         return toURDFJoint(sixDoFJointDefinition, urdfFormatter);
      if (jointDefinition instanceof PlanarJointDefinition planarJointDefinition)
         return toURDFJoint(planarJointDefinition, urdfFormatter);
      throw new UnsupportedOperationException("Unsupported joint type: " + jointDefinition);
   }

   public static URDFJoint toURDFJoint(RevoluteJointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      if (Double.isInfinite(jointDefinition.getPositionLowerLimit()) && Double.isInfinite(jointDefinition.getPositionLowerLimit()))
         urdfJoint.setType(URDFJointType.continuous);
      else
         urdfJoint.setType(URDFJointType.revolute);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), urdfFormatter));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), urdfFormatter));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), urdfFormatter));
      urdfJoint.setAxis(toURDFAxis(jointDefinition.getAxis(), urdfFormatter));
      urdfJoint.setLimit(toURDFLimit(jointDefinition, urdfFormatter));
      urdfJoint.setDynamics(toURDFDynamics(jointDefinition, urdfFormatter));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(PrismaticJointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.prismatic);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), urdfFormatter));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), urdfFormatter));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), urdfFormatter));
      urdfJoint.setAxis(toURDFAxis(jointDefinition.getAxis(), urdfFormatter));
      urdfJoint.setLimit(toURDFLimit(jointDefinition, urdfFormatter));
      urdfJoint.setDynamics(toURDFDynamics(jointDefinition, urdfFormatter));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(FixedJointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.fixed);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), urdfFormatter));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), urdfFormatter));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), urdfFormatter));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(SixDoFJointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.floating);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), urdfFormatter));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), urdfFormatter));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), urdfFormatter));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(PlanarJointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.planar);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), urdfFormatter));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), urdfFormatter));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), urdfFormatter));
      return urdfJoint;
   }

   public static List<URDFGazebo> toURDFGazebos(List<JointDefinition> jointDefinitions, URDFFormatter urdfFormatter)
   {
      if (jointDefinitions == null || jointDefinitions.isEmpty())
         return null;

      List<URDFGazebo> urdfGazebos = new ArrayList<>();

      for (JointDefinition jointDefinition : jointDefinitions)
      {
         List<URDFGazebo> jointURDFGazebos = toURDFGazebos(jointDefinition, urdfFormatter);
         if (jointURDFGazebos != null)
            urdfGazebos.addAll(jointURDFGazebos);
      }

      return urdfGazebos;
   }

   public static List<URDFGazebo> toURDFGazebos(JointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;
      List<URDFSensor> urdfSensors = toURDFSensors(jointDefinition.getSensorDefinitions(), urdfFormatter);
      if (urdfSensors == null)
         return null;

      List<URDFGazebo> urdfGazebos = new ArrayList<>();

      for (URDFSensor urdfSensor : urdfSensors)
      {
         URDFGazebo urdfGazebo = new URDFGazebo();
         urdfGazebo.setReference(jointDefinition.getName());
         urdfGazebo.setSensor(urdfSensor);
         urdfGazebos.add(urdfGazebo);
      }

      return urdfGazebos;
   }

   public static List<URDFSensor> toURDFSensors(List<SensorDefinition> sensorDefinitions, URDFFormatter urdfFormatter)
   {
      if (sensorDefinitions == null || sensorDefinitions.isEmpty())
         return null;

      List<URDFSensor> urdfSensors = new ArrayList<>();

      for (SensorDefinition sensorDefinition : sensorDefinitions)
      {
         URDFSensor urdfSensor = toURDFSensor(sensorDefinition, urdfFormatter);
         if (urdfSensor != null)
            urdfSensors.add(urdfSensor);
      }

      return urdfSensors;
   }

   public static URDFSensor toURDFSensor(SensorDefinition sensorDefinition, URDFFormatter urdfFormatter)
   {
      if (sensorDefinition == null)
         return null;

      if (sensorDefinition instanceof CameraSensorDefinition cameraSensorDefinition)
         return toURDFSensor(cameraSensorDefinition, urdfFormatter);
      if (sensorDefinition instanceof LidarSensorDefinition lidarSensorDefinition)
         return toURDFSensor(lidarSensorDefinition, urdfFormatter);
      if (sensorDefinition instanceof IMUSensorDefinition imuSensorDefinition)
         return toURDFSensor(imuSensorDefinition, urdfFormatter);
      if (sensorDefinition instanceof WrenchSensorDefinition wrenchSensorDefinition)
         return toURDFSensor(wrenchSensorDefinition, urdfFormatter);
      LogTools.warn("Unsupported sensor type: " + sensorDefinition);
      return null;
   }

   public static URDFSensor toURDFSensor(WrenchSensorDefinition sensorDefinition, URDFFormatter urdfFormatter)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), urdfFormatter.getDoubleFormatter(URDFSensor.class, "pose")));
      urdfSensor.setUpdateRate(urdfFormatter.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.force_torque);
      return urdfSensor;
   }

   public static URDFSensor toURDFSensor(IMUSensorDefinition sensorDefinition, URDFFormatter urdfFormatter)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), urdfFormatter.getDoubleFormatter(URDFSensor.class, "pose")));
      urdfSensor.setUpdateRate(urdfFormatter.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.imu);

      URDFIMU urdfIMU = new URDFIMU();

      URDFIMUNoise urdfIMUNoise = new URDFIMUNoise();
      urdfIMUNoise.setType(URDFIMUNoiseType.gaussian);

      { // Angular velocity noise
         URDFNoiseParameters urdfNoiseParameters = new URDFNoiseParameters();
         String mean = urdfFormatter.toString(URDFNoiseParameters.class, "mean", sensorDefinition.getAngularVelocityNoiseMean());
         String stddev = urdfFormatter.toString(URDFNoiseParameters.class, "stddev", sensorDefinition.getAngularVelocityNoiseStandardDeviation());
         String bias_mean = urdfFormatter.toString(URDFNoiseParameters.class, "bias_mean", sensorDefinition.getAngularVelocityBiasMean());
         String bias_stddev = urdfFormatter.toString(URDFNoiseParameters.class, "bias_stddev", sensorDefinition.getAngularVelocityBiasStandardDeviation());
         urdfNoiseParameters.setMean(mean);
         urdfNoiseParameters.setStddev(stddev);
         urdfNoiseParameters.setBias_mean(bias_mean);
         urdfNoiseParameters.setBias_stddev(bias_stddev);
         urdfIMUNoise.setRate(urdfNoiseParameters);
      }

      { // Acceleration noise
         String mean = urdfFormatter.toString(URDFNoiseParameters.class, "mean", sensorDefinition.getAccelerationNoiseMean());
         String stddev = urdfFormatter.toString(URDFNoiseParameters.class, "stddev", sensorDefinition.getAccelerationNoiseStandardDeviation());
         String bias_mean = urdfFormatter.toString(URDFNoiseParameters.class, "bias_mean", sensorDefinition.getAccelerationBiasMean());
         String bias_stddev = urdfFormatter.toString(URDFNoiseParameters.class, "bias_stddev", sensorDefinition.getAccelerationBiasStandardDeviation());
         URDFNoiseParameters urdfNoiseParameters = new URDFNoiseParameters();
         urdfNoiseParameters.setMean(mean);
         urdfNoiseParameters.setStddev(stddev);
         urdfNoiseParameters.setBias_mean(bias_mean);
         urdfNoiseParameters.setBias_stddev(bias_stddev);
         urdfIMUNoise.setAccel(urdfNoiseParameters);
      }

      urdfIMU.setNoise(urdfIMUNoise);

      return urdfSensor;
   }

   public static URDFSensor toURDFSensor(LidarSensorDefinition sensorDefinition, URDFFormatter urdfFormatter)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), urdfFormatter.getDoubleFormatter(URDFSensor.class, "pose")));
      urdfSensor.setUpdateRate(urdfFormatter.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.ray);
      urdfSensor.setRay(toURDFRay(sensorDefinition, urdfFormatter));
      return urdfSensor;
   }

   public static URDFRay toURDFRay(LidarSensorDefinition sensorDefinition, URDFFormatter urdfFormatter)
   {
      if (sensorDefinition == null)
         return null;

      URDFRay urdfRay = new URDFRay();

      URDFRange urdfRange = new URDFRange();
      urdfRange.setMin(urdfFormatter.toString(URDFRange.class, "min", sensorDefinition.getMinRange()));
      urdfRange.setMax(urdfFormatter.toString(URDFRange.class, "max", sensorDefinition.getMaxRange()));
      urdfRange.setResolution(urdfFormatter.toString(URDFRange.class, "resolution", sensorDefinition.getRangeResolution()));
      urdfRay.setRange(urdfRange);

      URDFScan urdfScan = new URDFScan();

      URDFHorizontalScan urdfHorizontalScan = new URDFHorizontalScan();
      urdfHorizontalScan.setMinAngle(urdfFormatter.toString(URDFHorizontalScan.class, "min_angle", sensorDefinition.getSweepYawMin()));
      urdfHorizontalScan.setMaxAngle(urdfFormatter.toString(URDFHorizontalScan.class, "max_angle", sensorDefinition.getSweepYawMax()));
      urdfHorizontalScan.setSamples(urdfFormatter.toString(URDFHorizontalScan.class, "samples", sensorDefinition.getPointsPerSweep()));
      urdfScan.setHorizontal(urdfHorizontalScan);

      URDFVerticalScan urdfVerticalScan = new URDFVerticalScan();
      urdfVerticalScan.setMinAngle(urdfFormatter.toString(URDFVerticalScan.class, "min_angle", sensorDefinition.getHeightPitchMin()));
      urdfVerticalScan.setMaxAngle(urdfFormatter.toString(URDFVerticalScan.class, "max_angle", sensorDefinition.getHeightPitchMax()));
      urdfVerticalScan.setSamples(urdfFormatter.toString(URDFVerticalScan.class, "samples", sensorDefinition.getScanHeight()));
      urdfScan.setVertical(urdfVerticalScan);
      urdfRay.setScan(urdfScan);

      URDFNoise urdfNoise = new URDFNoise();
      urdfNoise.setType(URDFNoiseType.gaussian);
      urdfNoise.setMean(urdfFormatter.toString(URDFNoise.class, "mean", sensorDefinition.getGaussianNoiseMean()));
      urdfNoise.setStddev(urdfFormatter.toString(URDFNoise.class, "stddev", sensorDefinition.getGaussianNoiseStandardDeviation()));
      urdfRay.setNoise(urdfNoise);

      return urdfRay;
   }

   public static URDFSensor toURDFSensor(CameraSensorDefinition sensorDefinition, URDFFormatter urdfFormatter)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      String name = sensorDefinition.getName();
      if (name.contains("_"))
         urdfSensor.setName(name.substring(0, name.lastIndexOf("_")));
      else
         urdfSensor.setName(name);
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), urdfFormatter.getDoubleFormatter(URDFSensor.class, "pose")));
      urdfSensor.setUpdateRate(urdfFormatter.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.camera);
      urdfSensor.setCamera(Collections.singletonList(toURDFCamera(sensorDefinition, urdfFormatter)));
      return urdfSensor;
   }

   public static URDFCamera toURDFCamera(CameraSensorDefinition sensorDefinition, URDFFormatter urdfFormatter)
   {
      if (sensorDefinition == null)
         return null;

      URDFCamera urdfCamera = new URDFCamera();
      String name = sensorDefinition.getName();
      if (name != null && name.contains("_"))
         urdfCamera.setName(name.substring(name.lastIndexOf("_") + 1));
      urdfCamera.setPose(toPoseString(sensorDefinition.getTransformToJoint(), urdfFormatter.getDoubleFormatter(URDFSensor.class, "pose")));
      urdfCamera.setHorizontalFov(sensorDefinition.getFieldOfView());
      urdfCamera.setImage(toURDFSensorImage(sensorDefinition.getImageWidth(), sensorDefinition.getImageHeight(), urdfFormatter));
      urdfCamera.setClip(toURDFClip(sensorDefinition.getClipNear(), sensorDefinition.getClipFar(), urdfFormatter));
      return urdfCamera;
   }

   public static URDFSensorImage toURDFSensorImage(int width, int height, URDFFormatter urdfFormatter)
   {
      return toURDFSensorImage(width, height, null, urdfFormatter);
   }

   public static URDFSensorImage toURDFSensorImage(int width, int height, String format, URDFFormatter urdfFormatter)
   {
      URDFSensorImage urdfSensorImage = new URDFSensorImage();
      urdfSensorImage.setWidth(width);
      urdfSensorImage.setHeight(height);
      urdfSensorImage.setFormat(format);
      return urdfSensorImage;
   }

   public static URDFClip toURDFClip(double near, double far, URDFFormatter urdfFormatter)
   {
      URDFClip urdfClip = new URDFClip();
      urdfClip.setNear(urdfFormatter.toString(URDFClip.class, "near", near));
      urdfClip.setFar(urdfFormatter.toString(URDFClip.class, "far", far));
      return urdfClip;
   }

   public static URDFOrigin toURDFOrigin(RigidBodyTransformReadOnly pose, URDFFormatter urdfFormatter)
   {
      if (pose == null)
         return null;

      URDFOrigin urdfOrigin = new URDFOrigin();
      Tuple3DReadOnly translation = pose.getTranslation();
      Orientation3DReadOnly rotation = pose.getRotation();

      String tx = urdfFormatter.toString(URDFOrigin.class, "xyz", translation.getX());
      String ty = urdfFormatter.toString(URDFOrigin.class, "xyz", translation.getY());
      String tz = urdfFormatter.toString(URDFOrigin.class, "xyz", translation.getZ());
      String rr = urdfFormatter.toString(URDFOrigin.class, "rpy", rotation.getRoll());
      String rp = urdfFormatter.toString(URDFOrigin.class, "rpy", rotation.getPitch());
      String ry = urdfFormatter.toString(URDFOrigin.class, "rpy", rotation.getYaw());

      urdfOrigin.setXYZ("%s %s %s".formatted(tx, ty, tz));
      urdfOrigin.setRPY("%s %s %s".formatted(rr, rp, ry));
      return urdfOrigin;
   }

   public static URDFOrigin toURDFOrigin(AffineTransformReadOnly pose, URDFFormatter urdfFormatter)
   {
      if (pose == null)
         return null;

      URDFOrigin urdfOrigin = new URDFOrigin();
      Tuple3DReadOnly translation = pose.getTranslation();
      Orientation3DReadOnly rotation = pose.getLinearTransform().getAsQuaternion();

      String tx = urdfFormatter.toString(URDFOrigin.class, "xyz", translation.getX());
      String ty = urdfFormatter.toString(URDFOrigin.class, "xyz", translation.getY());
      String tz = urdfFormatter.toString(URDFOrigin.class, "xyz", translation.getZ());
      String rr = urdfFormatter.toString(URDFOrigin.class, "rpy", rotation.getRoll());
      String rp = urdfFormatter.toString(URDFOrigin.class, "rpy", rotation.getPitch());
      String ry = urdfFormatter.toString(URDFOrigin.class, "rpy", rotation.getYaw());

      urdfOrigin.setXYZ("%s %s %s".formatted(tx, ty, tz));
      urdfOrigin.setRPY("%s %s %s".formatted(rr, rp, ry));

      if (!EuclidCoreTools.epsilonEquals(new Vector3D(1, 1, 1), pose.getLinearTransform().getScaleVector(), 1.0e-7))
         LogTools.warn("Discarding scale from affine trane transform.");
      return urdfOrigin;
   }

   public static String toPoseString(RigidBodyTransformReadOnly pose, DoubleFormatter doubleFormatter)
   {
      if (pose == null)
         return null;

      Tuple3DReadOnly translation = pose.getTranslation();
      Orientation3DReadOnly rotation = pose.getRotation();
      String tx = doubleFormatter.toString(translation.getX());
      String ty = doubleFormatter.toString(translation.getY());
      String tz = doubleFormatter.toString(translation.getZ());
      String rr = doubleFormatter.toString(rotation.getRoll());
      String rp = doubleFormatter.toString(rotation.getPitch());
      String ry = doubleFormatter.toString(rotation.getYaw());
      return "%s %s %s %s %s %s".formatted(tx, ty, tz, rr, rp, ry);
   }

   public static URDFAxis toURDFAxis(Tuple3DReadOnly axis, URDFFormatter urdfFormatter)
   {
      if (axis == null)
         return null;

      URDFAxis urdfAxis = new URDFAxis();
      String x = urdfFormatter.toString(URDFAxis.class, "xyz", axis.getX());
      String y = urdfFormatter.toString(URDFAxis.class, "xyz", axis.getY());
      String z = urdfFormatter.toString(URDFAxis.class, "xyz", axis.getZ());
      urdfAxis.setXYZ("%s %s %s".formatted(x, y, z));
      return urdfAxis;
   }

   public static URDFLimit toURDFLimit(OneDoFJointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      URDFLimit urdfLimit = new URDFLimit();
      urdfLimit.setLower(urdfFormatter.toString(URDFLimit.class, "lower", jointDefinition.getPositionLowerLimit()));
      urdfLimit.setUpper(urdfFormatter.toString(URDFLimit.class, "upper", jointDefinition.getPositionUpperLimit()));

      if (-jointDefinition.getVelocityLowerLimit() != jointDefinition.getVelocityUpperLimit())
      {
         LogTools.warn("Velocity limits no symmetric for joint {}, exporting smallest limit", jointDefinition.getName());
      }

      double velocity = Math.min(Math.abs(jointDefinition.getVelocityLowerLimit()), jointDefinition.getVelocityUpperLimit());
      urdfLimit.setVelocity(urdfFormatter.toString(URDFLimit.class, "velocity", velocity));

      if (-jointDefinition.getEffortLowerLimit() != jointDefinition.getEffortUpperLimit())
      {
         LogTools.warn("Effort limits no symmetric for joint {}, exporting smallest limit", jointDefinition.getName());
      }

      double effort = Math.min(Math.abs(jointDefinition.getEffortLowerLimit()), jointDefinition.getEffortUpperLimit());
      urdfLimit.setEffort(urdfFormatter.toString(URDFLimit.class, "effort", effort));

      return urdfLimit;
   }

   public static URDFDynamics toURDFDynamics(OneDoFJointDefinition jointDefinition, URDFFormatter urdfFormatter)
   {
      if (jointDefinition == null)
         return null;

      URDFDynamics urdfDynamics = new URDFDynamics();
      urdfDynamics.setFriction(urdfFormatter.toString(URDFDynamics.class, "friction", jointDefinition.getStiction()));
      urdfDynamics.setDamping(urdfFormatter.toString(URDFDynamics.class, "damping", jointDefinition.getDamping()));
      return urdfDynamics;
   }

   public static URDFLinkReference toURDFLinkReference(RigidBodyDefinition rigidBodyDefinition, URDFFormatter urdfFormatter)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFLinkReference urdfLinkReference = new URDFLinkReference();
      urdfLinkReference.setLink(rigidBodyDefinition.getName());
      return urdfLinkReference;
   }

   public static interface DoubleFormatter
   {
      String toString(double value);
   }

   public static class URDFFormatter
   {
      private DoubleFormatter defaultDoubleFormatter = Double::toString;
      private final Map<Class<? extends URDFItem>, URDFItemFormatter> urdfTypeFormatters = new HashMap<>();

      public void setDefaultDoubleFormatter(DoubleFormatter formatter)
      {
         this.defaultDoubleFormatter = formatter;
      }

      public void addDoubleFormatter(Class<? extends URDFItem> urdfType, DoubleFormatter formatter)
      {
         URDFItemFormatter urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter == null)
            urdfTypeFormatters.put(urdfType, urdfTypeFormatter = new URDFItemFormatter());
         urdfTypeFormatter.setDefaultFormatter(formatter);
      }

      public void addDoubleFormatter(Class<? extends URDFItem> urdfType, String fieldName, DoubleFormatter formatter)
      {
         URDFItemFormatter urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter == null)
            urdfTypeFormatters.put(urdfType, urdfTypeFormatter = new URDFItemFormatter());
         urdfTypeFormatter.addFormatter(fieldName, formatter);
      }

      public String toString(Class<? extends URDFItem> urdfType, String fieldName, double value)
      {
         DoubleFormatter formatter = getDoubleFormatter(urdfType, fieldName);
         if (formatter != null)
            return formatter.toString(value);
         else
            return null;
      }

      public DoubleFormatter getDoubleFormatter(Class<? extends URDFItem> urdfType)
      {
         URDFItemFormatter urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter != null)
         {
            if (urdfTypeFormatter.defaultDoubleFormatter != null)
               return urdfTypeFormatter.defaultDoubleFormatter;
         }

         return defaultDoubleFormatter;
      }

      public DoubleFormatter getDoubleFormatter(Class<? extends URDFItem> urdfType, String fieldName)
      {
         URDFItemFormatter urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter != null)
         {
            DoubleFormatter formatter = urdfTypeFormatter.getDoubleFormatter(fieldName.toLowerCase());
            if (formatter != null)
               return formatter;
         }

         return defaultDoubleFormatter;
      }
   }

   private static class URDFItemFormatter
   {
      private DoubleFormatter defaultDoubleFormatter;
      private final Map<String, DoubleFormatter> fieldToDoubleFormatterMap = new HashMap<>();

      public URDFItemFormatter()
      {
      }

      public void setDefaultFormatter(DoubleFormatter formatter)
      {
         defaultDoubleFormatter = formatter;
      }

      public void addFormatter(String fieldName, DoubleFormatter formatter)
      {
         fieldToDoubleFormatterMap.put(fieldName.toLowerCase(), formatter);
      }

      public DoubleFormatter getDoubleFormatter(String fieldName)
      {
         DoubleFormatter formatter = fieldToDoubleFormatterMap.get(fieldName.toLowerCase());
         if (formatter != null)
            return formatter;
         else if (defaultDoubleFormatter != null)
            return defaultDoubleFormatter;
         else
            return null;
      }
   }
}
