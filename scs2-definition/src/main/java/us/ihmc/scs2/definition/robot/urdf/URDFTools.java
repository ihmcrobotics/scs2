package us.ihmc.scs2.definition.robot.urdf;

import java.io.File;
import java.io.InputStream;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
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

   public static URDFModel toURDFModel(RobotDefinition robotDefinition)
   {
      URDFModel urdfModel = new URDFModel();

      urdfModel.setName(robotDefinition.getName());
      urdfModel.setLinks(toURDFLinks(robotDefinition.getAllRigidBodies()));
      urdfModel.setJoints(toURDFJoints(robotDefinition.getAllJoints()));
      urdfModel.setGazebos(toURDFGazebos(robotDefinition.getAllJoints()));

      return urdfModel;
   }

   public static List<URDFLink> toURDFLinks(List<RigidBodyDefinition> rigidBodyDefinitions)
   {
      if (rigidBodyDefinitions == null || rigidBodyDefinitions.isEmpty())
         return null;

      List<URDFLink> urdfLinks = new ArrayList<>();

      for (RigidBodyDefinition rigidBodyDefinition : rigidBodyDefinitions)
      {
         URDFLink urdfLink = toURDFLink(rigidBodyDefinition);
         if (urdfLink != null)
            urdfLinks.add(urdfLink);
      }

      return urdfLinks;
   }

   public static URDFLink toURDFLink(RigidBodyDefinition rigidBodyDefinition)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFLink urdfLink = new URDFLink();
      urdfLink.setName(rigidBodyDefinition.getName());
      urdfLink.setInertial(toURDFInterial(rigidBodyDefinition));
      urdfLink.setVisual(toURDFVisuals(rigidBodyDefinition.getVisualDefinitions()));
      urdfLink.setCollision(toURDFCollisions(rigidBodyDefinition.getCollisionShapeDefinitions()));
      return urdfLink;
   }

   public static URDFInertial toURDFInterial(RigidBodyDefinition rigidBodyDefinition)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFInertial urdfInertial = new URDFInertial();
      urdfInertial.setOrigin(toURDFOrigin(rigidBodyDefinition.getInertiaPose()));
      urdfInertial.setMass(toURDFMass(rigidBodyDefinition.getMass()));
      urdfInertial.setInertia(toURDFInertia(rigidBodyDefinition.getMomentOfInertia()));
      return urdfInertial;
   }

   public static URDFMass toURDFMass(double mass)
   {
      URDFMass urdfMass = new URDFMass();
      urdfMass.setValue(mass);
      return urdfMass;
   }

   public static URDFInertia toURDFInertia(MomentOfInertiaDefinition momentOfInertiaDefinition)
   {
      if (momentOfInertiaDefinition == null)
         return null;

      URDFInertia urdfInertia = new URDFInertia();
      urdfInertia.setIxx(momentOfInertiaDefinition.getIxx());
      urdfInertia.setIyy(momentOfInertiaDefinition.getIyy());
      urdfInertia.setIzz(momentOfInertiaDefinition.getIzz());
      urdfInertia.setIxy(momentOfInertiaDefinition.getIxy());
      urdfInertia.setIxz(momentOfInertiaDefinition.getIxz());
      urdfInertia.setIyz(momentOfInertiaDefinition.getIyz());
      return urdfInertia;
   }

   public static List<URDFVisual> toURDFVisuals(List<VisualDefinition> visualDefinitions)
   {
      if (visualDefinitions == null || visualDefinitions.isEmpty())
         return null;

      List<URDFVisual> urdfVisuals = new ArrayList<>();

      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         URDFVisual urdfVisual = toURDFVisual(visualDefinition);
         if (urdfVisual != null)
            urdfVisuals.add(urdfVisual);
      }

      return urdfVisuals;
   }

   public static URDFVisual toURDFVisual(VisualDefinition visualDefinition)
   {
      if (visualDefinition == null)
         return null;

      URDFVisual urdfVisual = new URDFVisual();
      urdfVisual.setName(visualDefinition.getName());
      urdfVisual.setOrigin(toURDFOrigin(visualDefinition.getOriginPose()));
      urdfVisual.setGeometry(toURDFGeometry(visualDefinition.getGeometryDefinition()));
      urdfVisual.setMaterial(toURDFMaterial(visualDefinition.getMaterialDefinition()));
      return urdfVisual;
   }

   public static List<URDFCollision> toURDFCollisions(List<CollisionShapeDefinition> collisionShapeDefinitions)
   {
      if (collisionShapeDefinitions == null || collisionShapeDefinitions.isEmpty())
         return null;

      List<URDFCollision> urdfCollisions = new ArrayList<>();

      for (CollisionShapeDefinition collisionShapeDefinition : collisionShapeDefinitions)
      {
         URDFCollision urdfCollision = toURDFCollision(collisionShapeDefinition);
         if (urdfCollision != null)
            urdfCollisions.add(urdfCollision);
      }

      return urdfCollisions;
   }

   public static URDFCollision toURDFCollision(CollisionShapeDefinition collisionShapeDefinition)
   {
      if (collisionShapeDefinition == null)
         return null;

      URDFCollision urdfCollision = new URDFCollision();
      urdfCollision.setName(collisionShapeDefinition.getName());
      urdfCollision.setOrigin(toURDFOrigin(collisionShapeDefinition.getOriginPose()));
      urdfCollision.setGeometry(toURDFGeometry(collisionShapeDefinition.getGeometryDefinition()));
      return urdfCollision;
   }

   public static URDFGeometry toURDFGeometry(GeometryDefinition geometryDefinition)
   {
      if (geometryDefinition == null)
         return null;

      URDFGeometry urdfGeometry = new URDFGeometry();

      if (geometryDefinition instanceof Box3DDefinition box3DGeometry)
      {
         urdfGeometry.setBox(toURDFBox(box3DGeometry));
      }
      else if (geometryDefinition instanceof Cylinder3DDefinition cylinder3DDefinition)
      {
         urdfGeometry.setCylinder(toURDFCylinder(cylinder3DDefinition));
      }
      else if (geometryDefinition instanceof Sphere3DDefinition sphere3DDefinition)
      {
         urdfGeometry.setSphere(toURDFSphere(sphere3DDefinition));
      }
      else if (geometryDefinition instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
      {
         urdfGeometry.setMesh(toURDFMesh(modelFileGeometryDefinition));
      }
      else
      {
         LogTools.warn("Unhandled geometry: {}", geometryDefinition);
      }

      return urdfGeometry;
   }

   public static URDFBox toURDFBox(Box3DDefinition box3DDefinition)
   {
      if (box3DDefinition == null)
         return null;

      URDFBox urdfBox = new URDFBox();
      urdfBox.setSize(EuclidCoreIOTools.getStringOf(" ", null, box3DDefinition.getSizeX(), box3DDefinition.getSizeY(), box3DDefinition.getSizeZ()));
      return urdfBox;
   }

   public static URDFCylinder toURDFCylinder(Cylinder3DDefinition cylinder3DDefinition)
   {
      if (cylinder3DDefinition == null)
         return null;

      URDFCylinder urdfCylinder = new URDFCylinder();
      urdfCylinder.setRadius(cylinder3DDefinition.getRadius());
      urdfCylinder.setLength(cylinder3DDefinition.getLength());
      return urdfCylinder;
   }

   public static URDFSphere toURDFSphere(Sphere3DDefinition sphere3DDefinition)
   {
      if (sphere3DDefinition == null)
         return null;

      URDFSphere urdfSphere = new URDFSphere();
      urdfSphere.setRadius(sphere3DDefinition.getRadius());
      return urdfSphere;
   }

   public static URDFMesh toURDFMesh(ModelFileGeometryDefinition modelFileGeometryDefinition)
   {
      if (modelFileGeometryDefinition == null)
         return null;

      URDFMesh urdfMesh = new URDFMesh();
      urdfMesh.setFilename(modelFileGeometryDefinition.getFileName());
      Vector3D scale = modelFileGeometryDefinition.getScale();
      if (scale != null)
         urdfMesh.setScale(EuclidCoreIOTools.getStringOf(" ", null, scale.getX(), scale.getY(), scale.getZ()));
      return urdfMesh;
   }

   public static URDFMaterial toURDFMaterial(MaterialDefinition materialDefinition)
   {
      if (materialDefinition == null)
         return null;

      URDFMaterial urdfMaterial = new URDFMaterial();
      urdfMaterial.setName(materialDefinition.getName());
      urdfMaterial.setColor(toURDFColor(materialDefinition.getDiffuseColor()));
      urdfMaterial.setTexture(toURDFTexture(materialDefinition.getDiffuseMap()));
      return urdfMaterial;
   }

   public static URDFColor toURDFColor(ColorDefinition colorDefinition)
   {
      if (colorDefinition == null)
         return null;

      URDFColor urdfColor = new URDFColor();
      urdfColor.setRGBA(EuclidCoreIOTools.getStringOf(" ",
                                                      null,
                                                      colorDefinition.getRed(),
                                                      colorDefinition.getGreen(),
                                                      colorDefinition.getBlue(),
                                                      colorDefinition.getAlpha()));
      return urdfColor;
   }

   public static URDFTexture toURDFTexture(TextureDefinition diffuseMap)
   {
      if (diffuseMap == null)
         return null;

      URDFTexture urdfTexture = new URDFTexture();
      urdfTexture.setFilename(diffuseMap.getFilename());
      return urdfTexture;
   }

   public static List<URDFJoint> toURDFJoints(List<JointDefinition> jointDefinitions)
   {
      if (jointDefinitions == null || jointDefinitions.isEmpty())
         return null;

      List<URDFJoint> urdfJoints = new ArrayList<>();

      for (JointDefinition jointDefinition : jointDefinitions)
      {
         URDFJoint urdfJoint = toURDFJoint(jointDefinition);
         if (urdfJoint != null)
            urdfJoints.add(urdfJoint);
      }

      return urdfJoints;
   }

   public static URDFJoint toURDFJoint(JointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      if (jointDefinition instanceof RevoluteJointDefinition revoluteJointDefinition)
         return toURDFJoint(revoluteJointDefinition);
      if (jointDefinition instanceof PrismaticJointDefinition prismaticJointDefinition)
         return toURDFJoint(prismaticJointDefinition);
      if (jointDefinition instanceof FixedJointDefinition fixedJointDefinition)
         return toURDFJoint(fixedJointDefinition);
      if (jointDefinition instanceof SixDoFJointDefinition sixDoFJointDefinition)
         return toURDFJoint(sixDoFJointDefinition);
      if (jointDefinition instanceof PlanarJointDefinition planarJointDefinition)
         return toURDFJoint(planarJointDefinition);
      throw new UnsupportedOperationException("Unsupported joint type: " + jointDefinition);
   }

   public static URDFJoint toURDFJoint(RevoluteJointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      if (Double.isInfinite(jointDefinition.getPositionLowerLimit()) && Double.isInfinite(jointDefinition.getPositionLowerLimit()))
         urdfJoint.setType(URDFJointType.continuous);
      else
         urdfJoint.setType(URDFJointType.revolute);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent()));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor()));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor()));
      urdfJoint.setAxis(toURDFAxis(jointDefinition.getAxis()));
      urdfJoint.setLimit(toURDFLimit(jointDefinition));
      urdfJoint.setDynamics(toURDFDynamics(jointDefinition));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(PrismaticJointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.prismatic);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent()));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor()));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor()));
      urdfJoint.setAxis(toURDFAxis(jointDefinition.getAxis()));
      urdfJoint.setLimit(toURDFLimit(jointDefinition));
      urdfJoint.setDynamics(toURDFDynamics(jointDefinition));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(FixedJointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.fixed);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent()));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor()));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor()));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(SixDoFJointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.floating);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent()));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor()));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor()));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(PlanarJointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.planar);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent()));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor()));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor()));
      return urdfJoint;
   }

   public static List<URDFGazebo> toURDFGazebos(List<JointDefinition> jointDefinitions)
   {
      if (jointDefinitions == null || jointDefinitions.isEmpty())
         return null;

      List<URDFGazebo> urdfGazebos = new ArrayList<>();

      for (JointDefinition jointDefinition : jointDefinitions)
      {
         List<URDFGazebo> jointURDFGazebos = toURDFGazebos(jointDefinition);
         if (jointURDFGazebos != null)
            urdfGazebos.addAll(jointURDFGazebos);
      }

      return urdfGazebos;
   }

   public static List<URDFGazebo> toURDFGazebos(JointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;
      List<URDFSensor> urdfSensors = toURDFSensors(jointDefinition.getSensorDefinitions());
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

   public static List<URDFSensor> toURDFSensors(List<SensorDefinition> sensorDefinitions)
   {
      if (sensorDefinitions == null || sensorDefinitions.isEmpty())
         return null;

      List<URDFSensor> urdfSensors = new ArrayList<>();

      for (SensorDefinition sensorDefinition : sensorDefinitions)
      {
         URDFSensor urdfSensor = toURDFSensor(sensorDefinition);
         if (urdfSensor != null)
            urdfSensors.add(urdfSensor);
      }

      return urdfSensors;
   }

   public static URDFSensor toURDFSensor(SensorDefinition sensorDefinition)
   {
      if (sensorDefinition == null)
         return null;

      if (sensorDefinition instanceof CameraSensorDefinition cameraSensorDefinition)
         return toURDFSensor(cameraSensorDefinition);
      if (sensorDefinition instanceof LidarSensorDefinition lidarSensorDefinition)
         return toURDFSensor(lidarSensorDefinition);
      if (sensorDefinition instanceof IMUSensorDefinition imuSensorDefinition)
         return toURDFSensor(imuSensorDefinition);
      if (sensorDefinition instanceof WrenchSensorDefinition wrenchSensorDefinition)
         return toURDFSensor(wrenchSensorDefinition);
      LogTools.warn("Unsupported sensor type: " + sensorDefinition);
      return null;
   }

   public static URDFSensor toURDFSensor(WrenchSensorDefinition sensorDefinition)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint()));
      urdfSensor.setUpdateRate(Double.toString(1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.force_torque);
      return urdfSensor;
   }

   public static URDFSensor toURDFSensor(IMUSensorDefinition sensorDefinition)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint()));
      urdfSensor.setUpdateRate(Double.toString(1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.imu);

      URDFIMU urdfIMU = new URDFIMU();

      URDFIMUNoise urdfIMUNoise = new URDFIMUNoise();
      urdfIMUNoise.setType(URDFIMUNoiseType.gaussian);

      { // Angular velocity noise
         URDFNoiseParameters urdfNoiseParameters = new URDFNoiseParameters();
         urdfNoiseParameters.setMean(sensorDefinition.getAngularVelocityNoiseMean());
         urdfNoiseParameters.setStddev(sensorDefinition.getAngularVelocityNoiseStandardDeviation());
         urdfNoiseParameters.setBias_mean(sensorDefinition.getAngularVelocityBiasMean());
         urdfNoiseParameters.setBias_stddev(sensorDefinition.getAngularVelocityBiasStandardDeviation());
         urdfIMUNoise.setRate(urdfNoiseParameters);
      }

      { // Acceleration noise
         URDFNoiseParameters urdfNoiseParameters = new URDFNoiseParameters();
         urdfNoiseParameters.setMean(sensorDefinition.getAccelerationNoiseMean());
         urdfNoiseParameters.setStddev(sensorDefinition.getAccelerationNoiseStandardDeviation());
         urdfNoiseParameters.setBias_mean(sensorDefinition.getAccelerationBiasMean());
         urdfNoiseParameters.setBias_stddev(sensorDefinition.getAccelerationBiasStandardDeviation());
         urdfIMUNoise.setAccel(urdfNoiseParameters);
      }

      urdfIMU.setNoise(urdfIMUNoise);

      return urdfSensor;
   }

   public static URDFSensor toURDFSensor(LidarSensorDefinition sensorDefinition)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint()));
      urdfSensor.setUpdateRate(Double.toString(1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.ray);
      urdfSensor.setRay(toURDFRay(sensorDefinition));
      return urdfSensor;
   }

   public static URDFRay toURDFRay(LidarSensorDefinition sensorDefinition)
   {
      if (sensorDefinition == null)
         return null;

      URDFRay urdfRay = new URDFRay();

      URDFRange urdfRange = new URDFRange();
      urdfRange.setMax(sensorDefinition.getMaxRange());
      urdfRange.setMin(sensorDefinition.getMinRange());
      urdfRange.setResolution(sensorDefinition.getRangeResolution());

      URDFScan urdfScan = new URDFScan();

      URDFHorizontalScan urdfHorizontalScan = new URDFHorizontalScan();
      urdfHorizontalScan.setMinAngle(sensorDefinition.getSweepYawMin());
      urdfHorizontalScan.setMaxAngle(sensorDefinition.getSweepYawMax());
      urdfHorizontalScan.setSamples(sensorDefinition.getPointsPerSweep());
      urdfScan.setHorizontal(urdfHorizontalScan);

      URDFVerticalScan urdfVerticalScan = new URDFVerticalScan();
      urdfVerticalScan.setMinAngle(sensorDefinition.getHeightPitchMin());
      urdfVerticalScan.setMaxAngle(sensorDefinition.getHeightPitchMax());
      urdfVerticalScan.setSamples(sensorDefinition.getScanHeight());
      urdfScan.setVertical(urdfVerticalScan);

      URDFNoise urdfNoise = new URDFNoise();
      urdfNoise.setType(URDFNoiseType.gaussian);
      urdfNoise.setMean(sensorDefinition.getGaussianNoiseMean());
      urdfNoise.setStddev(sensorDefinition.getGaussianNoiseStandardDeviation());
      urdfRay.setNoise(urdfNoise);

      urdfRay.setRange(urdfRange);

      return urdfRay;
   }

   public static URDFSensor toURDFSensor(CameraSensorDefinition sensorDefinition)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      String name = sensorDefinition.getName();
      if (name.contains("_"))
         urdfSensor.setName(name.substring(0, name.lastIndexOf("_")));
      else
         urdfSensor.setName(name);
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint()));
      urdfSensor.setUpdateRate(Double.toString(1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.camera);
      urdfSensor.setCamera(Collections.singletonList(toURDFCamera(sensorDefinition)));
      return urdfSensor;
   }

   public static URDFCamera toURDFCamera(CameraSensorDefinition sensorDefinition)
   {
      if (sensorDefinition == null)
         return null;

      URDFCamera urdfCamera = new URDFCamera();
      String name = sensorDefinition.getName();
      if (name != null && name.contains("_"))
         urdfCamera.setName(name.substring(name.lastIndexOf("_") + 1));
      urdfCamera.setPose(toPoseString(sensorDefinition.getTransformToJoint()));
      urdfCamera.setHorizontalFov(sensorDefinition.getFieldOfView());
      urdfCamera.setImage(toURDFSensorImage(sensorDefinition.getImageWidth(), sensorDefinition.getImageHeight()));
      urdfCamera.setClip(toURDFClip(sensorDefinition.getClipNear(), sensorDefinition.getClipFar()));
      return urdfCamera;
   }

   public static URDFSensorImage toURDFSensorImage(int width, int height)
   {
      return toURDFSensorImage(width, height, null);
   }

   public static URDFSensorImage toURDFSensorImage(int width, int height, String format)
   {
      URDFSensorImage urdfSensorImage = new URDFSensorImage();
      urdfSensorImage.setWidth(width);
      urdfSensorImage.setHeight(height);
      urdfSensorImage.setFormat(format);
      return urdfSensorImage;
   }

   public static URDFClip toURDFClip(double near, double far)
   {
      URDFClip urdfClip = new URDFClip();
      urdfClip.setNear(near);
      urdfClip.setFar(far);
      return urdfClip;
   }

   public static URDFOrigin toURDFOrigin(RigidBodyTransformReadOnly pose)
   {
      if (pose == null)
         return null;

      URDFOrigin urdfOrigin = new URDFOrigin();
      Tuple3DReadOnly translation = pose.getTranslation();
      urdfOrigin.setXYZ(EuclidCoreIOTools.getStringOf(" ", null, translation.getX(), translation.getY(), translation.getZ()));
      Orientation3DReadOnly rotation = pose.getRotation();
      urdfOrigin.setRPY(EuclidCoreIOTools.getStringOf(" ", null, rotation.getRoll(), rotation.getPitch(), rotation.getYaw()));
      return urdfOrigin;
   }

   public static URDFOrigin toURDFOrigin(AffineTransformReadOnly pose)
   {
      if (pose == null)
         return null;

      URDFOrigin urdfOrigin = new URDFOrigin();
      Tuple3DReadOnly translation = pose.getTranslation();
      urdfOrigin.setXYZ(EuclidCoreIOTools.getStringOf(" ", null, translation.getX(), translation.getY(), translation.getZ()));
      Orientation3DReadOnly rotation = pose.getLinearTransform().getAsQuaternion();
      urdfOrigin.setRPY(EuclidCoreIOTools.getStringOf(" ", null, rotation.getRoll(), rotation.getPitch(), rotation.getYaw()));
      if (!EuclidCoreTools.epsilonEquals(new Vector3D(1, 1, 1), pose.getLinearTransform().getScaleVector(), 1.0e-7))
         LogTools.warn("Discarding scale from affine trane transform.");
      return urdfOrigin;
   }

   public static String toPoseString(RigidBodyTransformReadOnly pose)
   {
      if (pose == null)
         return null;

      Tuple3DReadOnly translation = pose.getTranslation();
      Orientation3DReadOnly rotation = pose.getRotation();
      return EuclidCoreIOTools.getStringOf(" ",
                                           null,
                                           translation.getX(),
                                           translation.getY(),
                                           translation.getZ(),
                                           rotation.getRoll(),
                                           rotation.getPitch(),
                                           rotation.getYaw());
   }

   public static URDFAxis toURDFAxis(Tuple3DReadOnly axis)
   {
      if (axis == null)
         return null;

      URDFAxis urdfAxis = new URDFAxis();
      urdfAxis.setXYZ(EuclidCoreIOTools.getStringOf(" ", null, axis.getX(), axis.getY(), axis.getZ()));
      return urdfAxis;
   }

   public static URDFLimit toURDFLimit(OneDoFJointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      URDFLimit urdfLimit = new URDFLimit();
      urdfLimit.setLower(jointDefinition.getPositionLowerLimit());
      urdfLimit.setUpper(jointDefinition.getPositionUpperLimit());
      if (-jointDefinition.getVelocityLowerLimit() != jointDefinition.getVelocityUpperLimit())
      {
         LogTools.warn("Velocity limits no symmetric for joint {}, exporting smallest limit", jointDefinition.getName());
         urdfLimit.setVelocity(Math.min(Math.abs(jointDefinition.getVelocityLowerLimit()), jointDefinition.getVelocityUpperLimit()));
      }
      else
      {
         urdfLimit.setVelocity(jointDefinition.getVelocityUpperLimit());
      }

      if (-jointDefinition.getEffortLowerLimit() != jointDefinition.getEffortUpperLimit())
      {
         LogTools.warn("Effort limits no symmetric for joint {}, exporting smallest limit", jointDefinition.getName());
         urdfLimit.setEffort(Math.min(Math.abs(jointDefinition.getEffortLowerLimit()), jointDefinition.getEffortUpperLimit()));
      }
      else
      {
         urdfLimit.setEffort(jointDefinition.getEffortUpperLimit());
      }
      return urdfLimit;
   }

   public static URDFDynamics toURDFDynamics(OneDoFJointDefinition jointDefinition)
   {
      if (jointDefinition == null)
         return null;

      URDFDynamics urdfDynamics = new URDFDynamics();
      urdfDynamics.setFriction(jointDefinition.getStiction());
      urdfDynamics.setDamping(jointDefinition.getDamping());
      return urdfDynamics;
   }

   public static URDFLinkReference toURDFLinkReference(RigidBodyDefinition rigidBodyDefinition)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFLinkReference urdfLinkReference = new URDFLinkReference();
      urdfLinkReference.setLink(rigidBodyDefinition.getName());
      return urdfLinkReference;
   }
}
