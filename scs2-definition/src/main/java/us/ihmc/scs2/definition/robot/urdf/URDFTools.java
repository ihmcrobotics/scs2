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
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
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
import us.ihmc.scs2.definition.robot.urdf.items.URDFCollision;
import us.ihmc.scs2.definition.robot.urdf.items.URDFColor;
import us.ihmc.scs2.definition.robot.urdf.items.URDFDynamics;
import us.ihmc.scs2.definition.robot.urdf.items.URDFFilenameHolder;
import us.ihmc.scs2.definition.robot.urdf.items.URDFGazebo;
import us.ihmc.scs2.definition.robot.urdf.items.URDFGeometry;
import us.ihmc.scs2.definition.robot.urdf.items.URDFInertia;
import us.ihmc.scs2.definition.robot.urdf.items.URDFInertial;
import us.ihmc.scs2.definition.robot.urdf.items.URDFJoint;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLimit;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLink;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLinkReference;
import us.ihmc.scs2.definition.robot.urdf.items.URDFMass;
import us.ihmc.scs2.definition.robot.urdf.items.URDFMaterial;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.robot.urdf.items.URDFOrigin;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFCamera;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFIMU;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFIMU.URDFIMUNoise;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFIMU.URDFIMUNoise.URDFNoiseParameters;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFNoise;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFRange;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFScan.URDFHorizontalScan;
import us.ihmc.scs2.definition.robot.urdf.items.URDFSensor.URDFRay.URDFScan.URDFVerticalScan;
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
      switch (urdfJoint.getType())
      {
         case "continuous":
            return toRevoluteJointDefinition(urdfJoint, true);
         case "revolute":
            return toRevoluteJointDefinition(urdfJoint, false);
         case "prismatic":
            return toPrismaticJointDefinition(urdfJoint);
         case "fixed":
            return toFixedJointDefinition(urdfJoint);
         case "floating":
            return toSixDoFJointDefinition(urdfJoint);
         case "planar":
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

      switch (urdfSensor.getType())
      {
         case "camera":
         case "multicamera":
         case "depth":
            definitions.addAll(toCameraSensorDefinition(urdfSensor.getCamera()));
            break;
         case "imu":
            definitions.add(toIMUSensorDefinition(urdfSensor.getImu()));
            break;
         case "gpu_ray":
         case "ray":
            definitions.add(toLidarSensorDefinition(urdfSensor.getRay()));
            break;
         case "force_torque":
            definitions.add(new WrenchSensorDefinition());
            break;
         default:
            LogTools.error("Unsupport sensor type: " + urdfSensor.getType());
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
         if ("gaussian".equals(urdfNoise.getType()))
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
         if ("gaussian".equals(urdfNoise.getType()))
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
}
