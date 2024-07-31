package us.ihmc.scs2.definition.robot.urdf;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
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
import java.util.function.Supplier;
import java.util.stream.Collectors;

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
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.*;
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

   public static final URDFParserProperties DEFAULT_URDF_PARSER_PROPERTIES = new URDFParserProperties();
   public static final URDFGeneratorProperties DEFAULT_URDF_GENERATOR_PROPERTIES = new URDFGeneratorProperties();

   /**
    * Parse a {@link URDFModel} from the given URDF file.
    *
    * @param urdfFile the URDF file to be loaded.
    * @return the model.
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
    */
   public static URDFModel loadURDFModel(File urdfFile, Collection<String> resourceDirectories) throws JAXBException
   {
      return loadURDFModel(urdfFile, resourceDirectories, DEFAULT_URDF_PARSER_PROPERTIES);
   }

   /**
    * Parse a {@link URDFModel} from the given URDF file.
    *
    * @param urdfFile            the URDF file to be loaded.
    * @param resourceDirectories paths to resource directories. This allows to search for resources
    *                            that are not in the same directory as the {@code urdfFile}'s parent
    *                            directory. Paths can either be relative to the {@code urdfFile}'s
    *                            parent directory or absolute.
    * @param parserProperties    provides additional properties related to how the parsing show be
    *                            done.
    * @return the model.
    */
   public static URDFModel loadURDFModel(File urdfFile, Collection<String> resourceDirectories, URDFParserProperties parserProperties) throws JAXBException
   {
      try
      {
         // Internally, the unmarshaller does "new BufferedInputStream(new FileInputStream(urdfFile)" (see AbstractUnmarshallerImpl), no need to have 2 distinct implementations.
         return loadURDFModel(new BufferedInputStream(new FileInputStream(urdfFile)), resourceDirectories, null, parserProperties);
      }
      catch (FileNotFoundException e)
      {
         throw new IllegalArgumentException(e.getMessage());
      }
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
    */
   public static URDFModel loadURDFModel(InputStream inputStream, Collection<String> resourceDirectories, ClassLoader resourceClassLoader) throws JAXBException
   {
      return loadURDFModel(inputStream, resourceDirectories, resourceClassLoader, DEFAULT_URDF_PARSER_PROPERTIES);
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
    * @param parserProperties    provides additional properties related to how the parsing show be
    *                            done.
    * @return the model.
    */
   public static URDFModel loadURDFModel(InputStream inputStream,
                                         Collection<String> resourceDirectories,
                                         ClassLoader resourceClassLoader,
                                         URDFParserProperties parserProperties)
         throws JAXBException
   {
      try
      {
         Set<String> allResourceDirectories = new HashSet<>(resourceDirectories);
         URDFModel urdfModel;
         JAXBContext context = JAXBContext.newInstance(URDFModel.class);
         Unmarshaller um = context.createUnmarshaller();

         if (!parserProperties.ignoreNamespace)
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

         if (!parserProperties.linksToIgnore.isEmpty() && urdfModel.getLinks() != null)
            urdfModel.getLinks().removeIf(urdfLink -> parserProperties.linksToIgnore.contains(urdfLink.getName()));
         if (!parserProperties.jointsToIgnore.isEmpty() && urdfModel.getJoints() != null)
            urdfModel.getJoints().removeIf(urdfJoint -> parserProperties.jointsToIgnore.contains(urdfJoint.getName()));
         if (!parserProperties.parseSensors)
            urdfModel.getGazebos().removeIf(gazebo -> gazebo.getSensor() != null);

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
    * Converts the given URDF model into a {@code RobotDefinition}.
    * <p>
    * See {@link URDFParserProperties} for additional properties.
    * </p>
    *
    * @param urdfModel the URDF model to convert.
    * @return the robot definition which can be used to create a robot to be simulated in SCS2.
    */
   public static RobotDefinition toRobotDefinition(URDFModel urdfModel)
   {
      return toRobotDefinition(urdfModel, DEFAULT_URDF_PARSER_PROPERTIES);
   }

   /**
    * Converts the given URDF model into a {@code RobotDefinition}.
    *
    * @param urdfModel        the URDF model to convert.
    * @param parserProperties additional properties for tweaking the parsing operations such as
    *                         specifying the root joint.
    * @return the robot definition which can be used to create a robot to be simulated in SCS2.
    */
   public static RobotDefinition toRobotDefinition(URDFModel urdfModel, URDFParserProperties parserProperties)
   {
      List<URDFLink> urdfLinks = urdfModel.getLinks();
      List<URDFJoint> urdfJoints = urdfModel.getJoints();
      List<URDFGazebo> urdfGazebos = urdfModel.getGazebos();

      List<RigidBodyDefinition> rigidBodyDefinitions = new ArrayList<>();

      for (URDFLink urdfLink : urdfLinks)
      {
         rigidBodyDefinitions.add(toRigidBodyDefinition(urdfLink, parserProperties));
      }

      List<JointDefinition> jointDefinitions = new ArrayList<>();
      if (urdfJoints != null)
      {
         jointDefinitions = new ArrayList<>();
         for (URDFJoint urdfJoint : urdfJoints)
         {
            jointDefinitions.add(toJointDefinition(urdfJoint, parserProperties));
         }
      }

      RigidBodyDefinition startBodyDefinition = connectKinematics(rigidBodyDefinitions, jointDefinitions, urdfJoints);

      RigidBodyDefinition rootBodyDefinition;

      if (parserProperties.rootJointFactory != null)
      {
         JointDefinition rootJointDefinition = parserProperties.rootJointFactory.get();
         if (rootJointDefinition.getName() == null)
            rootJointDefinition.setName(startBodyDefinition.getName());
         rootJointDefinition.setSuccessor(startBodyDefinition);
         rootBodyDefinition = new RigidBodyDefinition("rootBody");
         rootBodyDefinition.addChildJoint(rootJointDefinition);
         jointDefinitions.add(rootJointDefinition); // This is required for sensors that are attached to the successor of the root joint.
      }
      else
      {
         rootBodyDefinition = startBodyDefinition;
      }

      addSensors(urdfGazebos, jointDefinitions, parserProperties);

      RobotDefinition robotDefinition = new RobotDefinition(urdfModel.getName());
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      if (parserProperties.simplifyKinematics)
         robotDefinition.simplifyKinematics();
      if (parserProperties.transformToZUp)
         robotDefinition.transformAllFramesToZUp();

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
   public static void addSensors(List<URDFGazebo> urdfGazebos, List<JointDefinition> jointDefinitions, URDFParserProperties parserProperties)
   {
      if (urdfGazebos == null || urdfGazebos.isEmpty())
         return;

      Map<String, JointDefinition> jointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(JointDefinition::getName, Function.identity()));
      Map<String, JointDefinition> linkNameToJointDefinitionMap = jointDefinitions.stream().collect(Collectors.toMap(joint -> joint.getSuccessor().getName(),
                                                                                                                     Function.identity()));

      for (URDFGazebo urdfGazebo : urdfGazebos)
      {
         if (urdfGazebo.getSensor() == null)
            continue;

         List<SensorDefinition> sensorDefinitions = toSensorDefinition(urdfGazebo.getSensor(), parserProperties);
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
      Map<String, RigidBodyDefinition> rigidBodyDefinitionMap = rigidBodyDefinitions.stream().collect(Collectors.toMap(RigidBodyDefinition::getName,
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
   public static RigidBodyDefinition toRigidBodyDefinition(URDFLink urdfLink, URDFParserProperties parserProperties)
   {
      RigidBodyDefinition definition = new RigidBodyDefinition(urdfLink.getName());

      URDFInertial urdfInertial = urdfLink.getInertial();

      if (urdfInertial == null)
      {
         definition.setMass(parseMass(null, parserProperties));
         definition.getMomentOfInertia().set(parseMomentOfInertia(null, parserProperties));
         definition.getInertiaPose().set(parseRigidBodyTransform(null, parserProperties));
      }
      else
      {
         definition.setMass(parseMass(urdfInertial.getMass(), parserProperties));
         definition.getMomentOfInertia().set(parseMomentOfInertia(urdfInertial.getInertia(), parserProperties));
         definition.getInertiaPose().set(parseRigidBodyTransform(urdfInertial.getOrigin(), parserProperties));
      }

      if (urdfLink.getVisual() != null)
      {
         List<URDFVisual> urdfVisuals = urdfLink.getVisual();

         for (int i = 0; i < urdfVisuals.size(); i++)
         {
            URDFVisual urdfVisual = urdfVisuals.get(i);
            VisualDefinition visual = toVisualDefinition(urdfVisual, parserProperties);
            if (visual == null)
               continue;
            if (parserProperties.autoGenerateVisualName && visual.getName() == null)
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
            CollisionShapeDefinition collision = toCollisionShapeDefinition(urdfCollision, parserProperties);
            if (collision == null)
               continue;
            if (parserProperties.autoGenerateCollisionName && collision.getName() == null)
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
    * @see #addSensors(List, List, URDFParserProperties)
    * @see #connectKinematics(List, List, List)
    */
   public static JointDefinition toJointDefinition(URDFJoint urdfJoint, URDFParserProperties parserProperties)
   {
      URDFJointType type = URDFJointType.parse(urdfJoint.getType());
      if (type == null)
         throw new RuntimeException("Unexpected value for the joint type: " + urdfJoint.getType());

      return switch (type)
      {
         case continuous -> toRevoluteJointDefinition(urdfJoint, true, parserProperties);
         case revolute -> toRevoluteJointDefinition(urdfJoint, false, parserProperties);
         case prismatic -> toPrismaticJointDefinition(urdfJoint, parserProperties);
         case fixed -> toFixedJointDefinition(urdfJoint, parserProperties);
         case floating -> toSixDoFJointDefinition(urdfJoint, parserProperties);
         case planar -> toPlanarJointDefinition(urdfJoint, parserProperties);
         case cross_four_bar -> toCrossFourBarJointDefinition(urdfJoint, parserProperties);
         case revolute_twins -> toRevoluteTwinsJointDefinition(urdfJoint, parserProperties);
      };
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toJointDefinition(URDFJoint, URDFParserProperties)
    */
   public static RevoluteJointDefinition toRevoluteJointDefinition(URDFJoint urdfJoint, boolean ignorePositionLimits, URDFParserProperties parserProperties)
   {
      RevoluteJointDefinition definition = new RevoluteJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin(), parserProperties));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis(), parserProperties));
      parseLimit(urdfJoint.getLimit(), definition, ignorePositionLimits, parserProperties);
      parseDynamics(urdfJoint.getDynamics(), definition, parserProperties);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toJointDefinition(URDFJoint, URDFParserProperties)
    */
   public static PrismaticJointDefinition toPrismaticJointDefinition(URDFJoint urdfJoint, URDFParserProperties parserProperties)
   {
      PrismaticJointDefinition definition = new PrismaticJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin(), parserProperties));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis(), parserProperties));
      parseLimit(urdfJoint.getLimit(), definition, false, parserProperties);
      parseDynamics(urdfJoint.getDynamics(), definition, parserProperties);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toJointDefinition(URDFJoint, URDFParserProperties)
    */
   public static FixedJointDefinition toFixedJointDefinition(URDFJoint urdfJoint, URDFParserProperties parserProperties)
   {
      FixedJointDefinition definition = new FixedJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin(), parserProperties));

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toJointDefinition(URDFJoint, URDFParserProperties)
    */
   public static SixDoFJointDefinition toSixDoFJointDefinition(URDFJoint urdfJoint, URDFParserProperties parserProperties)
   {
      SixDoFJointDefinition definition = new SixDoFJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin(), parserProperties));

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toJointDefinition(URDFJoint, URDFParserProperties)
    */
   public static PlanarJointDefinition toPlanarJointDefinition(URDFJoint urdfJoint, URDFParserProperties parserProperties)
   {
      PlanarJointDefinition definition = new PlanarJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin(), parserProperties));

      Vector3D surfaceNormal = parseAxis(urdfJoint.getAxis(), parserProperties);

      if (!surfaceNormal.geometricallyEquals(Axis3D.Y, 1.0e-5))
         throw new UnsupportedOperationException("Planar joint are supported only with a surface normal equal to: "
               + EuclidCoreIOTools.getTuple3DString(Axis3D.Y) + ", received:" + surfaceNormal);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toJointDefinition(URDFJoint, URDFParserProperties)
    */
   public static CrossFourBarJointDefinition toCrossFourBarJointDefinition(URDFJoint urdfJoint, URDFParserProperties parserProperties)
   {
      CrossFourBarJointDefinition definition = new CrossFourBarJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin(), parserProperties));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis(), parserProperties));
      parseLimit(urdfJoint.getLimit(), definition, false, parserProperties);
      parseDynamics(urdfJoint.getDynamics(), definition, parserProperties);

      if (urdfJoint.getSubJoints() == null || urdfJoint.getSubJoints().size() != 4)
         throw new IllegalArgumentException("Cross four bar joint requires 4 sub-joints.");
      if (urdfJoint.getSubLinks() == null || urdfJoint.getSubLinks().size() != 2)
         throw new IllegalArgumentException("Cross four bar joint requires 2 sub-links.");

      for (URDFJoint subJoint : urdfJoint.getSubJoints())
      {
         if (URDFJointType.parse(subJoint.getType()) != URDFJointType.revolute)
            throw new IllegalArgumentException("Cross four bar joint requires all sub-joints to be revolute.");
      }

      URDFJoint urdfJoint0 = urdfJoint.getSubJoints().get(0);
      URDFJoint urdfJoint1 = urdfJoint.getSubJoints().get(1);
      URDFJoint urdfJoint2 = urdfJoint.getSubJoints().get(2);
      URDFJoint urdfJoint3 = urdfJoint.getSubJoints().get(3);

      if (!Objects.equals(urdfJoint0.getParent().getLink(), urdfJoint.getParent().getLink())
            || !Objects.equals(urdfJoint1.getParent().getLink(), urdfJoint.getParent().getLink()))
      {
         throw new IllegalArgumentException("The 2 first sub-joints of the cross four bar joint must closest to the robot root and share the same parent link as the cross four bar joint.");
      }
      if (!Objects.equals(urdfJoint2.getChild().getLink(), urdfJoint.getChild().getLink())
            || !Objects.equals(urdfJoint3.getChild().getLink(), urdfJoint.getChild().getLink()))
      {
         throw new IllegalArgumentException("The 2 last sub-joints of the cross four bar joint must farthest from the robot root and share the same child link as the cross four bar joint.");
      }

      URDFLink urdfLink0 = urdfJoint.getSubLinks().get(0);
      URDFLink urdfLink1 = urdfJoint.getSubLinks().get(1);

      int actuatedJointIndex = parseInteger(urdfJoint.getActuatedJointIndex(), -1);
      if (actuatedJointIndex < 0 || actuatedJointIndex > 3)
         throw new IllegalArgumentException("The actuated joint index must be in [0, 3], was: " + actuatedJointIndex);

      URDFJoint urdfJointA, urdfJointB, urdfJointC, urdfJointD;
      URDFLink urdfLinkDA, urdfLinkBC;
      urdfJointA = urdfJoint0;
      urdfJointB = urdfJoint1;

      if (Objects.equals(urdfJointA.getChild().getLink(), urdfLink0.getName()))
      {
         if (!Objects.equals(urdfJointB.getChild().getLink(), urdfLink1.getName()))
            throw new IllegalArgumentException("Error when parsing the cross-bars, jointA child: " + urdfJointA.getChild().getLink() + ", jointB child: "
                  + urdfJointB.getChild().getLink() + ", link0: " + urdfLink0.getName() + ", link1: " + urdfLink1.getName());
         urdfLinkDA = urdfLink0;
         urdfLinkBC = urdfLink1;
      }
      else
      {
         if (!Objects.equals(urdfJointB.getChild().getLink(), urdfLink0.getName()))
            throw new IllegalArgumentException("Error when parsing the cross-bars, jointA child: " + urdfJointA.getChild().getLink() + ", jointB child: "
                  + urdfJointB.getChild().getLink() + ", link0: " + urdfLink0.getName() + ", link1: " + urdfLink1.getName());
         urdfLinkDA = urdfLink1;
         urdfLinkBC = urdfLink0;
      }

      if (Objects.equals(urdfJoint2.getParent().getLink(), urdfLinkDA.getName()))
      {
         if (!Objects.equals(urdfJoint3.getParent().getLink(), urdfLinkBC.getName()))
            throw new IllegalArgumentException("Error when parsing the cross-bars, joint2 parent: " + urdfJoint2.getParent().getLink() + ", joint3 parent: "
                  + urdfJoint3.getParent().getLink() + ", linkDA: " + urdfLinkDA.getName() + ", linkBC: " + urdfLinkBC.getName());
         urdfJointD = urdfJoint2;
         urdfJointC = urdfJoint3;
         if (actuatedJointIndex == 2)
            actuatedJointIndex = 3;
         else if (actuatedJointIndex == 3)
            actuatedJointIndex = 2;
      }
      else
      {
         if (!Objects.equals(urdfJoint2.getParent().getLink(), urdfLinkBC.getName()))
            throw new IllegalArgumentException("Error when parsing the cross-bars, joint2 parent: " + urdfJoint2.getParent().getLink() + ", joint3 parent: "
                  + urdfJoint3.getParent().getLink() + ", linkDA: " + urdfLinkDA.getName() + ", linkBC: " + urdfLinkBC.getName());
         if (!Objects.equals(urdfJoint3.getParent().getLink(), urdfLinkDA.getName()))
            throw new IllegalArgumentException("Error when parsing the cross-bars, joint2 parent: " + urdfJoint2.getParent().getLink() + ", joint3 parent: "
                  + urdfJoint3.getParent().getLink() + ", linkDA: " + urdfLinkDA.getName() + ", linkBC: " + urdfLinkBC.getName());
         urdfJointC = urdfJoint2;
         urdfJointD = urdfJoint3;
      }

      definition.setJointNameA(urdfJointA.getName());
      definition.setJointNameB(urdfJointB.getName());
      definition.setJointNameC(urdfJointC.getName());
      definition.setJointNameD(urdfJointD.getName());
      definition.setBodyDA(toRigidBodyDefinition(urdfLinkDA, parserProperties));
      definition.setBodyBC(toRigidBodyDefinition(urdfLinkBC, parserProperties));
      definition.setActuatedJointIndex(actuatedJointIndex);
      definition.setTransformAToPredecessor(parseRigidBodyTransform(urdfJointA.getOrigin(), parserProperties));
      definition.setTransformBToPredecessor(parseRigidBodyTransform(urdfJointB.getOrigin(), parserProperties));
      definition.setTransformCToB(parseRigidBodyTransform(urdfJointC.getOrigin(), parserProperties));
      definition.setTransformDToA(parseRigidBodyTransform(urdfJointD.getOrigin(), parserProperties));

      int loopClosureJointIndex = switch (actuatedJointIndex)
      {
         case 0 -> 3;
         case 1 -> 2;
         case 2 -> 1;
         case 3 -> 0;
         default -> -1;
      };
      definition.setLoopClosureJointIndex(loopClosureJointIndex);

      return definition;
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toJointDefinition(URDFJoint, URDFParserProperties)
    */
   public static RevoluteTwinsJointDefinition toRevoluteTwinsJointDefinition(URDFJoint urdfJoint, URDFParserProperties parserProperties)
   {
      RevoluteTwinsJointDefinition definition = new RevoluteTwinsJointDefinition(urdfJoint.getName());

      definition.getTransformToParent().set(parseRigidBodyTransform(urdfJoint.getOrigin(), parserProperties));
      definition.getAxis().set(parseAxis(urdfJoint.getAxis(), parserProperties));
      parseLimit(urdfJoint.getLimit(), definition, false, parserProperties);
      parseDynamics(urdfJoint.getDynamics(), definition, parserProperties);

      if (urdfJoint.getSubJoints() == null || urdfJoint.getSubJoints().size() != 2)
         throw new IllegalArgumentException("Revolute twins joint requires 2 sub-joints.");
      if (urdfJoint.getSubLinks() == null || urdfJoint.getSubLinks().size() != 1)
         throw new IllegalArgumentException("Revolute twins joint requires 1 sub-links.");

      for (URDFJoint subJoint : urdfJoint.getSubJoints())
      {
         if (URDFJointType.parse(subJoint.getType()) != URDFJointType.revolute)
            throw new IllegalArgumentException("Revolute twins joint requires all sub-joints to be revolute.");
      }

      URDFJoint urdfJointA = urdfJoint.getSubJoints().get(0);
      URDFJoint urdfJointB = urdfJoint.getSubJoints().get(1);

      if (!Objects.equals(urdfJointA.getParent().getLink(), urdfJoint.getParent().getLink()))
         throw new IllegalArgumentException("The first sub-joint of the revolute twins joint must share the same parent link as the revolute twins joint.");
      if (!Objects.equals(urdfJointB.getChild().getLink(), urdfJoint.getChild().getLink()))
         throw new IllegalArgumentException("The second sub-joint of the revolute twins joint must share the same child link as the revolute twins joint.");

      URDFLink urdfLinkAB = urdfJoint.getSubLinks().get(0);

      if (!Objects.equals(urdfJointA.getChild().getLink(), urdfLinkAB.getName()))
         throw new IllegalArgumentException("Error when parsing the revolute twins joint.");
      if (!Objects.equals(urdfJointB.getParent().getLink(), urdfLinkAB.getName()))
         throw new IllegalArgumentException("Error when parsing the revolute twins joint.");

      int actuatedJointIndex = parseInteger(urdfJoint.getActuatedJointIndex(), -1);
      if (actuatedJointIndex < 0 || actuatedJointIndex > 1)
         throw new IllegalArgumentException("The actuated joint index must be in [0, 1].");

      URDFJoint constrainedJoint = actuatedJointIndex == 0 ? urdfJointB : urdfJointA;
      if (constrainedJoint.getMimic() == null)
         throw new IllegalArgumentException("The constrained sub-joint of the revolute twins joint must have a mimic.");
      Objects.requireNonNull(constrainedJoint.getMimic().getMultiplier(),
                             "The constrained sub-joint of the revolute twins joint must have a mimic multiplier.");
      Objects.requireNonNull(constrainedJoint.getMimic().getOffset(), "The constrained sub-joint of the revolute twins joint must have a mimic offset.");
      double constraintRatio = parseDouble(constrainedJoint.getMimic().getMultiplier(), Double.NaN);
      double constraintOffset = parseDouble(constrainedJoint.getMimic().getOffset(), Double.NaN);

      definition.setJointNameA(urdfJointA.getName());
      definition.setJointNameB(urdfJointB.getName());
      definition.setBodyAB(toRigidBodyDefinition(urdfLinkAB, parserProperties));
      definition.setTransformAToPredecessor(parseRigidBodyTransform(urdfJointA.getOrigin(), parserProperties));
      definition.setTransformBToA(parseRigidBodyTransform(urdfJointB.getOrigin(), parserProperties));
      definition.setActuatedJointIndex(actuatedJointIndex);
      definition.setConstraintRatio(constraintRatio);
      definition.setConstraintOffset(constraintOffset);

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
   public static List<SensorDefinition> toSensorDefinition(URDFSensor urdfSensor, URDFParserProperties parserProperties)
   {
      List<SensorDefinition> definitions = new ArrayList<>();

      URDFSensorType type = URDFSensorType.parse(urdfSensor.getType());

      if (type == null)
      {
         LogTools.error("Unsupported sensor type: " + urdfSensor.getType());
         return null;
      }

      switch (type)
      {
         case camera:
         case multicamera:
         case depth:
            definitions.addAll(toCameraSensorDefinition(urdfSensor.getCamera(), parserProperties));
            break;
         case imu:
            definitions.add(toIMUSensorDefinition(urdfSensor.getImu(), parserProperties));
            break;
         case gpu_ray:
         case ray:
            definitions.add(toLidarSensorDefinition(urdfSensor.getRay(), parserProperties));
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
         definition.getTransformToJoint().preMultiply(parsePose(urdfSensor.getPose(), parserProperties));
         definition.setUpdatePeriod(updatePeriod);
      }

      return definitions;
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toSensorDefinition(URDFSensor, URDFParserProperties)
    */
   public static List<CameraSensorDefinition> toCameraSensorDefinition(List<URDFCamera> urdfCameras, URDFParserProperties parserProperties)
   {
      if (urdfCameras == null)
         return Collections.emptyList();
      return urdfCameras.stream().map(urdfCamera -> toCameraSensorDefinition(urdfCamera, parserProperties)).collect(Collectors.toList());
   }

   /**
    * <i>-- Intended for internal use --</i>
    *
    * @see #toSensorDefinition(URDFSensor, URDFParserProperties)
    */
   public static CameraSensorDefinition toCameraSensorDefinition(URDFCamera urdfCamera, URDFParserProperties parserProperties)
   {
      CameraSensorDefinition definition = new CameraSensorDefinition();
      definition.setName(urdfCamera.getName());
      definition.getTransformToJoint().set(parsePose(urdfCamera.getPose(), parserProperties));
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
    * @see #toSensorDefinition(URDFSensor, URDFParserProperties)
    */
   public static LidarSensorDefinition toLidarSensorDefinition(URDFRay urdfRay, URDFParserProperties parserProperties)
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

      definition.getTransformToJoint().set(parsePose(urdfRay.getPose(), parserProperties));
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
    * @see #toSensorDefinition(URDFSensor, URDFParserProperties)
    */
   public static IMUSensorDefinition toIMUSensorDefinition(URDFIMU urdfIMU, URDFParserProperties parserProperties)
   {
      IMUSensorDefinition definition = new IMUSensorDefinition();

      if (urdfIMU != null)
      {
         URDFIMUNoise urdfNoise = urdfIMU.getNoise();
         if (urdfNoise != null)
         {
            if (URDFIMUNoiseType.gaussian.equals(URDFIMUNoiseType.parse(urdfNoise.getType())))
            {
               URDFNoiseParameters accelerationNoise = urdfNoise.getAccel();
               URDFNoiseParameters angularVelocityNoise = urdfNoise.getRate();

               definition.setAccelerationNoiseParameters(parseDouble(accelerationNoise.getMean(), 0.0), parseDouble(accelerationNoise.getStddev(), 0.0));
               definition.setAccelerationBiasParameters(parseDouble(accelerationNoise.getBias_mean(), 0.0),
                                                        parseDouble(accelerationNoise.getBias_stddev(), 0.0));

               definition.setAngularVelocityNoiseParameters(parseDouble(angularVelocityNoise.getMean(), 0.0),
                                                            parseDouble(angularVelocityNoise.getStddev(), 0.0));
               definition.setAngularVelocityBiasParameters(parseDouble(angularVelocityNoise.getBias_mean(), 0.0),
                                                           parseDouble(angularVelocityNoise.getBias_stddev(), 0.0));
            }
            else
            {
               LogTools.error("Unknown IMU noise model: {}.", urdfNoise.getType());
            }
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
   public static VisualDefinition toVisualDefinition(URDFVisual urdfVisual, URDFParserProperties parserProperties)
   {
      if (urdfVisual == null)
         return null;

      VisualDefinition visualDefinition = new VisualDefinition();
      visualDefinition.setName(urdfVisual.getName());
      visualDefinition.setOriginPose(parseRigidBodyTransform(urdfVisual.getOrigin(), parserProperties));
      visualDefinition.setMaterialDefinition(toMaterialDefinition(urdfVisual.getMaterial(), parserProperties));
      visualDefinition.setGeometryDefinition(toGeometryDefinition(urdfVisual.getGeometry(), parserProperties));
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
   public static CollisionShapeDefinition toCollisionShapeDefinition(URDFCollision urdfCollision, URDFParserProperties parserProperties)
   {
      if (urdfCollision == null)
         return null;

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      collisionShapeDefinition.setName(urdfCollision.getName());
      collisionShapeDefinition.setOriginPose(parseRigidBodyTransform(urdfCollision.getOrigin(), parserProperties));
      collisionShapeDefinition.setGeometryDefinition(toGeometryDefinition(urdfCollision.getGeometry(), parserProperties));
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
   public static GeometryDefinition toGeometryDefinition(URDFGeometry urdfGeometry, URDFParserProperties parserProperties)
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
   public static MaterialDefinition toMaterialDefinition(URDFMaterial urdfMaterial, URDFParserProperties parserProperties)
   {
      if (urdfMaterial == null)
         return null;

      MaterialDefinition materialDefinition = new MaterialDefinition();
      materialDefinition.setName(urdfMaterial.getName());
      materialDefinition.setDiffuseColor(toColorDefinition(urdfMaterial.getColor(), parserProperties));
      materialDefinition.setDiffuseMap(toTextureDefinition(urdfMaterial.getTexture(), parserProperties));
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
   public static TextureDefinition toTextureDefinition(URDFTexture urdfTexture, URDFParserProperties parserProperties)
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
   public static ColorDefinition toColorDefinition(URDFColor urdfColor, URDFParserProperties parserProperties)
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

   public static RigidBodyTransform parsePose(String pose, URDFParserProperties parserProperties)
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

   public static YawPitchRollTransformDefinition parseRigidBodyTransform(URDFOrigin origin, URDFParserProperties parserProperties)
   {
      if (origin == null)
         origin = new URDFOrigin();

      YawPitchRollTransformDefinition rigidBodyTransform = new YawPitchRollTransformDefinition();
      rigidBodyTransform.getTranslation().set(parseVector3D(origin.getXYZ(), DEFAULT_ORIGIN_XYZ));
      rigidBodyTransform.getRotation().setEuler(parseVector3D(origin.getRPY(), DEFAULT_ORIGIN_RPY));
      return rigidBodyTransform;
   }

   public static Matrix3D parseMomentOfInertia(URDFInertia inertia, URDFParserProperties parserProperties)
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

   public static double parseMass(URDFMass urdfMass, URDFParserProperties parserProperties)
   {
      if (urdfMass == null)
         return DEFAULT_MASS;
      return parseDouble(urdfMass.getValue(), DEFAULT_MASS);
   }

   public static void parseLimit(URDFLimit urdfLimit,
                                 OneDoFJointDefinition jointDefinitionToParseLimitInto,
                                 boolean ignorePositionLimits,
                                 URDFParserProperties parserProperties)
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

   public static void parseDynamics(URDFDynamics urdfDynamics, OneDoFJointDefinition jointDefinitionToParseDynamicsInto, URDFParserProperties parserProperties)
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

   public static Vector3D parseAxis(URDFAxis axis, URDFParserProperties parserProperties)
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
      return toURDFModel(robotDefinition, DEFAULT_URDF_GENERATOR_PROPERTIES);
   }

   public static URDFModel toURDFModel(RobotDefinition robotDefinition, URDFGeneratorProperties properties)
   {
      URDFModel urdfModel = new URDFModel();

      List<JointDefinition> allJointsMinusRootJoint = new ArrayList<>(robotDefinition.getAllJoints());
      for (int i = 0; i < allJointsMinusRootJoint.size(); i++)
      {
         if(allJointsMinusRootJoint.get(i).equals(robotDefinition.getFloatingRootJointDefinition()))
            allJointsMinusRootJoint.remove(i);
      }

      urdfModel.setName(robotDefinition.getName());
      urdfModel.setLinks(toURDFLinks(RobotDefinition.collectSubtreeRigidBodyDefinitions(robotDefinition.getFloatingRootJointDefinition().getSuccessor()),
                                     properties));
      urdfModel.setJoints(toURDFJoints(allJointsMinusRootJoint, properties));
      urdfModel.setGazebos(toURDFGazebos(robotDefinition.getAllJoints(), properties));

      return urdfModel;
   }

   public static List<URDFLink> toURDFLinks(List<RigidBodyDefinition> rigidBodyDefinitions, URDFGeneratorProperties properties)
   {
      if (rigidBodyDefinitions == null || rigidBodyDefinitions.isEmpty())
         return null;

      List<URDFLink> urdfLinks = new ArrayList<>();

      for (RigidBodyDefinition rigidBodyDefinition : rigidBodyDefinitions)
      {
         URDFLink urdfLink = toURDFLink(rigidBodyDefinition, properties);
         if (urdfLink != null)
            urdfLinks.add(urdfLink);
      }

      return urdfLinks;
   }

   public static URDFLink toURDFLink(RigidBodyDefinition rigidBodyDefinition, URDFGeneratorProperties properties)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFLink urdfLink = new URDFLink();
      urdfLink.setName(rigidBodyDefinition.getName());
      urdfLink.setInertial(toURDFInterial(rigidBodyDefinition, properties));
      urdfLink.setVisual(toURDFVisuals(rigidBodyDefinition.getVisualDefinitions(), properties));
      urdfLink.setCollision(toURDFCollisions(rigidBodyDefinition.getCollisionShapeDefinitions(), properties));
      return urdfLink;
   }

   public static URDFInertial toURDFInterial(RigidBodyDefinition rigidBodyDefinition, URDFGeneratorProperties properties)
   {
      if (rigidBodyDefinition == null)
         return null;

      if (!properties.alwaysExportLinkInertial)
      {
         if (rigidBodyDefinition.getMass() == 0.0 && rigidBodyDefinition.getMomentOfInertia().isZero(0.0))
            return null;
      }

      URDFInertial urdfInertial = new URDFInertial();
      urdfInertial.setOrigin(toURDFOrigin(rigidBodyDefinition.getInertiaPose(), properties));
      urdfInertial.setMass(toURDFMass(rigidBodyDefinition.getMass(), properties));
      urdfInertial.setInertia(toURDFInertia(rigidBodyDefinition.getMomentOfInertia(), properties));
      return urdfInertial;
   }

   public static URDFMass toURDFMass(double mass, URDFGeneratorProperties properties)
   {
      URDFMass urdfMass = new URDFMass();
      urdfMass.setValue(properties.toString(URDFMass.class, "value", mass));
      return urdfMass;
   }

   public static URDFInertia toURDFInertia(MomentOfInertiaDefinition momentOfInertiaDefinition, URDFGeneratorProperties properties)
   {
      if (momentOfInertiaDefinition == null)
         return null;

      URDFInertia urdfInertia = new URDFInertia();
      urdfInertia.setIxx(properties.toString(URDFInertia.class, "ixx", momentOfInertiaDefinition.getIxx()));
      urdfInertia.setIyy(properties.toString(URDFInertia.class, "iyy", momentOfInertiaDefinition.getIyy()));
      urdfInertia.setIzz(properties.toString(URDFInertia.class, "izz", momentOfInertiaDefinition.getIzz()));
      urdfInertia.setIxy(properties.toString(URDFInertia.class, "ixy", momentOfInertiaDefinition.getIxy()));
      urdfInertia.setIxz(properties.toString(URDFInertia.class, "ixz", momentOfInertiaDefinition.getIxz()));
      urdfInertia.setIyz(properties.toString(URDFInertia.class, "iyz", momentOfInertiaDefinition.getIyz()));
      return urdfInertia;
   }

   public static List<URDFVisual> toURDFVisuals(List<VisualDefinition> visualDefinitions, URDFGeneratorProperties properties)
   {
      if (visualDefinitions == null || visualDefinitions.isEmpty())
         return null;

      List<URDFVisual> urdfVisuals = new ArrayList<>();

      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         URDFVisual urdfVisual = toURDFVisual(visualDefinition, properties);
         if (urdfVisual != null)
            urdfVisuals.add(urdfVisual);
      }

      return urdfVisuals;
   }

   public static URDFVisual toURDFVisual(VisualDefinition visualDefinition, URDFGeneratorProperties properties)
   {
      if (visualDefinition == null)
         return null;

      URDFVisual urdfVisual = new URDFVisual();
      urdfVisual.setName(visualDefinition.getName());
      urdfVisual.setOrigin(toURDFOrigin(visualDefinition.getOriginPose(), properties));
      urdfVisual.setGeometry(toURDFGeometry(visualDefinition.getGeometryDefinition(), properties));
      urdfVisual.setMaterial(toURDFMaterial(visualDefinition.getMaterialDefinition(), properties));
      return urdfVisual;
   }

   public static List<URDFCollision> toURDFCollisions(List<CollisionShapeDefinition> collisionShapeDefinitions, URDFGeneratorProperties properties)
   {
      if (collisionShapeDefinitions == null || collisionShapeDefinitions.isEmpty())
         return null;

      List<URDFCollision> urdfCollisions = new ArrayList<>();

      for (CollisionShapeDefinition collisionShapeDefinition : collisionShapeDefinitions)
      {
         URDFCollision urdfCollision = toURDFCollision(collisionShapeDefinition, properties);
         if (urdfCollision != null)
            urdfCollisions.add(urdfCollision);
      }

      return urdfCollisions;
   }

   public static URDFCollision toURDFCollision(CollisionShapeDefinition collisionShapeDefinition, URDFGeneratorProperties properties)
   {
      if (collisionShapeDefinition == null)
         return null;

      URDFCollision urdfCollision = new URDFCollision();
      urdfCollision.setName(collisionShapeDefinition.getName());
      urdfCollision.setOrigin(toURDFOrigin(collisionShapeDefinition.getOriginPose(), properties));
      urdfCollision.setGeometry(toURDFGeometry(collisionShapeDefinition.getGeometryDefinition(), properties));
      return urdfCollision;
   }

   public static URDFGeometry toURDFGeometry(GeometryDefinition geometryDefinition, URDFGeneratorProperties properties)
   {
      if (geometryDefinition == null)
         return null;

      URDFGeometry urdfGeometry = new URDFGeometry();

      if (geometryDefinition instanceof Box3DDefinition box3DGeometry)
      {
         urdfGeometry.setBox(toURDFBox(box3DGeometry, properties));
      }
      else if (geometryDefinition instanceof Cylinder3DDefinition cylinder3DDefinition)
      {
         urdfGeometry.setCylinder(toURDFCylinder(cylinder3DDefinition, properties));
      }
      else if (geometryDefinition instanceof Sphere3DDefinition sphere3DDefinition)
      {
         urdfGeometry.setSphere(toURDFSphere(sphere3DDefinition, properties));
      }
      else if (geometryDefinition instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
      {
         urdfGeometry.setMesh(toURDFMesh(modelFileGeometryDefinition, properties));
      }
      else
      {
         LogTools.warn("Unhandled geometry: {}", geometryDefinition);
      }

      return urdfGeometry;
   }

   public static URDFBox toURDFBox(Box3DDefinition box3DDefinition, URDFGeneratorProperties properties)
   {
      if (box3DDefinition == null)
         return null;

      URDFBox urdfBox = new URDFBox();
      urdfBox.setSize(properties.toString(URDFBox.class, "size", box3DDefinition.getSizeX(), box3DDefinition.getSizeY(), box3DDefinition.getSizeZ()));
      return urdfBox;
   }

   public static URDFCylinder toURDFCylinder(Cylinder3DDefinition cylinder3DDefinition, URDFGeneratorProperties properties)
   {
      if (cylinder3DDefinition == null)
         return null;

      URDFCylinder urdfCylinder = new URDFCylinder();
      urdfCylinder.setRadius(properties.toString(URDFCylinder.class, "radius", cylinder3DDefinition.getRadius()));
      urdfCylinder.setLength(properties.toString(URDFCylinder.class, "length", cylinder3DDefinition.getLength()));
      return urdfCylinder;
   }

   public static URDFSphere toURDFSphere(Sphere3DDefinition sphere3DDefinition, URDFGeneratorProperties properties)
   {
      if (sphere3DDefinition == null)
         return null;

      URDFSphere urdfSphere = new URDFSphere();
      urdfSphere.setRadius(properties.toString(URDFSphere.class, "radius", sphere3DDefinition.getRadius()));
      return urdfSphere;
   }

   public static URDFMesh toURDFMesh(ModelFileGeometryDefinition modelFileGeometryDefinition, URDFGeneratorProperties properties)
   {
      if (modelFileGeometryDefinition == null)
         return null;

      URDFMesh urdfMesh = new URDFMesh();
      urdfMesh.setFilename(modelFileGeometryDefinition.getFileName());
      Vector3D scale = modelFileGeometryDefinition.getScale();
      if (scale != null && !scale.equals(new Vector3D(1, 1, 1)))
      {
         urdfMesh.setScale(properties.toString(URDFMesh.class, "scale", scale.getX(), scale.getY(), scale.getZ()));
      }
      return urdfMesh;
   }

   public static URDFMaterial toURDFMaterial(MaterialDefinition materialDefinition, URDFGeneratorProperties properties)
   {
      if (materialDefinition == null)
         return null;

      URDFMaterial urdfMaterial = new URDFMaterial();
      urdfMaterial.setName(materialDefinition.getName());
      urdfMaterial.setColor(toURDFColor(materialDefinition.getDiffuseColor(), properties));
      urdfMaterial.setTexture(toURDFTexture(materialDefinition.getDiffuseMap(), properties));
      return urdfMaterial;
   }

   public static URDFColor toURDFColor(ColorDefinition colorDefinition, URDFGeneratorProperties properties)
   {
      if (colorDefinition == null)
         return null;

      URDFColor urdfColor = new URDFColor();
      urdfColor.setRGBA(properties.toString(URDFColor.class,
                                            "rgba",
                                            colorDefinition.getRed(),
                                            colorDefinition.getGreen(),
                                            colorDefinition.getBlue(),
                                            colorDefinition.getAlpha()));
      return urdfColor;
   }

   public static URDFTexture toURDFTexture(TextureDefinition diffuseMap, URDFGeneratorProperties properties)
   {
      if (diffuseMap == null)
         return null;

      URDFTexture urdfTexture = new URDFTexture();
      urdfTexture.setFilename(diffuseMap.getFilename());
      return urdfTexture;
   }

   public static List<URDFJoint> toURDFJoints(List<JointDefinition> jointDefinitions, URDFGeneratorProperties properties)
   {
      if (jointDefinitions == null || jointDefinitions.isEmpty())
         return null;

      List<URDFJoint> urdfJoints = new ArrayList<>();

      for (JointDefinition jointDefinition : jointDefinitions)
      {
         URDFJoint urdfJoint = toURDFJoint(jointDefinition, properties);
         if (urdfJoint != null)
            urdfJoints.add(urdfJoint);
      }

      return urdfJoints;
   }

   public static URDFJoint toURDFJoint(JointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      if (jointDefinition instanceof RevoluteJointDefinition revoluteJointDefinition)
         return toURDFJoint(revoluteJointDefinition, properties);
      if (jointDefinition instanceof PrismaticJointDefinition prismaticJointDefinition)
         return toURDFJoint(prismaticJointDefinition, properties);
      if (jointDefinition instanceof FixedJointDefinition fixedJointDefinition)
         return toURDFJoint(fixedJointDefinition, properties);
      if (jointDefinition instanceof SixDoFJointDefinition sixDoFJointDefinition)
         return toURDFJoint(sixDoFJointDefinition, properties);
      if (jointDefinition instanceof PlanarJointDefinition planarJointDefinition)
         return toURDFJoint(planarJointDefinition, properties);
      if (jointDefinition instanceof CrossFourBarJointDefinition crossFourBarJointDefinition)
         return toURDFJoint(crossFourBarJointDefinition, properties);
      throw new UnsupportedOperationException("Unsupported joint type: " + jointDefinition);
   }

   public static URDFJoint toURDFJoint(RevoluteJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      if (Double.isInfinite(jointDefinition.getPositionLowerLimit()) && Double.isInfinite(jointDefinition.getPositionLowerLimit()))
         urdfJoint.setType(URDFJointType.continuous);
      else
         urdfJoint.setType(URDFJointType.revolute);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), properties));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));
      urdfJoint.setAxis(toURDFAxis(jointDefinition.getAxis(), properties));
      urdfJoint.setLimit(toURDFLimit(jointDefinition, properties));
      urdfJoint.setDynamics(toURDFDynamics(jointDefinition, properties));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(PrismaticJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.prismatic);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), properties));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));
      urdfJoint.setAxis(toURDFAxis(jointDefinition.getAxis(), properties));
      urdfJoint.setLimit(toURDFLimit(jointDefinition, properties));
      urdfJoint.setDynamics(toURDFDynamics(jointDefinition, properties));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(FixedJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.fixed);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), properties));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));
      if (properties.alwaysExportJointAxis)
         urdfJoint.setAxis(toURDFAxis(EuclidCoreTools.zeroVector3D, properties));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(SixDoFJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.floating);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), properties));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));
      if (properties.alwaysExportJointAxis)
         urdfJoint.setAxis(toURDFAxis(EuclidCoreTools.zeroVector3D, properties));
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(PlanarJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.planar);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), properties));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));
      urdfJoint.setAxis(toURDFAxis(Axis3D.Y, properties)); // TODO need to upgrade PlanarJointDefinition to allow different axis
      return urdfJoint;
   }

   public static URDFJoint toURDFJoint(CrossFourBarJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      URDFJoint urdfJoint = new URDFJoint();
      urdfJoint.setName(jointDefinition.getName());
      urdfJoint.setType(URDFJointType.cross_four_bar);
      urdfJoint.setOrigin(toURDFOrigin(jointDefinition.getTransformToParent(), properties));
      urdfJoint.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJoint.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));
      urdfJoint.setAxis(toURDFAxis(jointDefinition.getAxis(), properties));
      urdfJoint.setLimit(toURDFLimit(jointDefinition, properties));
      urdfJoint.setDynamics(toURDFDynamics(jointDefinition, properties));

      urdfJoint.setActuatedJointIndex(Integer.toString(jointDefinition.getActuatedJointIndex()));

      URDFLink urdfLinkDA = toURDFLink(jointDefinition.getBodyDA(), properties);
      URDFLink urdfLinkBC = toURDFLink(jointDefinition.getBodyBC(), properties);

      URDFJoint urdfJointA = new URDFJoint();
      URDFJoint urdfJointB = new URDFJoint();
      URDFJoint urdfJointC = new URDFJoint();
      URDFJoint urdfJointD = new URDFJoint();

      urdfJointA.setName(jointDefinition.getJointNameA());
      urdfJointB.setName(jointDefinition.getJointNameB());
      urdfJointC.setName(jointDefinition.getJointNameC());
      urdfJointD.setName(jointDefinition.getJointNameD());

      urdfJointA.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJointB.setParent(toURDFLinkReference(jointDefinition.getPredecessor(), properties));
      urdfJointC.setParent(toURDFLinkReference(jointDefinition.getBodyBC(), properties));
      urdfJointD.setParent(toURDFLinkReference(jointDefinition.getBodyDA(), properties));

      urdfJointA.setChild(toURDFLinkReference(jointDefinition.getBodyDA(), properties));
      urdfJointB.setChild(toURDFLinkReference(jointDefinition.getBodyBC(), properties));
      urdfJointC.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));
      urdfJointD.setChild(toURDFLinkReference(jointDefinition.getSuccessor(), properties));

      urdfJointA.setLimit(toURDFLimit(jointDefinition, properties));
      urdfJointB.setLimit(toURDFLimit(jointDefinition, properties));
      urdfJointC.setLimit(toURDFLimit(jointDefinition, properties));
      urdfJointD.setLimit(toURDFLimit(jointDefinition, properties));

      urdfJointA.setType(URDFJointType.revolute);
      urdfJointB.setType(URDFJointType.revolute);
      urdfJointC.setType(URDFJointType.revolute);
      urdfJointD.setType(URDFJointType.revolute);

      urdfJointA.setOrigin(toURDFOrigin(jointDefinition.getTransformAToPredecessor(), properties));
      urdfJointB.setOrigin(toURDFOrigin(jointDefinition.getTransformBToPredecessor(), properties));
      urdfJointC.setOrigin(toURDFOrigin(jointDefinition.getTransformCToB(), properties));
      urdfJointD.setOrigin(toURDFOrigin(jointDefinition.getTransformDToA(), properties));

      urdfJoint.setSubJoints(new ArrayList<>());
      urdfJoint.getSubJoints().add(urdfJointA);
      urdfJoint.getSubJoints().add(urdfJointB);
      urdfJoint.getSubJoints().add(urdfJointC);
      urdfJoint.getSubJoints().add(urdfJointD);

      urdfJoint.setSubLinks(new ArrayList<>());
      urdfJoint.getSubLinks().add(urdfLinkDA);
      urdfJoint.getSubLinks().add(urdfLinkBC);

      return urdfJoint;
   }

   public static List<URDFGazebo> toURDFGazebos(List<JointDefinition> jointDefinitions, URDFGeneratorProperties properties)
   {
      if (jointDefinitions == null || jointDefinitions.isEmpty())
         return null;

      List<URDFGazebo> urdfGazebos = new ArrayList<>();

      for (JointDefinition jointDefinition : jointDefinitions)
      {
         List<URDFGazebo> jointURDFGazebos = toURDFGazebos(jointDefinition, properties);
         if (jointURDFGazebos != null)
            urdfGazebos.addAll(jointURDFGazebos);
      }

      return urdfGazebos;
   }

   public static List<URDFGazebo> toURDFGazebos(JointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;
      List<URDFSensor> urdfSensors = toURDFSensors(jointDefinition.getSensorDefinitions(), properties);
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

   public static List<URDFSensor> toURDFSensors(List<SensorDefinition> sensorDefinitions, URDFGeneratorProperties properties)
   {
      if (sensorDefinitions == null || sensorDefinitions.isEmpty())
         return null;

      List<URDFSensor> urdfSensors = new ArrayList<>();

      for (SensorDefinition sensorDefinition : sensorDefinitions)
      {
         URDFSensor urdfSensor = toURDFSensor(sensorDefinition, properties);
         if (urdfSensor != null)
            urdfSensors.add(urdfSensor);
      }

      return urdfSensors;
   }

   public static URDFSensor toURDFSensor(SensorDefinition sensorDefinition, URDFGeneratorProperties properties)
   {
      if (sensorDefinition == null)
         return null;

      if (sensorDefinition instanceof CameraSensorDefinition cameraSensorDefinition)
         return toURDFSensor(cameraSensorDefinition, properties);
      if (sensorDefinition instanceof LidarSensorDefinition lidarSensorDefinition)
         return toURDFSensor(lidarSensorDefinition, properties);
      if (sensorDefinition instanceof IMUSensorDefinition imuSensorDefinition)
         return toURDFSensor(imuSensorDefinition, properties);
      if (sensorDefinition instanceof WrenchSensorDefinition wrenchSensorDefinition)
         return toURDFSensor(wrenchSensorDefinition, properties);
      LogTools.warn("Unsupported sensor type: " + sensorDefinition);
      return null;
   }

   public static URDFSensor toURDFSensor(WrenchSensorDefinition sensorDefinition, URDFGeneratorProperties properties)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), properties.getDoubleFormatter(URDFSensor.class, "pose")));
      if (sensorDefinition.getUpdatePeriod() > 0)
         urdfSensor.setUpdateRate(properties.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.force_torque);
      return urdfSensor;
   }

   public static URDFSensor toURDFSensor(IMUSensorDefinition sensorDefinition, URDFGeneratorProperties properties)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), properties.getDoubleFormatter(URDFSensor.class, "pose")));
      if (sensorDefinition.getUpdatePeriod() > 0)
         urdfSensor.setUpdateRate(properties.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.imu);

      URDFIMU urdfIMU = new URDFIMU();

      URDFIMUNoise urdfIMUNoise = new URDFIMUNoise();
      urdfIMUNoise.setType(URDFIMUNoiseType.gaussian);

      { // Angular velocity noise
         URDFNoiseParameters urdfNoiseParameters = new URDFNoiseParameters();
         String mean = properties.toString(URDFNoiseParameters.class, "mean", sensorDefinition.getAngularVelocityNoiseMean());
         String stddev = properties.toString(URDFNoiseParameters.class, "stddev", sensorDefinition.getAngularVelocityNoiseStandardDeviation());
         String bias_mean = properties.toString(URDFNoiseParameters.class, "bias_mean", sensorDefinition.getAngularVelocityBiasMean());
         String bias_stddev = properties.toString(URDFNoiseParameters.class, "bias_stddev", sensorDefinition.getAngularVelocityBiasStandardDeviation());
         urdfNoiseParameters.setMean(mean);
         urdfNoiseParameters.setStddev(stddev);
         urdfNoiseParameters.setBias_mean(bias_mean);
         urdfNoiseParameters.setBias_stddev(bias_stddev);
         urdfIMUNoise.setRate(urdfNoiseParameters);
      }

      { // Acceleration noise
         String mean = properties.toString(URDFNoiseParameters.class, "mean", sensorDefinition.getAccelerationNoiseMean());
         String stddev = properties.toString(URDFNoiseParameters.class, "stddev", sensorDefinition.getAccelerationNoiseStandardDeviation());
         String bias_mean = properties.toString(URDFNoiseParameters.class, "bias_mean", sensorDefinition.getAccelerationBiasMean());
         String bias_stddev = properties.toString(URDFNoiseParameters.class, "bias_stddev", sensorDefinition.getAccelerationBiasStandardDeviation());
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

   public static URDFSensor toURDFSensor(LidarSensorDefinition sensorDefinition, URDFGeneratorProperties properties)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      urdfSensor.setName(sensorDefinition.getName());
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), properties.getDoubleFormatter(URDFSensor.class, "pose")));
      if (sensorDefinition.getUpdatePeriod() > 0)
         urdfSensor.setUpdateRate(properties.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.ray);
      urdfSensor.setRay(toURDFRay(sensorDefinition, properties));
      return urdfSensor;
   }

   public static URDFRay toURDFRay(LidarSensorDefinition sensorDefinition, URDFGeneratorProperties properties)
   {
      if (sensorDefinition == null)
         return null;

      URDFRay urdfRay = new URDFRay();

      URDFRange urdfRange = new URDFRange();
      urdfRange.setMin(properties.toString(URDFRange.class, "min", sensorDefinition.getMinRange()));
      urdfRange.setMax(properties.toString(URDFRange.class, "max", sensorDefinition.getMaxRange()));
      urdfRange.setResolution(properties.toString(URDFRange.class, "resolution", sensorDefinition.getRangeResolution()));
      urdfRay.setRange(urdfRange);

      URDFScan urdfScan = new URDFScan();

      URDFHorizontalScan urdfHorizontalScan = new URDFHorizontalScan();
      urdfHorizontalScan.setMinAngle(properties.toString(URDFHorizontalScan.class, "min_angle", sensorDefinition.getSweepYawMin()));
      urdfHorizontalScan.setMaxAngle(properties.toString(URDFHorizontalScan.class, "max_angle", sensorDefinition.getSweepYawMax()));
      urdfHorizontalScan.setSamples(Integer.toString(sensorDefinition.getPointsPerSweep()));
      urdfScan.setHorizontal(urdfHorizontalScan);

      URDFVerticalScan urdfVerticalScan = new URDFVerticalScan();
      urdfVerticalScan.setMinAngle(properties.toString(URDFVerticalScan.class, "min_angle", sensorDefinition.getHeightPitchMin()));
      urdfVerticalScan.setMaxAngle(properties.toString(URDFVerticalScan.class, "max_angle", sensorDefinition.getHeightPitchMax()));
      urdfVerticalScan.setSamples(Integer.toString(sensorDefinition.getScanHeight()));
      urdfScan.setVertical(urdfVerticalScan);
      urdfRay.setScan(urdfScan);

      URDFNoise urdfNoise = new URDFNoise();
      urdfNoise.setType(URDFNoiseType.gaussian);
      urdfNoise.setMean(properties.toString(URDFNoise.class, "mean", sensorDefinition.getGaussianNoiseMean()));
      urdfNoise.setStddev(properties.toString(URDFNoise.class, "stddev", sensorDefinition.getGaussianNoiseStandardDeviation()));
      urdfRay.setNoise(urdfNoise);

      return urdfRay;
   }

   public static URDFSensor toURDFSensor(CameraSensorDefinition sensorDefinition, URDFGeneratorProperties properties)
   {
      if (sensorDefinition == null)
         return null;

      URDFSensor urdfSensor = new URDFSensor();
      String name = sensorDefinition.getName();
      if (name.contains("_"))
         urdfSensor.setName(name.substring(0, name.lastIndexOf("_")));
      else
         urdfSensor.setName(name);
      urdfSensor.setPose(toPoseString(sensorDefinition.getTransformToJoint(), properties.getDoubleFormatter(URDFSensor.class, "pose")));
      if (sensorDefinition.getUpdatePeriod() > 0)
         urdfSensor.setUpdateRate(properties.toString(URDFSensor.class, "update_rate", 1000.0 / sensorDefinition.getUpdatePeriod()));
      urdfSensor.setType(URDFSensorType.camera);
      urdfSensor.setCamera(Collections.singletonList(toURDFCamera(sensorDefinition, properties)));
      return urdfSensor;
   }

   public static URDFCamera toURDFCamera(CameraSensorDefinition sensorDefinition, URDFGeneratorProperties properties)
   {
      if (sensorDefinition == null)
         return null;

      URDFCamera urdfCamera = new URDFCamera();
      String name = sensorDefinition.getName();
      if (name != null && name.contains("_"))
         urdfCamera.setName(name.substring(name.lastIndexOf("_") + 1));
      urdfCamera.setPose(toPoseString(sensorDefinition.getTransformToJoint(), properties.getDoubleFormatter(URDFSensor.class, "pose")));
      urdfCamera.setHorizontalFov(properties.toString(URDFCamera.class, "horizontal_fov", sensorDefinition.getFieldOfView()));
      urdfCamera.setImage(toURDFSensorImage(sensorDefinition.getImageWidth(), sensorDefinition.getImageHeight(), properties));
      urdfCamera.setClip(toURDFClip(sensorDefinition.getClipNear(), sensorDefinition.getClipFar(), properties));
      return urdfCamera;
   }

   public static URDFSensorImage toURDFSensorImage(int width, int height, URDFGeneratorProperties properties)
   {
      return toURDFSensorImage(width, height, null, properties);
   }

   public static URDFSensorImage toURDFSensorImage(int width, int height, String format, URDFGeneratorProperties properties)
   {
      URDFSensorImage urdfSensorImage = new URDFSensorImage();
      urdfSensorImage.setWidth(Integer.toString(width));
      urdfSensorImage.setHeight(Integer.toString(height));
      urdfSensorImage.setFormat(format);
      return urdfSensorImage;
   }

   public static URDFClip toURDFClip(double near, double far, URDFGeneratorProperties properties)
   {
      URDFClip urdfClip = new URDFClip();
      urdfClip.setNear(properties.toString(URDFClip.class, "near", near));
      urdfClip.setFar(properties.toString(URDFClip.class, "far", far));
      return urdfClip;
   }

   public static URDFOrigin toURDFOrigin(RigidBodyTransformReadOnly pose, URDFGeneratorProperties properties)
   {
      if (pose == null)
         return null;

      URDFOrigin urdfOrigin = new URDFOrigin();
      Tuple3DReadOnly translation = pose.getTranslation();
      Orientation3DReadOnly rotation = pose.getRotation();

      urdfOrigin.setXYZ(properties.toString(URDFOrigin.class, "xyz", translation.getX(), translation.getY(), translation.getZ()));
      urdfOrigin.setRPY(properties.toString(URDFOrigin.class, "rpy", rotation.getRoll(), rotation.getPitch(), rotation.getYaw()));
      return urdfOrigin;
   }

   public static URDFOrigin toURDFOrigin(AffineTransformReadOnly pose, URDFGeneratorProperties properties)
   {
      if (pose == null)
         return null;

      URDFOrigin urdfOrigin = new URDFOrigin();
      Tuple3DReadOnly translation = pose.getTranslation();
      Orientation3DReadOnly rotation = pose.getLinearTransform().getAsQuaternion();

      urdfOrigin.setXYZ(properties.toString(URDFOrigin.class, "xyz", translation.getX(), translation.getY(), translation.getZ()));
      urdfOrigin.setRPY(properties.toString(URDFOrigin.class, "rpy", rotation.getRoll(), rotation.getPitch(), rotation.getYaw()));

      if (!EuclidCoreTools.epsilonEquals(new Vector3D(1, 1, 1), pose.getLinearTransform().getScaleVector(), 1.0e-7))
         LogTools.warn("Discarding scale from affine trane transform.");
      return urdfOrigin;
   }

   public static String toPoseString(RigidBodyTransformReadOnly pose, DoubleFormatter doubleFormatter)
   {
      return toPoseString(pose, 1, doubleFormatter);
   }

   public static String toPoseString(RigidBodyTransformReadOnly pose, int spaceCount, DoubleFormatter doubleFormatter)
   {
      if (pose == null)
         return null;

      Tuple3DReadOnly translation = pose.getTranslation();
      Orientation3DReadOnly rotation = pose.getRotation();

      return doubleFormatter.toString(spaceCount,
                                      translation.getX(),
                                      translation.getY(),
                                      translation.getZ(),
                                      rotation.getRoll(),
                                      rotation.getPitch(),
                                      rotation.getYaw());
   }

   public static URDFAxis toURDFAxis(Tuple3DReadOnly axis, URDFGeneratorProperties properties)
   {
      if (axis == null)
         return null;

      URDFAxis urdfAxis = new URDFAxis();
      urdfAxis.setXYZ(properties.toString(URDFAxis.class, "xyz", axis.getX(), axis.getY(), axis.getZ()));
      return urdfAxis;
   }

   public static URDFLimit toURDFLimit(OneDoFJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      URDFLimit urdfLimit = new URDFLimit();
      urdfLimit.setLower(properties.toString(URDFLimit.class, "lower", jointDefinition.getPositionLowerLimit()));
      urdfLimit.setUpper(properties.toString(URDFLimit.class, "upper", jointDefinition.getPositionUpperLimit()));

      if (-jointDefinition.getVelocityLowerLimit() != jointDefinition.getVelocityUpperLimit())
      {
         LogTools.warn("Velocity limits no symmetric for joint {}, exporting smallest limit", jointDefinition.getName());
      }

      double velocity = Math.min(Math.abs(jointDefinition.getVelocityLowerLimit()), jointDefinition.getVelocityUpperLimit());
      urdfLimit.setVelocity(properties.toString(URDFLimit.class, "velocity", velocity));

      if (-jointDefinition.getEffortLowerLimit() != jointDefinition.getEffortUpperLimit())
      {
         LogTools.warn("Effort limits no symmetric for joint {}, exporting smallest limit", jointDefinition.getName());
      }

      double effort = Math.min(Math.abs(jointDefinition.getEffortLowerLimit()), jointDefinition.getEffortUpperLimit());
      urdfLimit.setEffort(properties.toString(URDFLimit.class, "effort", effort));

      return urdfLimit;
   }

   public static URDFDynamics toURDFDynamics(OneDoFJointDefinition jointDefinition, URDFGeneratorProperties properties)
   {
      if (jointDefinition == null)
         return null;

      if (!properties.alwaysExportJointDynamics)
      {
         if (jointDefinition.getStiction() == 0.0 && jointDefinition.getDamping() == 0.0)
            return null;
      }

      URDFDynamics urdfDynamics = new URDFDynamics();
      urdfDynamics.setFriction(properties.toString(URDFDynamics.class, "friction", jointDefinition.getStiction()));
      urdfDynamics.setDamping(properties.toString(URDFDynamics.class, "damping", jointDefinition.getDamping()));
      return urdfDynamics;
   }

   public static URDFLinkReference toURDFLinkReference(RigidBodyDefinition rigidBodyDefinition, URDFGeneratorProperties properties)
   {
      if (rigidBodyDefinition == null)
         return null;

      URDFLinkReference urdfLinkReference = new URDFLinkReference();
      urdfLinkReference.setLink(rigidBodyDefinition.getName());
      return urdfLinkReference;
   }

   /**
    * This class provides extra properties for tweaking operations when parsing a URDF file. It is used
    * in both
    * {@link URDFTools#loadURDFModel(InputStream, Collection, ClassLoader, URDFParserProperties)} and
    * {@link URDFTools#toRobotDefinition(URDFModel, URDFParserProperties)}.
    *
    * @author Sylvain Bertrand
    */
   public static class URDFParserProperties
   {
      private boolean ignoreNamespace = false;
      private final Set<String> jointsToIgnore = new HashSet<>();
      private final Set<String> linksToIgnore = new HashSet<>();
      private Supplier<? extends JointDefinition> rootJointFactory = SixDoFJointDefinition::new;

      private boolean autoGenerateVisualName = true;
      private boolean autoGenerateCollisionName = true;
      private boolean parseSensors = true;
      private boolean simplifyKinematics = true;
      private boolean transformToZUp = true;

      /**
       * Sets whether XML namespaces should be ignored.
       * <p>
       * Note that should always be false, it is only to handle malformed XML file that do not properly
       * declare their namespaces.
       * </p>
       *
       * @param ignoreNamespace {@code true} to ignore namespaces. Recommended value {@code false}.
       * @see URDFTools#loadURDFModel(InputStream, Collection, ClassLoader, URDFParserProperties)
       */
      public void setIgnoreNamespace(boolean ignoreNamespace)
      {
         this.ignoreNamespace = ignoreNamespace;
      }

      /**
       * Indicates that a joint should be ignored when loading the URDF.
       * <p>
       * Be careful with this as it can result in a corrupted model. Only use to handle poorly generated
       * URDF files.
       *
       * @param nameOfJointToIgnore the name of a joint to be ignored when parsing the URDF file.
       * @see URDFTools#loadURDFModel(InputStream, Collection, ClassLoader, URDFParserProperties)
       */
      public void addJointToIgnore(String nameOfJointToIgnore)
      {
         jointsToIgnore.add(nameOfJointToIgnore);
      }

      /**
       * Indicates that a link should be ignored when loading the URDF.
       * <p>
       * Be careful with this as it can result in a corrupted model. Only use to handle poorly generated
       * URDF files.
       *
       * @param nameOfLinkToIgnore the name of a link to be ignored when parsing the URDF file.
       * @see URDFTools#loadURDFModel(InputStream, Collection, ClassLoader, URDFParserProperties)
       */
      public void addLinkToIgnore(String nameOfLinkToIgnore)
      {
         linksToIgnore.add(nameOfLinkToIgnore);
      }

      /**
       * Specifies the joint factory used to create an additional root joint for the robot.
       * <p>
       * Typically, the URDF model does not explicitly declare the floating joint connecting the robot to
       * the world. When creating the robot definition, a new rigid-body representing the world and a new
       * floating joint connecting the robot to the world are created.
       * </p>
       *
       * @param rootJointFactory the factory used to create the root joint of the robot. If the URDF
       *                         already declares that joint (uncommon), then set this to {@code null}.
       * @see URDFTools#toRobotDefinition(URDFModel, URDFParserProperties)
       */
      public void setRootJointFactory(Supplier<? extends JointDefinition> rootJointFactory)
      {
         this.rootJointFactory = rootJointFactory;
      }

      /**
       * Specifies whether a default name should be generated for {@code URDFVisual} that do not
       * explicitly declare one.
       *
       * @param autoGenerateVisualName whether to automatically generate a name for visuals when it is
       *                               missing.
       */
      public void setAutoGenerateVisualName(boolean autoGenerateVisualName)
      {
         this.autoGenerateVisualName = autoGenerateVisualName;
      }

      /**
       * Specifies whether a default name should be generated for {@code URDFCollision} that do not
       * explicitly declare one.
       *
       * @param autoGenerateCollisionName whether to automatically generate a name for collisions when it
       *                                  is missing.
       */
      public void setAutoGenerateCollisionName(boolean autoGenerateCollisionName)
      {
         this.autoGenerateCollisionName = autoGenerateCollisionName;
      }

      /**
       * Specifies whether the sensors should be parsed.
       *
       * @param parseSensors {@code true} [default value] to parse the sensors from the URDF file.
       */
      public void setParseSensors(boolean parseSensors)
      {
         this.parseSensors = parseSensors;
      }

      /**
       * Specifies whether {@link RobotDefinition#simplifyKinematics()} should be called after parsing the
       * URDF model.
       * <p>
       * This will remove fixed joints from the robot definition while preserving the physical properties
       * of the robot.
       * </p>
       *
       * @param simplifyKinematics {@code true} [default value] simplifies the robot kinematics,
       *                           {@code false} do nothing.
       */
      public void setSimplifyKinematics(boolean simplifyKinematics)
      {
         this.simplifyKinematics = simplifyKinematics;
      }

      /**
       * Specifies whether {@link RobotDefinition#transformAllFramesToZUp()} should be called after
       * parsing the URDF model.
       * <p>
       * This will ensure that all local frames a pointing z-up and x-forward when the robot is in the
       * zero pose.
       * </p>
       *
       * @param transformToZUp {@code true} [default value] transforms the local frame to follow the z-up
       *                       convention.
       */
      public void setTransformToZUp(boolean transformToZUp)
      {
         this.transformToZUp = transformToZUp;
      }
   }

   /**
    * This class provides extra properties for tweaking operations when generating a URDF model. It is
    * used in {@link URDFTools#toURDFModel(RobotDefinition, URDFGeneratorProperties)}.
    *
    * @author Sylvain Bertrand
    */
   public static class URDFGeneratorProperties
   {
      private boolean alwaysExportJointDynamics = false;
      private boolean alwaysExportJointAxis = false;
      private boolean alwaysExportLinkInertial = false;

      private int defaultSpaceCount = 1;

      private DoubleFormatter defaultDoubleFormatter = Double::toString;
      private final Map<Class<? extends URDFItem>, URDFItemGeneratorProperties> urdfTypeFormatters = new HashMap<>();

      /**
       * Specifies whether the axis item should always be exported regardless of the joint type.
       *
       * @param alwaysExportJointAxis {@code true} to always export the axis item, {@code false} [default
       *                              value] to only export it for joints requiring it, such as 1-DoF
       *                              joints.
       */
      public void setAlwaysExportJointAxis(boolean alwaysExportJointAxis)
      {
         this.alwaysExportJointAxis = alwaysExportJointAxis;
      }

      /**
       * Specifies whether the dynamics item should always be exported.
       *
       * @param alwaysExportJointDynamics {@code true} to always export the dynamics item, {@code false}
       *                                  [default value] to only export it for joints requiring it, i.e.
       *                                  only when it contains meaningful values.
       */
      public void setAlwaysExportJointDynamics(boolean alwaysExportJointDynamics)
      {
         this.alwaysExportJointDynamics = alwaysExportJointDynamics;
      }

      /**
       * Specifies whether the inertial item should always be exported.
       *
       * @param alwaysExportLinkInertial {@code true} to always export the inertial item, {@code false}
       *                                 [default value] to only export it for links requiring it, i.e.
       *                                 only when it contains meaningful values.
       */
      public void setAlwaysExportLinkInertial(boolean alwaysExportLinkInertial)
      {
         this.alwaysExportLinkInertial = alwaysExportLinkInertial;
      }

      /**
       * Specifies the number of spaces to use for separating values of a multi-value item, for instance
       * the axis item. This is the default value used unless a specific value (for a given URDF item such
       * as {@code URDFAxis}) is given.
       *
       * @param defaultSpaceCount the number of spaces to use for separating values of a multi-value item.
       *                          Default value is 1.
       */
      public void setDefaultSpaceCount(int defaultSpaceCount)
      {
         this.defaultSpaceCount = defaultSpaceCount;
      }

      /**
       * Specifies the number of spaces to use for separating values of a multi-value item, and only apply
       * it to a given URDF item.
       *
       * @param urdfType   the URDF item to which the space count should be applied.
       * @param spaceCount the number of spaces to use for separating values of a multi-value item.
       *                   Default value is 1.
       */
      public void setSpaceCount(Class<? extends URDFItem> urdfType, int spaceCount)
      {
         URDFItemGeneratorProperties urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter == null)
            urdfTypeFormatters.put(urdfType, urdfTypeFormatter = new URDFItemGeneratorProperties());
         urdfTypeFormatter.setDefaultSpaceCount(spaceCount);
      }

      /**
       * Specifies a default double formatter to use for generating {@code String} for double values.
       *
       * @param formatter the default formatter to use.
       */
      public void setDefaultDoubleFormatter(DoubleFormatter formatter)
      {
         this.defaultDoubleFormatter = formatter;
      }

      /**
       * Specifies a double formatter to use instead of the default one for a given URDF item.
       *
       * @param urdfType  the URDF item to which the formatter should be applied.
       * @param formatter the formatter to use.
       */
      public void addDoubleFormatter(Class<? extends URDFItem> urdfType, DoubleFormatter formatter)
      {
         URDFItemGeneratorProperties urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter == null)
            urdfTypeFormatters.put(urdfType, urdfTypeFormatter = new URDFItemGeneratorProperties());
         urdfTypeFormatter.setDefaultDoubleFormatter(formatter);
      }

      /**
       * Specifies a double formatter to use instead of the default one for the field of a given URDF
       * item. This overrides both formatter provided via
       * {@link #setDefaultDoubleFormatter(DoubleFormatter)} and
       * {@link #addDoubleFormatter(Class, DoubleFormatter)}.
       *
       * @param urdfType  the URDF item to which the formatter should be applied.
       * @param fieldName the name of the field for which the formatter is to be applied.
       * @param formatter the formatter to use.
       */
      public void addDoubleFormatter(Class<? extends URDFItem> urdfType, String fieldName, DoubleFormatter formatter)
      {
         URDFItemGeneratorProperties urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter == null)
            urdfTypeFormatters.put(urdfType, urdfTypeFormatter = new URDFItemGeneratorProperties());
         urdfTypeFormatter.addFormatter(fieldName, formatter);
      }

      private String toString(Class<? extends URDFItem> urdfType, String fieldName, double... values)
      {
         DoubleFormatter formatter = getDoubleFormatter(urdfType, fieldName);
         int spaceCount = Math.max(1, getSpaceCount(urdfType));
         return formatter.toString(spaceCount, values);
      }

      private int getSpaceCount(Class<? extends URDFItem> urdfType)
      {
         URDFItemGeneratorProperties urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter != null)
         {
            if (urdfTypeFormatter.defaultSpaceCount != -1)
               return urdfTypeFormatter.defaultSpaceCount;
         }

         return defaultSpaceCount;
      }

      private DoubleFormatter getDoubleFormatter(Class<? extends URDFItem> urdfType, String fieldName)
      {
         URDFItemGeneratorProperties urdfTypeFormatter = urdfTypeFormatters.get(urdfType);
         if (urdfTypeFormatter != null)
         {
            if (fieldName == null)
            {
               if (urdfTypeFormatter.defaultDoubleFormatter != null)
                  return urdfTypeFormatter.defaultDoubleFormatter;
            }
            else
            {
               DoubleFormatter formatter = urdfTypeFormatter.getDoubleFormatter(fieldName.toLowerCase());
               if (formatter != null)
                  return formatter;
            }
         }

         return defaultDoubleFormatter;
      }
   }

   private static class URDFItemGeneratorProperties
   {
      /**
       * Default value for the number of spaces to use to separate values.
       */
      private int defaultSpaceCount = -1;

      private DoubleFormatter defaultDoubleFormatter;
      private final Map<String, DoubleFormatter> fieldToDoubleFormatterMap = new HashMap<>();

      public URDFItemGeneratorProperties()
      {
      }

      public void setDefaultSpaceCount(int defaultSpaceCount)
      {
         this.defaultSpaceCount = defaultSpaceCount;
      }

      public void setDefaultDoubleFormatter(DoubleFormatter formatter)
      {
         defaultDoubleFormatter = formatter;
      }

      public void addFormatter(String fieldName, DoubleFormatter formatter)
      {
         fieldToDoubleFormatterMap.put(fieldName.toLowerCase(), formatter);
      }

      private DoubleFormatter getDoubleFormatter(String fieldName)
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

   /**
    * Functional interface dedicated to generate {@code String} for double values when generating a
    * URDF model.
    *
    * @author Sylvain Bertrand
    */
   public static interface DoubleFormatter
   {
      /**
       * Generates a representative {@code String} for the given value.
       *
       * @param value the value to generate the {@code String} of.
       * @return the representative {@code String}.
       */
      String toString(double value);

      /**
       * Convenience method for expanding this formatter to format multiple double values at once.
       *
       * @param spaceCount the number of spaces for separating values.
       * @param values     the values to generate the {@code String} of.
       * @return the representative {@code String}.
       */
      default String toString(int spaceCount, double... values)
      {
         if (values == null)
            return null;
         if (values.length == 0)
            return "";

         if (values.length == 1)
            return toString(values[0]);

         String separator = " ".repeat(spaceCount);

         StringBuilder sb = new StringBuilder(toString(values[0]));

         for (int i = 1; i < values.length; i++)
         {
            sb.append(separator).append(toString(values[i]));
         }
         return sb.toString();
      }
   }
}
