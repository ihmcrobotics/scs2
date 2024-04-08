package us.ihmc.scs2.definition;

import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.*;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.state.JointState;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.PlanarJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.SphericalJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.MaterialScriptDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBADoubleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBAIntDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoCompositeDefinition;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoYawPitchRollDefinition;
import us.ihmc.scs2.definition.yoGraphic.*;
import us.ihmc.scs2.definition.yoVariable.YoBooleanDefinition;
import us.ihmc.scs2.definition.yoVariable.YoDoubleDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEnumDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationAliasDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationListDefinition;
import us.ihmc.scs2.definition.yoVariable.YoIntegerDefinition;
import us.ihmc.scs2.definition.yoVariable.YoLongDefinition;
import us.ihmc.scs2.definition.yoVariable.YoRegistryDefinition;
import us.ihmc.scs2.definition.yoVariable.YoVariableDefinition;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLClassLoader;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * Tools to load and save definitions.
 */
public class DefinitionIOTools
{
   private static final JAXBContext definitionContext;

   // Load all the resources in the classpath to ensure they are available for the JAXB context.
   static
   {
      try
      {
         List<Class<?>> classesToBeBound = new ArrayList<>();
         classesToBeBound.add(CollisionShapeDefinition.class);
         classesToBeBound.add(TerrainObjectDefinition.class);

         // GeometryDefinitions
         classesToBeBound.add(GeometryDefinition.class);
         classesToBeBound.add(ArcTorus3DDefinition.class);
         classesToBeBound.add(Box3DDefinition.class);
         classesToBeBound.add(Capsule3DDefinition.class);
         classesToBeBound.add(Cone3DDefinition.class);
         //         classesToBeBound.add(ConvexPolytope3DDefinition.class); TODO Figure this one out
         classesToBeBound.add(Cylinder3DDefinition.class);
         classesToBeBound.add(Ellipsoid3DDefinition.class);
         classesToBeBound.add(ExtrudedPolygon2DDefinition.class);
         //         classesToBeBound.add(ExtrusionDefinition.class); TODO Figure this one out
         classesToBeBound.add(HemiEllipsoid3DDefinition.class);
         classesToBeBound.add(ModelFileGeometryDefinition.class);
         classesToBeBound.add(Point3DDefinition.class);
         classesToBeBound.add(Polygon2DDefinition.class);
         classesToBeBound.add(Polygon3DDefinition.class);
         classesToBeBound.add(PyramidBox3DDefinition.class);
         classesToBeBound.add(Ramp3DDefinition.class);
         classesToBeBound.add(Sphere3DDefinition.class);
         classesToBeBound.add(STPBox3DDefinition.class);
         classesToBeBound.add(STPCapsule3DDefinition.class);
         //         classesToBeBound.add(STPConvexPolytope3DDefinition.class); TODO Figure this one out
         classesToBeBound.add(STPCylinder3DDefinition.class);
         classesToBeBound.add(STPRamp3DDefinition.class);
         classesToBeBound.add(Tetrahedron3DDefinition.class);
         classesToBeBound.add(Torus3DDefinition.class);
         classesToBeBound.add(TriangleMesh3DDefinition.class);
         classesToBeBound.add(TruncatedCone3DDefinition.class);

         // SensorDefinitions
         classesToBeBound.add(SensorDefinition.class);
         classesToBeBound.add(CameraSensorDefinition.class);
         classesToBeBound.add(IMUSensorDefinition.class);
         classesToBeBound.add(LidarSensorDefinition.class);
         classesToBeBound.add(WrenchSensorDefinition.class);

         // Key robot points
         classesToBeBound.add(KinematicPointDefinition.class);
         classesToBeBound.add(ExternalWrenchPointDefinition.class);
         classesToBeBound.add(GroundContactPointDefinition.class);

         // RobotDefinition
         classesToBeBound.add(RobotDefinition.class);
         classesToBeBound.add(RobotStateDefinition.class);
         classesToBeBound.add(RigidBodyDefinition.class);
         classesToBeBound.add(JointDefinition.class);
         classesToBeBound.add(FixedJointDefinition.class);
         classesToBeBound.add(OneDoFJointDefinition.class);
         classesToBeBound.add(CrossFourBarJointDefinition.class);
         classesToBeBound.add(PrismaticJointDefinition.class);
         classesToBeBound.add(RevoluteJointDefinition.class);
         classesToBeBound.add(PlanarJointDefinition.class);
         classesToBeBound.add(SixDoFJointDefinition.class);
         classesToBeBound.add(SphericalJointDefinition.class);

         // JointState
         classesToBeBound.add(JointState.class);
         classesToBeBound.add(OneDoFJointState.class);
         classesToBeBound.add(SixDoFJointState.class);
         classesToBeBound.add(SphericalJointState.class);
         classesToBeBound.add(PlanarJointState.class);

         // Visuals
         classesToBeBound.add(PaintDefinition.class);
         classesToBeBound.add(ColorDefinition.class);
         classesToBeBound.add(YoColorRGBADoubleDefinition.class);
         classesToBeBound.add(YoColorRGBAIntDefinition.class);
         classesToBeBound.add(YoColorRGBASingleDefinition.class);
         classesToBeBound.add(MaterialDefinition.class);
         classesToBeBound.add(MaterialScriptDefinition.class);
         classesToBeBound.add(TextureDefinition.class);
         classesToBeBound.add(VisualDefinition.class);

         // YoGraphicDefinition
         classesToBeBound.add(YoGraphic2DDefinition.class);
         classesToBeBound.add(YoGraphic3DDefinition.class);
         classesToBeBound.add(YoGraphicArrow3DDefinition.class);
         classesToBeBound.add(YoGraphicBox3DDefinition.class);
         classesToBeBound.add(YoGraphicCapsule3DDefinition.class);
         classesToBeBound.add(YoGraphicCone3DDefinition.class);
         classesToBeBound.add(YoGraphicConvexPolytope3DDefinition.class);
         classesToBeBound.add(YoGraphicCoordinateSystem3DDefinition.class);
         classesToBeBound.add(YoGraphicCylinder3DDefinition.class);
         classesToBeBound.add(YoGraphicDefinition.class);
         classesToBeBound.add(YoGraphicEllipsoid3DDefinition.class);
         classesToBeBound.add(YoGraphicGroupDefinition.class);
         classesToBeBound.add(YoGraphicLine2DDefinition.class);
         classesToBeBound.add(YoGraphicListDefinition.class);
         classesToBeBound.add(YoGraphicPoint2DDefinition.class);
         classesToBeBound.add(YoGraphicPoint3DDefinition.class);
         classesToBeBound.add(YoGraphicPointcloud2DDefinition.class);
         classesToBeBound.add(YoGraphicPointcloud3DDefinition.class);
         classesToBeBound.add(YoGraphicPolygon2DDefinition.class);
         classesToBeBound.add(YoGraphicPolygonExtruded3DDefinition.class);
         classesToBeBound.add(YoGraphicPolynomial3DDefinition.class);
         classesToBeBound.add(YoGraphicRamp3DDefinition.class);
         classesToBeBound.add(YoGraphicSTPBox3DDefinition.class);
         classesToBeBound.add(YoGraphicRobotDefinition.class);
         classesToBeBound.add(YoListDefinition.class);

         // YoCompositeDefinition
         classesToBeBound.add(YoCompositeDefinition.class);
         classesToBeBound.add(YoTuple2DDefinition.class);
         classesToBeBound.add(YoTuple3DDefinition.class);
         classesToBeBound.add(YoOrientation3DDefinition.class);
         classesToBeBound.add(YoQuaternionDefinition.class);
         classesToBeBound.add(YoYawPitchRollDefinition.class);

         // YoVariableDefinition
         classesToBeBound.add(YoRegistryDefinition.class);
         classesToBeBound.add(YoVariableDefinition.class);
         classesToBeBound.add(YoBooleanDefinition.class);
         classesToBeBound.add(YoDoubleDefinition.class);
         classesToBeBound.add(YoIntegerDefinition.class);
         classesToBeBound.add(YoLongDefinition.class);
         classesToBeBound.add(YoEnumDefinition.class);

         // YoEquationDefinition
         classesToBeBound.add(YoEquationDefinition.class);
         classesToBeBound.add(EquationAliasDefinition.class);
         classesToBeBound.add(YoEquationListDefinition.class);

         classesToBeBound.add(SessionInformationDefinition.class);

         definitionContext = JAXBContext.newInstance(classesToBeBound.toArray(new Class[classesToBeBound.size()]));
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void loadResources()
   {
      // Only need to load this class to get the resources loaded.
   }

   /**
    * Loads a {@link YoGraphicListDefinition} from the given file.
    * This loader also figures out the resources when any.
    *
    * @param file the file to load the definition from.
    * @return the loaded definition.
    */
   public static YoGraphicListDefinition loadYoGraphicListDefinition(File file) throws JAXBException, IOException
   {
      YoGraphicListDefinition yoGraphicListDefinition = loadYoGraphicListDefinition(new FileInputStream(file));
      setResourcesClassLoaderRecursive(file, yoGraphicListDefinition);
      return yoGraphicListDefinition;
   }

   /**
    * Sets the {@link RobotDefinition#setResourceClassLoader(ClassLoader)} for all the {@link RobotDefinition} in the given {@link YoGraphicDefinition}.
    *
    * @param file                the file from which the {@link YoGraphicDefinition} was loaded.
    * @param yoGraphicDefinition the definition to process.
    */
   private static void setResourcesClassLoaderRecursive(File file, YoGraphicDefinition yoGraphicDefinition) throws MalformedURLException
   {
      if (yoGraphicDefinition instanceof YoGraphicRobotDefinition yoGraphicRobotDefinition)
      {
         RobotDefinition robotDefinition = yoGraphicRobotDefinition.getRobotDefinition();
         robotDefinition.setResourceClassLoader(new URLClassLoader(new URL[] {file.getParentFile().toURI().toURL()}));
      }
      else if (yoGraphicDefinition instanceof YoGraphicGroupDefinition yoGraphicGroupDefinition)
      {
         for (YoGraphicDefinition child : yoGraphicGroupDefinition.getChildren())
         {
            setResourcesClassLoaderRecursive(file, child);
         }
      }
      else if (yoGraphicDefinition instanceof YoGraphicListDefinition yoGraphicListDefinition)
      {
         for (YoGraphicDefinition child : yoGraphicListDefinition.getYoGraphics())
         {
            setResourcesClassLoaderRecursive(file, child);
         }
      }
   }

   /**
    * Loads a {@link YoGraphicListDefinition} from the given input stream.
    * This loader cannot figure out resources.
    *
    * @param inputStream the input stream to load the definition from.
    * @return the loaded definition.
    */
   public static YoGraphicListDefinition loadYoGraphicListDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (YoGraphicListDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   /**
    * Saves the given {@link YoGraphicListDefinition} to the given file.
    * This method also saves the resources when any.
    *
    * @param definitionFile     the file to save the definition to.
    * @param definition         the definition to save.
    * @param resourcesDirectory the directory to save the resources to.
    */
   public static void saveYoGraphicListDefinitionAndResources(File definitionFile, YoGraphicListDefinition definition, File resourcesDirectory)
         throws JAXBException, IOException, URISyntaxException
   {
      YoGraphicListDefinition copy = definition.copy();
      processYoGraphicResources(definitionFile, copy, resourcesDirectory);
      saveYoGraphicListDefinition(new FileOutputStream(definitionFile), copy);
   }

   /**
    * Saves the given {@link YoGraphicListDefinition} to the given output stream.
    * This method cannot save the resources.
    *
    * @param outputStream the output stream to save the definition to.
    * @param definition   the definition to save.
    */
   public static void saveYoGraphicListDefinition(OutputStream outputStream, YoGraphicListDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   /**
    * Processes and saves the resources of the given {@link YoGraphicDefinition} and copies them to the given {@code resourcesDirectory}.
    *
    * @param yoGraphicFile      the file from which the {@link YoGraphicDefinition} is being saved to.
    * @param start              the definition to process.
    * @param resourcesDirectory the directory to save the resources to.
    */
   private static void processYoGraphicResources(File yoGraphicFile, YoGraphicDefinition start, File resourcesDirectory) throws IOException, URISyntaxException
   {
      if (start instanceof YoGraphicRobotDefinition yoGraphicRobotDefinition)
      {
         RobotDefinition robotDefinition = yoGraphicRobotDefinition.getRobotDefinition();
         if (robotDefinition != null)
            processResources(yoGraphicFile, robotDefinition, resourcesDirectory, robotDefinition.getResourceClassLoader());
      }
      else if (start instanceof YoGraphicGroupDefinition yoGraphicGroupDefinition)
      {
         for (YoGraphicDefinition child : yoGraphicGroupDefinition.getChildren())
         {
            processYoGraphicResources(yoGraphicFile, child, resourcesDirectory);
         }
      }
      else if (start instanceof YoGraphicListDefinition yoGraphicListDefinition)
      {
         for (YoGraphicDefinition child : yoGraphicListDefinition.getYoGraphics())
         {
            processYoGraphicResources(yoGraphicFile, child, resourcesDirectory);
         }
      }
   }

   /**
    * Loads a {@link RobotDefinition} from the given input stream.
    * This loader cannot figure out resources.
    *
    * @param inputStream the input stream to load the definition from.
    * @return the loaded definition.
    */
   public static RobotDefinition loadRobotDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         RobotDefinition loadedRobot = (RobotDefinition) unmarshaller.unmarshal(inputStream);
         connectKinematicsRecursive(loadedRobot.getRootBodyDefinition());
         return loadedRobot;
      }
   }

   /**
    * Saves the given {@link RobotDefinition} to the given file.
    * This method also saves the resources when any.
    *
    * @param robotDefinitionFile the file to save the definition to.
    * @param robotDefinition     the definition to save.
    * @param resourceDirectory   the directory to save the resources to.
    * @param defaultClassLoader  the class loader to use to load the resources to be copied over.
    */
   public static void saveRobotDefinitionAndResources(File robotDefinitionFile,
                                                      RobotDefinition robotDefinition,
                                                      File resourceDirectory,
                                                      ClassLoader defaultClassLoader) throws IOException, JAXBException, URISyntaxException
   {
      ClassLoader classLoader = robotDefinition.getResourceClassLoader() != null ? robotDefinition.getResourceClassLoader() : defaultClassLoader;
      RobotDefinition copy = new RobotDefinition(robotDefinition);
      processResources(robotDefinitionFile, robotDefinition, resourceDirectory, classLoader);
      saveRobotDefinition(new FileOutputStream(robotDefinitionFile), copy);
   }

   /**
    * Saves the given {@link RobotDefinition} to the given output stream.
    * This method cannot save the resources.
    *
    * @param outputStream the output stream to save the definition to.
    * @param definition   the definition to save.
    */
   public static void saveRobotDefinition(OutputStream outputStream, RobotDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   /**
    * Loads a {@link TerrainObjectDefinition} from the given input stream.
    * This loader cannot figure out resources.
    *
    * @param inputStream the input stream to load the definition from.
    * @return the loaded definition.
    */
   public static TerrainObjectDefinition loadTerrainObjectDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (TerrainObjectDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   /**
    * Saves the given {@link TerrainObjectDefinition} to the given file.
    * This method also saves the resources when any.
    *
    * @param terrainObjectFile       the file to save the definition to.
    * @param terrainObjectDefinition the definition to save.
    * @param resourceDirectory       the directory to save the resources to.
    * @param defaultClassLoader      the class loader to use to load the resources to be copied over.
    */
   public static void saveTerrainObjectDefinitionAndResources(File terrainObjectFile,
                                                              TerrainObjectDefinition terrainObjectDefinition,
                                                              File resourceDirectory,
                                                              ClassLoader defaultClassLoader) throws IOException, JAXBException, URISyntaxException
   {
      ClassLoader classLoader =
            terrainObjectDefinition.getResourceClassLoader() != null ? terrainObjectDefinition.getResourceClassLoader() : defaultClassLoader;
      TerrainObjectDefinition copy = new TerrainObjectDefinition(terrainObjectDefinition);
      processResources(terrainObjectFile, terrainObjectDefinition, resourceDirectory, classLoader);
      saveTerrainObjectDefinition(new FileOutputStream(terrainObjectFile), copy);
   }

   /**
    * Saves the given {@link TerrainObjectDefinition} to the given output stream.
    * This method cannot save the resources.
    *
    * @param outputStream the output stream to save the definition to.
    * @param definition   the definition to save.
    */
   public static void saveTerrainObjectDefinition(OutputStream outputStream, TerrainObjectDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   /**
    * Loads a {@link YoEquationListDefinition} from the given input stream.
    *
    * @param inputStream the input stream to load the definition from.
    * @return the loaded definition.
    */
   public static RobotStateDefinition loadRobotStateDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (RobotStateDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   /**
    * Saves the given {@link RobotStateDefinition} to the given output stream.
    *
    * @param outputStream the output stream to save the definition to.
    * @param definition   the definition to save.
    */
   public static void saveRobotStateDefinition(OutputStream outputStream, RobotStateDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   /**
    * Loads a {@link YoEquationListDefinition} from the given input stream.
    *
    * @param inputStream the input stream to load the definition from.
    * @return the loaded definition.
    */
   public static YoEquationListDefinition loadYoEquationListDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (YoEquationListDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   /**
    * Saves the given list of {@link YoEquationDefinition}s to the given output stream.
    *
    * @param outputStream the output stream to save the definition to.
    * @param definitions  the definitions to save.
    */
   public static void saveYoEquationListDefinition(OutputStream outputStream, List<YoEquationDefinition> definitions) throws JAXBException, IOException
   {
      saveYoEquationListDefinition(outputStream, new YoEquationListDefinition(definitions));
   }

   /**
    * Saves the given {@link YoEquationListDefinition} to the given output stream.
    *
    * @param outputStream the output stream to save the definition to.
    * @param definition   the definition to save.
    */
   public static void saveYoEquationListDefinition(OutputStream outputStream, YoEquationListDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   /**
    * Loads a {@link SessionInformationDefinition} from the given input stream.
    *
    * @param inputStream the input stream to load the definition from.
    * @return the loaded definition.
    */
   public static SessionInformationDefinition loadSessionInformationDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (SessionInformationDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   /**
    * Saves the given {@link SessionInformationDefinition} to the given output stream.
    *
    * @param outputStream the output stream to save the definition to.
    * @param definition   the definition to save.
    */
   public static void saveSessionInformationDefinition(OutputStream outputStream, SessionInformationDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   /**
    * Copy the possible resources pointed to by the {@code robotDefinition} to the {@code resourceDirectory}.
    * The filename of the {@code robotDefinition} is updated to point to the new resource location.
    *
    * @param robotFile          where the robot is saved. Used to resolve relative paths.
    * @param robotDefinition    the definition to process. Modified.
    * @param resourceDirectory  where to save the resources.
    * @param defaultClassLoader the class loader to use to load the resources to be copied over.
    */
   private static void processResources(File robotFile, RobotDefinition robotDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      processResources(robotFile, robotDefinition.getRootBodyDefinition(), resourceDirectory, defaultClassLoader);
   }

   /**
    * Copy the possible resources pointed to by the {@code rigidBody} to the {@code resourceDirectory}.
    * The filename of the {@code rigidBody} is updated to point to the new resource location.
    *
    * @param baseFile           where the rigid body is saved. Used to resolve relative paths.
    * @param rigidBody          the definition to process. Modified.
    * @param resourceDirectory  where to save the resources.
    * @param defaultClassLoader the class loader to use to load the resources to be copied over.
    */
   private static void processResources(File baseFile, RigidBodyDefinition rigidBody, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      for (VisualDefinition visualDefinition : rigidBody.getVisualDefinitions())
      {
         processResources(baseFile, visualDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
         processResources(baseFile, visualDefinition.getMaterialDefinition(), resourceDirectory, defaultClassLoader);
      }

      for (CollisionShapeDefinition collisionShapeDefinition : rigidBody.getCollisionShapeDefinitions())
      {
         processResources(baseFile, collisionShapeDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
      }

      for (JointDefinition jointDefinition : rigidBody.getChildrenJoints())
      {
         processResources(baseFile, jointDefinition.getSuccessor(), resourceDirectory, defaultClassLoader);
      }
   }

   /**
    * Copy the possible resources pointed to by the {@code terrainObjectDefinition} to the {@code resourceDirectory}.
    * The filename of the {@code terrainObjectDefinition} is updated to point to the new resource location.
    *
    * @param terrainObjectFile       where the geometry is saved. Used to resolve relative paths.
    * @param terrainObjectDefinition the definition to process. Modified.
    * @param resourceDirectory       where to save the resources.
    * @param defaultClassLoader      the class loader to use to load the resources to be copied over.
    */
   private static void processResources(File terrainObjectFile,
                                        TerrainObjectDefinition terrainObjectDefinition,
                                        File resourceDirectory,
                                        ClassLoader defaultClassLoader) throws IOException, URISyntaxException
   {
      TerrainObjectDefinition copy = new TerrainObjectDefinition(terrainObjectDefinition);

      for (VisualDefinition visualDefinition : copy.getVisualDefinitions())
      {
         processResources(terrainObjectFile, visualDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
         processResources(terrainObjectFile, visualDefinition.getMaterialDefinition(), resourceDirectory, defaultClassLoader);
      }

      for (CollisionShapeDefinition collisionShapeDefinition : copy.getCollisionShapeDefinitions())
      {
         processResources(terrainObjectFile, collisionShapeDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
      }
   }

   /**
    * Copy the possible resources pointed to by the {@code geometryDefinition} to the {@code resourceDirectory}.
    * The filename of the {@code geometryDefinition} is updated to point to the new resource location.
    *
    * @param baseFile           where the geometry is saved. Used to resolve relative paths.
    * @param geometryDefinition the definition to process. Modified.
    * @param resourceDirectory  where to save the resources.
    * @param defaultClassLoader the class loader to use to load the resources to be copied over.
    */
   private static void processResources(File baseFile, GeometryDefinition geometryDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      if (geometryDefinition == null)
         return;

      if (geometryDefinition instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
      {
         if (modelFileGeometryDefinition.getFileName() == null)
            return;

         String filename = modelFileGeometryDefinition.getFileName();
         Path targetPath = computeResourceTargetPath(resourceDirectory, filename);
         Path relativePath = baseFile == null ? targetPath : baseFile.getParentFile().toPath().relativize(targetPath);
         modelFileGeometryDefinition.setFileName(relativePath.toString().replace("\\", "/"));

         if (Files.exists(targetPath))
            return;

         Files.createDirectories(targetPath.getParent());

         ClassLoader resourceClassLoader = modelFileGeometryDefinition.getResourceClassLoader();
         if (resourceClassLoader == null)
            resourceClassLoader = defaultClassLoader;
         URL sourceURL = filenameToURL(filename, resourceClassLoader);

         copyFileAndSiblings(sourceURL, targetPath, defaultClassLoader);
      }
   }

   /**
    * Convenience method that resolves the filename in the resource directory while simplifying the path.
    *
    * @param resourceDirectory the directory where the resources are saved.
    * @param filename          the filename to resolve.
    * @return
    */
   private static Path computeResourceTargetPath(File resourceDirectory, String filename)
   {
      if (filename == null || filename.isEmpty())
         return null;

      filename = filename.replace("\\", "/");

      Path targetPath;

      if (filename.contains("/"))
      {
         while (filename.substring(0, filename.indexOf("/")).equals(resourceDirectory.getName()))
         {
            filename = filename.substring(filename.indexOf("/") + 1);
         }
      }

      targetPath = resourceDirectory.toPath().resolve(filename);
      return targetPath;
   }

   /**
    * Copy the possible resources pointed to by the {@code materialDefinition} to the {@code resourceDirectory}.
    * The filename of the {@code materialDefinition} is updated to point to the new resource location.
    *
    * @param baseFile           where the material is saved. Used to resolve relative paths.
    * @param materialDefinition the definition to process. Modified.
    * @param resourceDirectory  where to save the resources.
    * @param defaultClassLoader the class loader to use to load the resources to be copied over.
    */
   private static void processResources(File baseFile, MaterialDefinition materialDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      if (materialDefinition == null)
         return;
      processResources(baseFile, materialDefinition.getDiffuseMap(), resourceDirectory, defaultClassLoader);
      processResources(baseFile, materialDefinition.getEmissiveMap(), resourceDirectory, defaultClassLoader);
      processResources(baseFile, materialDefinition.getNormalMap(), resourceDirectory, defaultClassLoader);
      processResources(baseFile, materialDefinition.getSpecularMap(), resourceDirectory, defaultClassLoader);
   }

   /**
    * Copy the possible resources pointed to by the {@code textureDefinition} to the {@code resourceDirectory}.
    * The filename of the {@code textureDefinition} is updated to point to the new resource location.
    *
    * @param baseFile           where the texture is saved. Used to resolve relative paths.
    * @param textureDefinition  the definition to process. Modified.
    * @param resourceDirectory  where to save the resources.
    * @param defaultClassLoader the class loader to use to load the resources to be copied over.
    */
   private static void processResources(File baseFile, TextureDefinition textureDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      if (textureDefinition == null)
         return;

      if (textureDefinition.getImage() != null)
         throw new UnsupportedOperationException("Implement me");

      URL sourceURL;

      String filename = textureDefinition.getFilename();

      if (filename != null)
      {
         sourceURL = filenameToURL(filename, defaultClassLoader);
      }
      else if (textureDefinition.getFileURL() != null)
      {
         sourceURL = textureDefinition.getFileURL();
         filename = sourceURL.getPath();
      }
      else
      {
         return;
      }

      Path targetPath = computeResourceTargetPath(resourceDirectory, filename);
      Path relativePath = baseFile == null ? targetPath : baseFile.getParentFile().toPath().relativize(targetPath);
      textureDefinition.setFilename(relativePath.toString().replace("\\", "/"));

      if (!Files.exists(targetPath))
      {
         Files.createDirectories(targetPath.getParent());
         copyFileAndSiblings(sourceURL, targetPath, defaultClassLoader);
      }
   }

   /**
    * Convenience method to retrieve the URL of a file.
    *
    * @param filename            the name of the file.
    * @param resourceClassLoader the class loader to use to load the resources.
    * @return the URL of the file.
    */
   public static URL filenameToURL(String filename, ClassLoader resourceClassLoader)
   {
      URL fileURL = resourceClassLoader.getResource(filename);

      if (fileURL == null)
      {
         File file = new File(filename);
         try
         {
            fileURL = file.toURI().toURL();
         }
         catch (MalformedURLException e)
         {
            throw new RuntimeException(e);
         }
      }
      return fileURL;
   }

   /**
    * Copy the files in the {@code sourceURL}'s parent folder to the {@code targetPath}.
    *
    * @param sourceURL           the URL pointing to the file to copy.
    * @param targetPath          the path to copy the file to.
    * @param resourceClassLoader the class loader to use to load the resources to be copied over.
    */
   private static void copyFileAndSiblings(URL sourceURL, Path targetPath, ClassLoader resourceClassLoader) throws IOException, URISyntaxException
   {
      if (sourceURL.getProtocol().equals("jar"))
      {
         copyJarFileAndSiblings(sourceURL, targetPath, resourceClassLoader);
      }
      else
      {
         copyFileTree(Paths.get(sourceURL.toURI()).getParent(), targetPath.getParent());
      }
   }

   /**
    * Copy the file tree from the jar file pointed to by the {@code sourceURL} to the {@code targetPath}.
    *
    * @param sourceURL           the URL pointing to the jar file.
    * @param targetPath          the path to copy the file tree to.
    * @param resourceClassLoader the class loader to use to load the resources to be copied over.
    */
   private static void copyJarFileAndSiblings(URL sourceURL, Path targetPath, ClassLoader resourceClassLoader) throws IOException
   {
      Path targetParentPath = targetPath.getParent();

      String internalParentPath = sourceURL.getPath().substring(sourceURL.getPath().indexOf("!") + 2);

      if (internalParentPath.contains("/"))
      {
         internalParentPath = internalParentPath.substring(0, internalParentPath.lastIndexOf("/") + 1);
         String correctedTarget = targetParentPath.toString().replace("\\", "/");
         int endIndex = correctedTarget.lastIndexOf(internalParentPath.substring(0, internalParentPath.length() - 1)); // removing the last '/'
         correctedTarget = correctedTarget.substring(0, endIndex);
         targetParentPath = Paths.get(correctedTarget);
      }
      else
      {
         internalParentPath = null;
      }

      String jarPath = sourceURL.getPath().substring(5, sourceURL.getPath().indexOf("!"));
      try (JarFile jar = new JarFile(URLDecoder.decode(jarPath, StandardCharsets.UTF_8)))
      {
         Enumeration<JarEntry> entries = jar.entries(); //gives ALL entries in jar

         while (entries.hasMoreElements())
         {
            JarEntry nextElement = entries.nextElement();
            String nextElementName = nextElement.getName();

            if (internalParentPath != null && !nextElementName.startsWith(internalParentPath))
               continue;

            if (nextElementName.equals(internalParentPath))
               continue;

            if (Files.exists(targetParentPath.resolve(nextElementName)))
               continue;

            Files.copy(filenameToURL(nextElementName, resourceClassLoader).openStream(), targetParentPath.resolve(nextElementName));
         }
      }
   }

   /**
    * Copy the file tree from {@code sourcePath} to {@code targetPath}.
    *
    * @param sourcePath the path to copy from.
    * @param targetPath the path to copy to.
    */
   private static void copyFileTree(Path sourcePath, Path targetPath) throws IOException
   {
      if (!Files.exists(targetPath))
         Files.copy(sourcePath, targetPath);

      if (Files.isDirectory(sourcePath))
      {
         MutableObject<IOException> thrownException = new MutableObject<>(null);

         Files.list(sourcePath).forEach(sourceChildPath ->
                                        {
                                           try
                                           {
                                              copyFileTree(sourceChildPath, targetPath.resolve(sourceChildPath.getFileName()));
                                           }
                                           catch (IOException e)
                                           {
                                              thrownException.setValue(e);
                                           }
                                        });

         if (thrownException.getValue() != null)
            throw thrownException.getValue();
      }
   }

   /**
    * Ensures {@link  JointDefinition#setPredecessor(RigidBodyDefinition)} and {@link RigidBodyDefinition#setParentJoint(JointDefinition)} are properly set for
    * all the joints and bodies in the kinematic chain starting from the given {@code rootBody}.
    *
    * @param currentBody the root of the kinematic chain to process.
    */
   private static void connectKinematicsRecursive(RigidBodyDefinition currentBody)
   {
      if (currentBody.getChildrenJoints() == null)
         return;

      for (JointDefinition childJoint : currentBody.getChildrenJoints())
      {
         childJoint.setPredecessor(currentBody);
         if (childJoint.getSuccessor() == null)
            continue;
         childJoint.getSuccessor().setParentJoint(childJoint);
         connectKinematicsRecursive(childJoint.getSuccessor());
      }
   }
}
