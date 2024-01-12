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
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
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

public class DefinitionIOTools
{
   private static final JAXBContext definitionContext;

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

   public static YoGraphicListDefinition loadYoGraphicListDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (YoGraphicListDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   public static void saveYoGraphicListDefinition(OutputStream outputStream, YoGraphicListDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

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

   public static void saveRobotDefinition(OutputStream outputStream, RobotDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   public static TerrainObjectDefinition loadTerrainObjectDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (TerrainObjectDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   public static void saveTerrainObjectDefinition(OutputStream outputStream, TerrainObjectDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   public static RobotStateDefinition loadRobotStateDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (RobotStateDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   public static void saveRobotStateDefinition(OutputStream outputStream, RobotStateDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   public static YoEquationListDefinition loadYoEquationListDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (YoEquationListDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   public static void saveYoEquationListDefinition(OutputStream outputStream, List<YoEquationDefinition> definitions) throws JAXBException, IOException
   {
      saveYoEquationListDefinition(outputStream, new YoEquationListDefinition(definitions));
   }

   public static void saveYoEquationListDefinition(OutputStream outputStream, YoEquationListDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   public static SessionInformationDefinition loadSessionInformationDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try (inputStream)
      {
         Unmarshaller unmarshaller = definitionContext.createUnmarshaller();
         return (SessionInformationDefinition) unmarshaller.unmarshal(inputStream);
      }
   }

   public static void saveSessionInformationDefinition(OutputStream outputStream, SessionInformationDefinition definition) throws JAXBException, IOException
   {
      try (outputStream)
      {
         Marshaller marshaller = definitionContext.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
   }

   public static void saveResources(RobotDefinition robotDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      processResources(robotDefinition.getRootBodyDefinition(), resourceDirectory, defaultClassLoader);
   }

   private static void processResources(RigidBodyDefinition rigidBody, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      for (VisualDefinition visualDefinition : rigidBody.getVisualDefinitions())
      {
         processResources(visualDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
         processResources(visualDefinition.getMaterialDefinition(), resourceDirectory, defaultClassLoader);
      }

      for (CollisionShapeDefinition collisionShapeDefinition : rigidBody.getCollisionShapeDefinitions())
      {
         processResources(collisionShapeDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
      }

      for (JointDefinition jointDefinition : rigidBody.getChildrenJoints())
      {
         processResources(jointDefinition.getSuccessor(), resourceDirectory, defaultClassLoader);
      }
   }

   public static void saveResources(TerrainObjectDefinition terrainObjectDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      for (VisualDefinition visualDefinition : terrainObjectDefinition.getVisualDefinitions())
      {
         processResources(visualDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
         processResources(visualDefinition.getMaterialDefinition(), resourceDirectory, defaultClassLoader);
      }

      for (CollisionShapeDefinition collisionShapeDefinition : terrainObjectDefinition.getCollisionShapeDefinitions())
      {
         processResources(collisionShapeDefinition.getGeometryDefinition(), resourceDirectory, defaultClassLoader);
      }
   }

   private static void processResources(GeometryDefinition geometryDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      if (geometryDefinition == null)
         return;

      if (geometryDefinition instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
      {
         if (modelFileGeometryDefinition.getFileName() == null)
            return;

         String filename = modelFileGeometryDefinition.getFileName();
         Path targetPath = resourceDirectory.toPath().resolve(filename);

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

   private static void processResources(MaterialDefinition materialDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      if (materialDefinition == null)
         return;
      processResources(materialDefinition.getDiffuseMap(), resourceDirectory, defaultClassLoader);
      processResources(materialDefinition.getEmissiveMap(), resourceDirectory, defaultClassLoader);
      processResources(materialDefinition.getNormalMap(), resourceDirectory, defaultClassLoader);
      processResources(materialDefinition.getSpecularMap(), resourceDirectory, defaultClassLoader);
   }

   private static void processResources(TextureDefinition textureDefinition, File resourceDirectory, ClassLoader defaultClassLoader)
         throws IOException, URISyntaxException
   {
      if (textureDefinition == null)
         return;

      if (textureDefinition.getImage() != null)
         throw new UnsupportedOperationException("Implement me");

      URL sourceURL;

      if (textureDefinition.getFilename() != null)
      {
         sourceURL = filenameToURL(textureDefinition.getFilename(), defaultClassLoader);
      }
      else if (textureDefinition.getFileURL() != null)
      {
         sourceURL = textureDefinition.getFileURL();
      }
      else
      {
         return;
      }

      Path targetPath = resourceDirectory.toPath().resolve(textureDefinition.getFilename());

      if (Files.exists(targetPath))
         return;

      Files.createDirectories(targetPath.getParent());
      copyFileAndSiblings(sourceURL, targetPath, defaultClassLoader);
   }

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

   private static void copyJarFileAndSiblings(URL sourceURL, Path targetPath, ClassLoader resourceClassLoader)
         throws UnsupportedEncodingException, IOException, URISyntaxException
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
