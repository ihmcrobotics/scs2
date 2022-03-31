package us.ihmc.scs2.sessionVisualizer.jfx.definition;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.lang.reflect.Modifier;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.io.FilenameUtils;

import com.interactivemesh.jfx.importer.FilePath;
import com.interactivemesh.jfx.importer.ImportException;
import com.interactivemesh.jfx.importer.ModelImporter;
import com.interactivemesh.jfx.importer.col.ColAsset;
import com.interactivemesh.jfx.importer.col.ColImportOption;
import com.interactivemesh.jfx.importer.col.ColModelImporter;
import com.interactivemesh.jfx.importer.obj.ObjImportOption;
import com.interactivemesh.jfx.importer.obj.ObjModelImporter;
import com.interactivemesh.jfx.importer.stl.StlMeshImporter;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Shape3D;
import javafx.scene.shape.Sphere;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories;

public class JavaFXVisualTools
{
   private static class CachedImportedModel
   {
      private final Mesh[] meshes;
      private final Material[] materials;
      private final List<Transform>[] transforms;

      @SuppressWarnings("unchecked")
      private CachedImportedModel(MeshView[] importedMeshViews)
      {
         meshes = new Mesh[importedMeshViews.length];
         materials = new Material[importedMeshViews.length];
         transforms = new List[importedMeshViews.length];

         for (int i = 0; i < importedMeshViews.length; i++)
         {
            MeshView meshView = importedMeshViews[i];
            meshes[i] = meshView.getMesh();
            materials[i] = meshView.getMaterial();
            transforms[i] = new ArrayList<>(meshView.getTransforms());
         }
      }

      private MeshView[] newMeshViews()
      {
         MeshView[] meshViews = new MeshView[meshes.length];
         for (int i = 0; i < meshes.length; i++)
         {
            MeshView meshView = new MeshView(meshes[i]);
            meshView.setMaterial(materials[i]);
            meshView.getTransforms().setAll(transforms[i]);
            meshViews[i] = meshView;
         }
         return meshViews;
      }
   }

   // TODO This improves drastically the performance of the 3D view when loading Atlas model.
   private static final Map<Long, PhongMaterial> cachedNamedColladaMaterials = new ConcurrentHashMap<>();
   private static final Map<Long, PhongMaterial> cachedNamedOBJMaterials = new ConcurrentHashMap<>();
   private static final Map<String, CachedImportedModel> cachedColladaModels = new ConcurrentHashMap<>();
   private static final Map<String, CachedImportedModel> cachedOBJModels = new ConcurrentHashMap<>();

   private static final Rotate ROTATE_PI_X = new Rotate(180.0, Rotate.X_AXIS);
   private static final Color DEFAULT_COLOR = Color.DARKBLUE;
   private static final Material DEFAULT_MATERIAL = new PhongMaterial(DEFAULT_COLOR);
   private static final Shape3D DEFAULT_GEOMETRY = null;
   private static final Box DEFAULT_BOX = null;
   private static final Cylinder DEFAULT_CYLINDER = null;
   private static final Sphere DEFAULT_SPHERE = null;
   private static final Node[] DEFAULT_MESH_VIEWS = null;

   public static Node collectNodes(List<VisualDefinition> visualDefinitions)
   {
      return collectNodes(visualDefinitions, null);
   }

   public static Node collectNodes(List<VisualDefinition> visualDefinitions, ClassLoader resourceClassLoader)
   {
      List<Node> nodes = new ArrayList<>();

      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         Node node = toNode(visualDefinition, resourceClassLoader);

         if (node != null)
         {
            nodes.add(node);
         }
      }

      if (nodes.isEmpty())
         return null;
      else if (nodes.size() == 1)
         return nodes.get(0);
      else
         return new Group(nodes);
   }

   public static Node toNode(VisualDefinition visualDefinition, ClassLoader resourceClassLoader)
   {
      Node node = toShape3D(visualDefinition.getGeometryDefinition(), visualDefinition.getMaterialDefinition(), resourceClassLoader);

      AffineTransformDefinition originPose = visualDefinition.getOriginPose();

      if (node != null && originPose != null && (originPose.hasTranslation() || originPose.hasLinearTransform()))
      {
         Affine nodeAffine = new Affine();
         JavaFXTools.convertEuclidAffineToJavaFXAffine(originPose, nodeAffine);
         node.getTransforms().add(0, nodeAffine);
      }

      return node;
   }

   public static Node toShape3D(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition, ClassLoader resourceClassLoader)
   {
      if (geometryDefinition == null)
      {
         return DEFAULT_GEOMETRY;
      }
      else if (geometryDefinition instanceof Box3DDefinition)
      {
         Box box = toBox((Box3DDefinition) geometryDefinition);
         box.setMaterial(toMaterial(materialDefinition, resourceClassLoader));
         return box;
      }
      else if (geometryDefinition instanceof Cylinder3DDefinition)
      {
         Shape3D cylinder = toCylinder((Cylinder3DDefinition) geometryDefinition);
         cylinder.setMaterial(toMaterial(materialDefinition, resourceClassLoader));
         return cylinder;
      }
      else if (geometryDefinition instanceof Sphere3DDefinition)
      {
         Sphere sphere = toSphere((Sphere3DDefinition) geometryDefinition);
         sphere.setMaterial(toMaterial(materialDefinition, resourceClassLoader));
         return sphere;
      }
      else if (geometryDefinition instanceof ModelFileGeometryDefinition)
      {
         Node[] nodes = importModel((ModelFileGeometryDefinition) geometryDefinition, resourceClassLoader);
         if (nodes == null)
            return null;

         //         if (materialDefinition != null)
         //         {
         //            Material material = toMaterial(materialDefinition);
         //            for (Node node : nodes)
         //               ((Shape3D) node).setMaterial(material);
         //         }
         return nodes.length == 1 ? nodes[0] : new Group(nodes);
      }
      else
      {
         MeshView meshView = toMeshView(geometryDefinition);
         if (meshView != null)
         {
            meshView.setMaterial(toMaterial(materialDefinition, resourceClassLoader));
            return meshView;
         }
      }

      return DEFAULT_GEOMETRY;
   }

   public static MeshView toMeshView(GeometryDefinition geometryDefinition)
   {
      if (geometryDefinition == null)
         return null;

      TriangleMesh3DDefinition triangleMesh3D = TriangleMesh3DFactories.TriangleMesh(geometryDefinition);

      if (triangleMesh3D == null)
         return null;

      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(triangleMesh3D);
      MeshView meshView = new MeshView(mesh);
      return meshView;
   }

   public static Box toBox(Box3DDefinition geometryDefinition)
   {
      if (geometryDefinition == null)
         return DEFAULT_BOX;

      Box box = new Box(geometryDefinition.getSizeX(), geometryDefinition.getSizeY(), geometryDefinition.getSizeZ());
      if (!geometryDefinition.isCentered())
      {
         box.getTransforms().add(new Translate(0.0, 0.0, 0.5 * geometryDefinition.getSizeZ()));
      }
      return box;
   }

   public static Shape3D toCylinder(Cylinder3DDefinition geometryDefinition)
   {
      if (geometryDefinition == null)
         return DEFAULT_CYLINDER;

      Cylinder cylinder = new Cylinder(geometryDefinition.getRadius(), geometryDefinition.getLength());
      cylinder.getTransforms().add(new Rotate(90.0, Rotate.X_AXIS));
      return cylinder;
   }

   public static Sphere toSphere(Sphere3DDefinition geometryDefinition)
   {
      if (geometryDefinition == null)
         return DEFAULT_SPHERE;

      return new Sphere(geometryDefinition.getRadius());
   }

   public static Node[] importModel(ModelFileGeometryDefinition geometryDefinition, ClassLoader resourceClassLoader)
   {
      if (geometryDefinition == null || geometryDefinition.getFileName() == null)
         return DEFAULT_MESH_VIEWS;

      String filename = geometryDefinition.getFileName();

      if (resourceClassLoader == null)
         resourceClassLoader = JavaFXTools.class.getClassLoader();
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

      Node[] importedModel = importModel(fileURL);

      Vector3D scale = geometryDefinition.getScale();

      if (scale != null && importedModel != null && importedModel.length > 0)
      {
         if (importedModel.length == 1)
         {
            importedModel[0].getTransforms().add(new Scale(scale.getX(), scale.getY(), scale.getZ()));
         }
         else
         {
            Group group = new Group(importedModel);
            group.getTransforms().add(new Scale(scale.getX(), scale.getY(), scale.getZ()));
            importedModel = new Node[] {group};
         }
      }

      return importedModel;
   }

   public static Node[] importModel(URL fileURL)
   {
      try
      {
         String file = fileURL.getFile();
         String fileExtension = FilenameUtils.getExtension(file).toLowerCase();
         Node[] importedNodes;

         switch (fileExtension)
         {
            case "dae":
               importedNodes = JavaFXVisualTools.importColladaModel(fileURL);
               break;
            case "stl":
               importedNodes = JavaFXVisualTools.importSTLModel(fileURL);
               break;
            case "obj":
               importedNodes = JavaFXVisualTools.importOBJModel(fileURL);
               break;
            default:
               importedNodes = DEFAULT_MESH_VIEWS;
               break;
         }
         setNodeIDs(importedNodes, FilenameUtils.getBaseName(file), true);
         return importedNodes;
      }
      catch (Exception e)
      {
         LogTools.error("Could not import model file: " + fileURL.toExternalForm() + "\n " + e.getClass().getSimpleName() + ": " + e.getMessage());
         return null;
      }
   }

   public static Node[] importColladaModel(URL fileURL)
   {
      MeshView[] importedModel;
      CachedImportedModel cachedImportedModel = cachedColladaModels.get(fileURL.toExternalForm());

      if (cachedImportedModel == null)
      {
         ColModelImporter importer = new ColModelImporter();
         importer.getOptions().add(ColImportOption.IGNORE_LIGHTS);
         importer.getOptions().add(ColImportOption.IGNORE_CAMERAS);
         importer.getOptions().add(ColImportOption.GENERATE_NORMALS);
         importer.read(fileURL);

         ColAsset asset = importer.getAsset();
         importedModel = unwrapGroups(importer.getImport(), MeshView.class).toArray(new MeshView[0]);
         Stream.of(importedModel).forEach(model -> model.getTransforms().add(0, new Rotate(-90, Rotate.X_AXIS)));

         if (!EuclidCoreTools.epsilonEquals(1.0, asset.getUnitMeter(), 1.0e-3))
            Stream.of(importedModel)
                  .forEach(model -> model.getTransforms().add(0, new Scale(asset.getUnitMeter(), asset.getUnitMeter(), asset.getUnitMeter())));

         cacheMaterials(importer, importedModel, cachedNamedColladaMaterials);
         cachedColladaModels.put(fileURL.toExternalForm(), new CachedImportedModel(importedModel));
         importer.close();
      }
      else
      {
         importedModel = cachedImportedModel.newMeshViews();
      }

      return importedModel;
   }

   public static Node[] importSTLModel(URL fileURL)
   {
      StlMeshImporter importer = new StlMeshImporter();
      importer.read(fileURL);
      MeshView meshView = new MeshView(importer.getImport());
      meshView.getTransforms().add(ROTATE_PI_X);
      importer.close();
      return new Node[] {meshView};
   }

   public static Node[] importOBJModel(URL fileURL) throws URISyntaxException, IOException
   {
      MeshView[] importedModel;
      String externalForm = fileURL.toExternalForm();
      CachedImportedModel cachedImportedModel = cachedOBJModels.get(externalForm);

      if (cachedImportedModel == null)
      {
         ObjModelImporter importer = new ObjModelImporter();
         importer.setOptions(ObjImportOption.GENERATE_NORMALS);
         try
         {
            importer.read(fileURL);
         }
         catch (ImportException e)
         {
            if (e.getMessage().contains("Material not found"))
            {
               String filePath = fileURL.getPath();
               File tempFileWithoutMTL = new File(filePath.substring(0, filePath.lastIndexOf(".")) + "Temp.obj");
               if (tempFileWithoutMTL.exists())
               {
                  throw e;
               }
               tempFileWithoutMTL.createNewFile();
               BufferedReader bufferedReader = new BufferedReader(new FileReader(new File(fileURL.toURI())));
               BufferedWriter bufferedWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(tempFileWithoutMTL)));

               String line;
               while ((line = bufferedReader.readLine()) != null)
               {
                  if (line.toLowerCase().startsWith("mtllib"))
                     continue;
                  if (line.toLowerCase().startsWith("usemtl"))
                     continue;

                  try
                  {
                     bufferedWriter.write(line);
                     bufferedWriter.newLine();
                  }
                  catch (Exception e1)
                  {
                     tempFileWithoutMTL.delete();
                     bufferedReader.close();
                     bufferedWriter.close();
                     throw e1;
                  }
               }

               bufferedReader.close();
               bufferedWriter.close();

               try
               {
                  importer.close();
                  importer = new ObjModelImporter();
                  importer.setOptions(ObjImportOption.GENERATE_NORMALS);
                  importer.read(tempFileWithoutMTL);
               }
               catch (Exception e1)
               {
                  tempFileWithoutMTL.delete();
                  throw e1;
               }
               tempFileWithoutMTL.delete();
            }
         }

         importedModel = importer.getImport();
         Stream.of(importedModel).forEach(model -> model.getTransforms().add(ROTATE_PI_X));
         cacheMaterials(importer, importedModel, cachedNamedOBJMaterials);
         cachedOBJModels.put(fileURL.toExternalForm(), new CachedImportedModel(importedModel));

         importer.close();
      }
      else
      {
         importedModel = cachedImportedModel.newMeshViews();
      }

      return importedModel;
   }

   private static void cacheMaterials(ModelImporter importer, MeshView[] importedModels, Map<Long, PhongMaterial> cacheToUpdate)
   {
      Map<String, PhongMaterial> namedMaterials = importer.getNamedMaterials();

      if (namedMaterials == null)
         return;

      Map<Image, FilePath> imageFilePaths = importer.getImageFilePaths();

      Map<PhongMaterial, Long> materialToHashCodeMap = new HashMap<>();

      for (Entry<String, PhongMaterial> entry : namedMaterials.entrySet())
      {
         PhongMaterial material = entry.getValue();
         long materialHashCode = entry.getKey().hashCode();

         Image materialImage = material.getDiffuseMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();
         materialImage = material.getSpecularMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();
         materialImage = material.getBumpMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();
         materialImage = material.getSelfIlluminationMap();
         if (materialImage != null)
            materialHashCode = 31L * materialHashCode + imageFilePaths.get(materialImage).getAbsolutePath().hashCode();

         materialToHashCodeMap.put(material, materialHashCode);
         cacheToUpdate.putIfAbsent(materialHashCode, material);
      }

      for (MeshView importedModel : importedModels)
      {
         Long materialHashCode = materialToHashCodeMap.get(importedModel.getMaterial());
         importedModel.setMaterial(cacheToUpdate.get(materialHashCode));
      }
   }

   public static <T extends Node> List<T> unwrapGroups(Node[] nodes, Class<T> filterClass)
   {
      if (nodes == null || nodes.length == 0)
         return null;

      List<T> filteredNodes = new ArrayList<>();
      for (Node node : nodes)
         filteredNodes.addAll(unwrapGroups(node, filterClass));
      return filteredNodes;
   }

   public static <T extends Node> List<T> unwrapGroups(Node node, Class<T> filterClass)
   {
      if (!(node instanceof Group))
      {
         if (filterClass.isInstance(node))
            return Collections.singletonList(filterClass.cast(node));
         else
            return Collections.emptyList();
      }

      Group group = (Group) node;

      if (group.getChildren().isEmpty())
         return Collections.emptyList();

      List<T> filteredNodes = new ArrayList<>();

      for (Node child : group.getChildren())
      {
         child.getTransforms().addAll(0, group.getTransforms());
         filteredNodes.addAll(unwrapGroups(child, filterClass));
      }

      return filteredNodes;
   }

   public static Material toMaterial(MaterialDefinition materialDefinition, ClassLoader resourceClassLoader)
   {
      if (materialDefinition == null)
         return DEFAULT_MATERIAL;

      PhongMaterial phongMaterial = new PhongMaterial();

      ColorDefinition diffuseColor = materialDefinition.getDiffuseColor();
      // TODO Not sure what to do with these 2
      //      ColorDefinition ambientColor = materialDefinition.getAmbientColor();
      //      ColorDefinition emissiveColor = materialDefinition.getEmissiveColor();
      ColorDefinition specularColor = materialDefinition.getSpecularColor();

      TextureDefinition diffuseMap = materialDefinition.getDiffuseMap();
      TextureDefinition emissiveMap = materialDefinition.getEmissiveMap();
      TextureDefinition normalMap = materialDefinition.getNormalMap();
      TextureDefinition specularMap = materialDefinition.getSpecularMap();

      boolean atLeastOneFieldSet = false;

      if (diffuseColor != null)
      {
         phongMaterial.setDiffuseColor(toColor(diffuseColor));
         atLeastOneFieldSet = true;
      }
      if (specularColor != null)
      {
         phongMaterial.setSpecularColor(toColor(specularColor));
         phongMaterial.setSpecularPower(materialDefinition.getShininess());
         atLeastOneFieldSet = true;
      }

      if (diffuseMap != null)
      {
         phongMaterial.setDiffuseMap(toImage(diffuseMap, resourceClassLoader));
         atLeastOneFieldSet = true;
      }
      if (emissiveMap != null)
      {
         phongMaterial.setSelfIlluminationMap(toImage(emissiveMap, resourceClassLoader));
         atLeastOneFieldSet = true;
      }
      if (normalMap != null)
      {
         phongMaterial.setBumpMap(toImage(normalMap, resourceClassLoader));
         atLeastOneFieldSet = true;
      }
      if (specularMap != null)
      {
         phongMaterial.setSpecularMap(toImage(specularMap, resourceClassLoader));
         phongMaterial.setSpecularPower(materialDefinition.getShininess());
         atLeastOneFieldSet = true;
      }

      return atLeastOneFieldSet ? phongMaterial : DEFAULT_MATERIAL;
   }

   public static Image toImage(TextureDefinition textureDefinition, ClassLoader resourceClassLoader)
   {
      try
      {
         if (textureDefinition == null)
            return null;
         if (textureDefinition.getImage() != null)
            return SwingFXUtils.toFXImage(textureDefinition.getImage(), null);
         if (textureDefinition.getFileURL() != null)
            return new Image(textureDefinition.getFileURL().openStream());
         if (textureDefinition.getFilename() != null)
         {
            if (resourceClassLoader != null)
               return new Image(resourceClassLoader.getResourceAsStream(textureDefinition.getFilename()));
            else
               return new Image(textureDefinition.getFilename());
         }
         return null;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public static Color toColor(ColorDefinition colorDefinition)
   {
      return toColor(colorDefinition, DEFAULT_COLOR);
   }

   public static Color toColor(ColorDefinition colorDefinition, Color defaultValue)
   {
      if (colorDefinition == null)
         return defaultValue;
      else
         return Color.color(colorDefinition.getRed(), colorDefinition.getGreen(), colorDefinition.getBlue(), colorDefinition.getAlpha());
   }

   public static final List<String> colorNameList;

   static
   {
      List<String> list = Stream.of(Color.class.getDeclaredFields()).filter(field -> Modifier.isStatic(field.getModifiers()))
                                .filter(field -> Modifier.isPublic(field.getModifiers())).filter(field -> field.getType() == Color.class)
                                .map(field -> field.getName().toLowerCase()).collect(Collectors.toList());
      colorNameList = Collections.unmodifiableList(list);
   }

   public static void setNodeIDs(Node[] nodes, String id, boolean overrideExistingIDs)
   {
      if (nodes == null || nodes.length == 0)
         return;
      if (nodes.length == 1)
      {
         setNodeIDs(nodes[0], id, overrideExistingIDs);
      }
      else
      {
         for (int i = 0; i < nodes.length; i++)
         {
            setNodeIDs(nodes[i], id + "_" + i, overrideExistingIDs);
         }
      }
   }

   public static void setMissingNodeIDs(Node node, String id)
   {
      setNodeIDs(node, id, false);
   }

   public static void setNodeIDs(Node node, String id, boolean overrideExistingIDs)
   {
      if (node == null)
         return;

      if (node.getId() == null || overrideExistingIDs)
         node.setId(id);

      if (node instanceof Group)
      {
         Group group = (Group) node;

         for (int i = 0; i < group.getChildren().size(); i++)
         {
            setNodeIDs(group.getChildren().get(i), id + "_" + i, overrideExistingIDs);
         }
      }
   }
}
