package us.ihmc.scs2.definition.geometry;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlTransient;

import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Definition for creating a geometry from model file.
 */
public class ModelFileGeometryDefinition extends GeometryDefinition
{
   private String fileName;
   private List<SubMeshDefinition> submeshes;
   private List<String> resourceDirectories;
   private ClassLoader resourceClassLoader;
   private Vector3D scale;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public ModelFileGeometryDefinition()
   {
   }

   /**
    * Creates and initializes a definition for loading a model file.
    * 
    * @param fileName the path to the model file.
    */
   public ModelFileGeometryDefinition(String fileName)
   {
      setFileName(fileName);
   }

   public ModelFileGeometryDefinition(ModelFileGeometryDefinition other)
   {
      setName(other.getName());
      fileName = other.fileName;
      if (other.submeshes != null)
         submeshes = other.submeshes.stream().map(SubMeshDefinition::new).collect(Collectors.toList());
      if (other.resourceDirectories != null)
         resourceDirectories = new ArrayList<>(other.resourceDirectories);
      resourceClassLoader = other.resourceClassLoader;
      if (other.scale != null)
         scale = new Vector3D(other.scale);
   }

   /**
    * Sets the path to the model file.
    * 
    * @param fileName the path to the model file.
    */
   @XmlElement
   public void setFileName(String fileName)
   {
      this.fileName = fileName;
   }

   /**
    * Sets the list of sub-meshes to be loaded together with the main model file.
    * 
    * @param submeshes the list of the sub-meshes.
    */
   @XmlElement(name = "submesh")
   public void setSubmeshes(List<SubMeshDefinition> submeshes)
   {
      this.submeshes = submeshes;
   }

   /**
    * Sets the list of directories containing resources that may be needed to load the model file.
    * 
    * @param resourceDirectories the list of the directories containing resources needed for the
    *                            loading the model file.
    */
   @XmlElement
   public void setResourceDirectories(List<String> resourceDirectories)
   {
      this.resourceDirectories = resourceDirectories;
   }

   /**
    * Sets the class loader to use for loading the resource(s).
    * 
    * @param resourceClassLoader the class loader to use with this model file.
    */
   @XmlTransient
   public void setResourceClassLoader(ClassLoader resourceClassLoader)
   {
      this.resourceClassLoader = resourceClassLoader;
   }

   /**
    * Sets the scale to apply to the loaded mesh.
    * 
    * @param scale scale to apply to the loaded mesh.
    */
   @XmlElement
   public void setScale(Vector3D scale)
   {
      this.scale = scale;
   }

   /**
    * Returns the path to the model file.
    * 
    * @return the path to the model file.
    */
   public String getFileName()
   {
      return fileName;
   }

   /**
    * Returns the list of sub-meshes to be loaded together with the main model file.
    * 
    * @return the list of sub-meshes to be loaded together with the main model file.
    */
   public List<SubMeshDefinition> getSubmeshes()
   {
      return submeshes;
   }

   /**
    * Returns the list of directories containing resources that may be needed to load the model file.
    * 
    * @return the list of directories containing resources that may be needed to load the model file.
    */
   public List<String> getResourceDirectories()
   {
      return resourceDirectories;
   }

   /**
    * Returns the class loader to use for loading the resource(s).
    * 
    * @return the class loader to use for loading the resource(s).
    */
   public ClassLoader getResourceClassLoader()
   {
      return resourceClassLoader;
   }

   /**
    * Returns the scale to apply to the loaded mesh.
    * 
    * @return the scale to apply to the loaded mesh.
    */
   public Vector3D getScale()
   {
      return scale;
   }

   @Override
   public ModelFileGeometryDefinition copy()
   {
      return new ModelFileGeometryDefinition(this);
   }

   /**
    * Definition of a sub-mesh to be loaded together with a main mesh.
    */
   public static class SubMeshDefinition
   {
      private String name;
      private boolean center;

      /**
       * Creates an empty definition. The parameters have to be all set before this definition can be
       * used.
       */
      public SubMeshDefinition()
      {
      }

      /**
       * Creates and initializes a definition for loading a sub-mesh.
       * 
       * @param name   the name of the sub-mesh.
       * @param center whether to center this sub-mesh vertices at the origin or load the sub-mesh as is.
       */
      public SubMeshDefinition(String name, boolean center)
      {
         this.name = name;
         this.center = center;
      }

      public SubMeshDefinition(SubMeshDefinition other)
      {
         name = other.name;
         center = other.center;
      }

      /**
       * Sets the name of the sub-mesh.
       * 
       * @param name the name of the sub-mesh.
       */
      @XmlElement
      public void setName(String name)
      {
         this.name = name;
      }

      /**
       * Sets whether the vertices of this sub-mesh should be centered at (0, 0, 0) which would
       * effectively remove any transformations specific to the sub-mesh.
       * 
       * @param center whether to center this sub-mesh vertices at the origin or load the sub-mesh as is.
       */
      @XmlElement
      public void setCenter(boolean center)
      {
         this.center = center;
      }

      /**
       * Returns the name of this sub-mesh.
       * 
       * @return the name of this sub-mesh.
       */
      public String getName()
      {
         return name;
      }

      /**
       * Returns whether the vertices of this sub-mesh should be centered at the origin or if the sub-mesh
       * should be loaded as is.
       * 
       * @return whether the vertices of this sub-mesh should be centered at the origin or if the sub-mesh
       *         should be loaded as is.
       */
      public boolean getCenter()
      {
         return center;
      }

      public SubMeshDefinition copy()
      {
         return new SubMeshDefinition(this);
      }
   }
}
