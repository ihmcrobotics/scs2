package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlElement;

public class SDFGeometry implements SDFItem
{
   private Box box;
   private Sphere sphere;
   private Cylinder cylinder;
   private Mesh mesh;
   private Plane plane;
   private GeometryImage image;
   private HeightMap heightMap;

   public Box getBox()
   {
      return box;
   }

   @XmlElement(name = "box")
   public void setBox(Box box)
   {
      this.box = box;
   }

   public Sphere getSphere()
   {
      return sphere;
   }

   @XmlElement(name = "sphere")
   public void setSphere(Sphere sphere)
   {
      this.sphere = sphere;
   }

   public Cylinder getCylinder()
   {
      return cylinder;
   }

   @XmlElement(name = "cylinder")
   public void setCylinder(Cylinder cylinder)
   {
      this.cylinder = cylinder;
   }

   public Mesh getMesh()
   {
      return mesh;
   }

   @XmlElement(name = "mesh")
   public void setMesh(Mesh mesh)
   {
      this.mesh = mesh;
   }

   public Plane getPlane()
   {
      return plane;
   }

   @XmlElement(name = "plane")
   public void setPlane(Plane plane)
   {
      this.plane = plane;
   }

   public GeometryImage getImage()
   {
      return image;
   }

   @XmlElement(name = "image")
   public void setImage(GeometryImage image)
   {
      this.image = image;
   }

   public HeightMap getHeightMap()
   {
      return heightMap;
   }

   @XmlElement(name = "heightmap")
   public void setHeightMap(HeightMap heightMap)
   {
      this.heightMap = heightMap;
   }

   @Override
   public String getContentAsString()
   {
      return format("[box: %s, sphere: %s, cylinder: %s, mesh: %s, plane: %s, image: %s, heightMap: %s]", box, sphere, cylinder, mesh, plane, image, heightMap);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemURIHolders(box, sphere, cylinder, mesh, plane, image, heightMap);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class Box implements SDFItem
   {
      private String size;

      public String getSize()
      {
         return size;
      }

      @XmlElement(name = "size")
      public void setSize(String size)
      {
         this.size = size;
      }

      @Override
      public String getContentAsString()
      {
         return format("[size: %s]", size);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }
   }

   public static class Sphere implements SDFItem
   {
      private String radius;

      public String getRadius()
      {
         return radius;
      }

      @XmlElement(name = "radius")
      public void setRadius(String radius)
      {
         this.radius = radius;
      }

      @Override
      public String getContentAsString()
      {
         return format("[radius: %s]", radius);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }
   }

   public static class Cylinder implements SDFItem
   {
      private String radius;
      private String length;

      public String getRadius()
      {
         return radius;
      }

      @XmlElement(name = "radius")
      public void setRadius(String radius)
      {
         this.radius = radius;
      }

      public String getLength()
      {
         return length;
      }

      @XmlElement(name = "length")
      public void setLength(String length)
      {
         this.length = length;
      }

      @Override
      public String getContentAsString()
      {
         return format("[radius: %s, length: %s]", radius, length);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }
   }

   public static class Mesh implements SDFItem, SDFURIHolder
   {
      private String uri;
      private String scale;
      private SubMesh submesh;

      @Override
      public String getUri()
      {
         return uri;
      }

      @Override
      @XmlElement(name = "uri")
      public void setUri(String uri)
      {
         this.uri = uri;
      }

      public String getScale()
      {
         return scale;
      }

      @XmlElement(name = "scale")
      public void setScale(String scale)
      {
         this.scale = scale;
      }

      public SubMesh getSubmesh()
      {
         return submesh;
      }

      @XmlElement(name = "submesh")
      public void setSubmesh(SubMesh submesh)
      {
         this.submesh = submesh;
      }

      @Override
      public String getContentAsString()
      {
         return format("[uri: %s, scale: %s, submesh: %s]", uri, scale, submesh);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.singletonList(this);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class SubMesh implements SDFItem
      {
         private String name;
         private String center;

         public String getName()
         {
            return name;
         }

         public String getCenter()
         {
            return center;
         }

         @XmlElement(name = "name")
         public void setName(String name)
         {
            this.name = name;
         }

         @XmlElement(name = "center")
         public void setCenter(String center)
         {
            this.center = center;
         }

         @Override
         public String getContentAsString()
         {
            return format("[name: %s, center: %s]", name, center);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }
   }

   public static class Plane implements SDFItem
   {
      private String normal;
      private String size;

      public String getNormal()
      {
         return normal;
      }

      @XmlElement(name = "normal")
      public void setNormal(String normal)
      {
         this.normal = normal;
      }

      public String getSize()
      {
         return size;
      }

      @XmlElement(name = "size")
      public void setSize(String size)
      {
         this.size = size;
      }

      @Override
      public String getContentAsString()
      {
         return format("[normal: %s, size: %s]", normal, size);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }
   }

   public static class GeometryImage implements SDFItem, SDFURIHolder
   {
      private String uri;
      private String scale;
      private String threshold;
      private String height;
      private String granularity;

      @Override
      public String getUri()
      {
         return uri;
      }

      @Override
      @XmlElement(name = "uri")
      public void setUri(String uri)
      {
         this.uri = uri;
      }

      public String getScale()
      {
         return scale;
      }

      @XmlElement(name = "scale")
      public void setScale(String scale)
      {
         this.scale = scale;
      }

      public String getThreshold()
      {
         return threshold;
      }

      @XmlElement(name = "threshold")
      public void setThreshold(String threshold)
      {
         this.threshold = threshold;
      }

      public String getHeight()
      {
         return height;
      }

      @XmlElement(name = "height")
      public void setHeight(String height)
      {
         this.height = height;
      }

      public String getGranularity()
      {
         return granularity;
      }

      @XmlElement(name = "granularity")
      public void setGranularity(String granularity)
      {
         this.granularity = granularity;
      }

      @Override
      public String getContentAsString()
      {
         return format("[uri: %s, scale: %s, threshold: %s, height: %s, granularity: %s]", uri, scale, threshold, height, granularity);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.singletonList(this);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }
   }

   public static class HeightMap implements SDFItem, SDFURIHolder
   {
      private String uri;
      private String size;
      private String pos;

      private List<Texture> textures;
      private List<Blend> blends;

      public String getUri()
      {
         return uri;
      }

      @XmlElement(name = "uri")
      public void setUri(String uri)
      {
         this.uri = uri;
      }

      public String getSize()
      {
         return size;
      }

      @XmlElement(name = "size")
      public void setSize(String size)
      {
         this.size = size;
      }

      public String getPos()
      {
         return pos;
      }

      @XmlElement(name = "pos")
      public void setPos(String pos)
      {
         this.pos = pos;
      }

      public List<Texture> getTextures()
      {
         return textures;
      }

      @XmlElement(name = "texture")
      public void setTextures(List<Texture> textures)
      {
         this.textures = textures;
      }

      public List<Blend> getBlends()
      {
         return blends;
      }

      @XmlElement(name = "blend")
      public void setBlends(List<Blend> blends)
      {
         this.blends = blends;
      }

      @Override
      public String getContentAsString()
      {
         return format("[uri: %s, size: %s, pos: %s, textures: %s, blends: %s]", uri, size, pos, textures, blends);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.singletonList(this);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class Texture implements SDFItem
      {
         private String size;
         private String diffuse;
         private String normal;

         public String getSize()
         {
            return size;
         }

         @XmlElement(name = "size")
         public void setSize(String size)
         {
            this.size = size;
         }

         public String getDiffuse()
         {
            return diffuse;
         }

         @XmlElement(name = "diffuse")
         public void setDiffuse(String diffuse)
         {
            this.diffuse = diffuse;
         }

         public String getNormal()
         {
            return normal;
         }

         @XmlElement(name = "normal")
         public void setNormal(String normal)
         {
            this.normal = normal;
         }

         @Override
         public String getContentAsString()
         {
            return format("[size: %s, diffuse: %s, normal: %s]", size, diffuse, normal);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }

      public static class Blend implements SDFItem
      {
         private String minHeight;
         private String fadeDist;

         public String getMinHeight()
         {
            return minHeight;
         }

         @XmlElement(name = "min_height")
         public void setMinHeight(String minHeight)
         {
            this.minHeight = minHeight;
         }

         public String getFadeDist()
         {
            return fadeDist;
         }

         @XmlElement(name = "fade_dist")
         public void setFadeDist(String fadeDist)
         {
            this.fadeDist = fadeDist;
         }

         @Override
         public String getContentAsString()
         {
            return format("[minHeight: %s, fadeDist: %s]", minHeight, fadeDist);
         }

         @Override
         public List<SDFURIHolder> getURIHolders()
         {
            return Collections.emptyList();
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }
   }
}