package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

import us.ihmc.scs2.definition.robot.sdf.items.SDFURIHolder.SimpleSDFURIHolder;

public class SDFVisual implements SDFItem
{
   private String name;
   private String castShadows;
   private String laserRetro;
   private String transparency;
   private String pose;
   private SDFGeometry geometry;
   private SDFMaterial material;

   public String getName()
   {
      return name;
   }

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   public String getCastShadows()
   {
      return castShadows;
   }

   @XmlElement(name = "cast_shadows")
   public void setCastShadows(String castShadows)
   {
      this.castShadows = castShadows;
   }

   public String getLaserRetro()
   {
      return laserRetro;
   }

   @XmlElement(name = "laser_retro")
   public void setLaserRetro(String laserRetro)
   {
      this.laserRetro = laserRetro;
   }

   public String getTransparency()
   {
      return transparency;
   }

   @XmlElement(name = "transparency")
   public void setTransparency(String transparency)
   {
      this.transparency = transparency;
   }

   public String getPose()
   {
      return pose;
   }

   @XmlElement(name = "pose")
   public void setPose(String pose)
   {
      this.pose = pose;
   }

   public SDFGeometry getGeometry()
   {
      return geometry;
   }

   @XmlElement(name = "geometry")
   public void setGeometry(SDFGeometry geometry)
   {
      this.geometry = geometry;
   }

   public SDFMaterial getMaterial()
   {
      return material;
   }

   @XmlElement(name = "material")
   public void setMaterial(SDFMaterial material)
   {
      this.material = material;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, castShadows: %s, laserRetro: %s, transparency: %s, pose: %s, geometry: %s, material: %s]",
                    name,
                    castShadows,
                    laserRetro,
                    transparency,
                    pose,
                    geometry,
                    material);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemURIHolders(geometry, material);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class SDFMaterial implements SDFItem
   {
      private SDFScript script;

      private String lighting;
      private String ambient;
      private String diffuse;
      private String specular;
      private String emissive;

      public SDFScript getScript()
      {
         return script;
      }

      @XmlElement(name = "script")
      public void setScript(SDFScript script)
      {
         this.script = script;
      }

      public String getLighting()
      {
         return lighting;
      }

      public String getAmbient()
      {
         return ambient;
      }

      public String getDiffuse()
      {
         return diffuse;
      }

      public String getSpecular()
      {
         return specular;
      }

      public String getEmissive()
      {
         return emissive;
      }

      @XmlElement(name = "lighting")
      public void setLighting(String lighting)
      {
         this.lighting = lighting;
      }

      @XmlElement(name = "ambient")
      public void setAmbient(String ambient)
      {
         this.ambient = ambient;
      }

      @XmlElement(name = "diffuse")
      public void setDiffuse(String diffuse)
      {
         this.diffuse = diffuse;
      }

      @XmlElement(name = "specular")
      public void setSpecular(String specular)
      {
         this.specular = specular;
      }

      @XmlElement(name = "emissive")
      public void setEmissive(String emissive)
      {
         this.emissive = emissive;
      }

      @Override
      public String getContentAsString()
      {
         return format("[script: %s, lighting: %s, ambient: %s, diffuse: %s, specular: %s, emissive: %s]",
                       script,
                       lighting,
                       ambient,
                       diffuse,
                       specular,
                       emissive);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return SDFItem.combineItemURIHolders(script);
      }

      @Override
      public String toString()
      {
         return itemToString();
      }

      public static class SDFScript implements SDFItem
      {
         private String name;
         private List<SimpleSDFURIHolder> uris;

         public String getName()
         {
            return name;
         }

         @XmlElement(name = "name")
         public void setName(String name)
         {
            this.name = name;
         }

         public List<SimpleSDFURIHolder> getUris()
         {
            return uris;
         }

         @XmlElement(name = "uri")
         public void setUri(List<String> uri)
         {
            this.uris = SDFURIHolder.toSimpleSDFURIHolders(uri);
         }

         @Override
         public String getContentAsString()
         {
            return format("[name: %s, uris: %s]", name, uris);
         }

         @Override
         public List<SimpleSDFURIHolder> getURIHolders()
         {
            return uris;
         }

         @Override
         public String toString()
         {
            return itemToString();
         }
      }
   }
}