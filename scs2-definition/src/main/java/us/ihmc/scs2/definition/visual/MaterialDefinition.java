package us.ihmc.scs2.definition.visual;

import java.util.Objects;

import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class MaterialDefinition
{
   private String name;
   private ColorDefinition ambientColor;
   private ColorDefinition diffuseColor;
   private ColorDefinition specularColor;
   private ColorDefinition emissiveColor;

   private double shininess = Double.NaN;

   private TextureDefinition diffuseMap;
   private TextureDefinition normalMap;
   private TextureDefinition specularMap;
   private TextureDefinition emissiveMap;

   private MaterialScriptDefinition scriptDefinition;

   public MaterialDefinition()
   {
   }

   public MaterialDefinition(TextureDefinition diffuseMap)
   {
      this.diffuseMap = diffuseMap;
   }

   public MaterialDefinition(ColorDefinition diffuseColor)
   {
      this.diffuseColor = diffuseColor;
   }

   public MaterialDefinition(ColorDefinition ambientColor,
                             ColorDefinition diffuseColor,
                             ColorDefinition specularColor,
                             ColorDefinition emissiveColor,
                             double shininess)
   {
      this.ambientColor = ambientColor;
      this.diffuseColor = diffuseColor;
      this.specularColor = specularColor;
      this.emissiveColor = emissiveColor;
      this.shininess = shininess;
   }

   public MaterialDefinition(MaterialDefinition other)
   {
      set(other);
   }

   public void set(MaterialDefinition other)
   {
      name = other.name;
      ambientColor = other.ambientColor == null ? null : other.ambientColor.copy();
      diffuseColor = other.diffuseColor == null ? null : other.diffuseColor.copy();
      specularColor = other.specularColor == null ? null : other.specularColor.copy();
      emissiveColor = other.emissiveColor == null ? null : other.emissiveColor.copy();

      shininess = other.shininess;

      diffuseMap = other.diffuseMap == null ? null : other.diffuseMap.copy();
      normalMap = other.normalMap == null ? null : other.normalMap.copy();
      specularMap = other.specularMap == null ? null : other.specularMap.copy();
      emissiveMap = other.emissiveMap == null ? null : other.emissiveMap.copy();

      scriptDefinition = other.scriptDefinition == null ? null : other.scriptDefinition.copy();
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setAmbientColor(ColorDefinition ambientColor)
   {
      this.ambientColor = ambientColor;
   }

   public void setDiffuseColor(ColorDefinition diffuseColor)
   {
      this.diffuseColor = diffuseColor;
   }

   public void setSpecularColor(ColorDefinition specularColor)
   {
      this.specularColor = specularColor;
   }

   public void setEmissiveColor(ColorDefinition emissiveColor)
   {
      this.emissiveColor = emissiveColor;
   }

   public void setShininess(double shininess)
   {
      this.shininess = shininess;
   }

   public void setDiffuseMap(TextureDefinition diffuseMap)
   {
      this.diffuseMap = diffuseMap;
   }

   public void setNormalMap(TextureDefinition normalMap)
   {
      this.normalMap = normalMap;
   }

   public void setSpecularMap(TextureDefinition specularMap)
   {
      this.specularMap = specularMap;
   }

   public void setEmissiveMap(TextureDefinition emissiveMap)
   {
      this.emissiveMap = emissiveMap;
   }

   public void setScriptDefinition(MaterialScriptDefinition scriptDefinition)
   {
      this.scriptDefinition = scriptDefinition;
   }

   public String getName()
   {
      return name;
   }

   public ColorDefinition getAmbientColor()
   {
      return ambientColor;
   }

   public ColorDefinition getDiffuseColor()
   {
      return diffuseColor;
   }

   public ColorDefinition getSpecularColor()
   {
      return specularColor;
   }

   public ColorDefinition getEmissiveColor()
   {
      return emissiveColor;
   }

   public double getShininess()
   {
      return shininess;
   }

   public TextureDefinition getDiffuseMap()
   {
      return diffuseMap;
   }

   public TextureDefinition getNormalMap()
   {
      return normalMap;
   }

   public TextureDefinition getSpecularMap()
   {
      return specularMap;
   }

   public TextureDefinition getEmissiveMap()
   {
      return emissiveMap;
   }

   public MaterialScriptDefinition getScriptDefinition()
   {
      return scriptDefinition;
   }

   public MaterialDefinition copy()
   {
      return new MaterialDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, name);
      bits = EuclidHashCodeTools.addToHashCode(bits, ambientColor);
      bits = EuclidHashCodeTools.addToHashCode(bits, diffuseColor);
      bits = EuclidHashCodeTools.addToHashCode(bits, specularColor);
      bits = EuclidHashCodeTools.addToHashCode(bits, emissiveColor);
      bits = EuclidHashCodeTools.addToHashCode(bits, shininess);
      bits = EuclidHashCodeTools.addToHashCode(bits, diffuseMap);
      bits = EuclidHashCodeTools.addToHashCode(bits, normalMap);
      bits = EuclidHashCodeTools.addToHashCode(bits, specularMap);
      bits = EuclidHashCodeTools.addToHashCode(bits, emissiveMap);
      bits = EuclidHashCodeTools.addToHashCode(bits, scriptDefinition);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      MaterialDefinition other = (MaterialDefinition) object;

      if (!Objects.equals(name, other.name))
         return false;

      if (!Objects.equals(ambientColor, other.ambientColor))
         return false;
      if (!Objects.equals(diffuseColor, other.diffuseColor))
         return false;
      if (!Objects.equals(specularColor, other.specularColor))
         return false;
      if (!Objects.equals(emissiveColor, other.emissiveColor))
         return false;

      if (Double.doubleToLongBits(shininess) == Double.doubleToLongBits(other.shininess))
         return false;

      if (!Objects.equals(diffuseMap, other.diffuseMap))
         return false;
      if (!Objects.equals(normalMap, other.normalMap))
         return false;
      if (!Objects.equals(specularMap, other.specularMap))
         return false;
      if (!Objects.equals(emissiveMap, other.emissiveMap))
         return false;

      if (!Objects.equals(scriptDefinition, other.scriptDefinition))
         return false;

      return true;
   }

}