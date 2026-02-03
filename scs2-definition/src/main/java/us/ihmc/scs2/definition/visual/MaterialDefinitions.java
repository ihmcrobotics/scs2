package us.ihmc.scs2.definition.visual;

public final class MaterialDefinitions
{
   public static MaterialDefinition AluminumMaterial()
   {
      MaterialDefinition mat = new MaterialDefinition();
      mat.setSpecularColor(new ColorDefinition(0.5f, 0.5f, 0.5f));
      mat.setDiffuseColor(new ColorDefinition(0.2f, 0.4f, 0.5f));
      mat.setShininess(7.5f);
      mat.setAmbientColor(new ColorDefinition(0.17f, 0.5f, 0.7f));

      return mat;
   }

   public static MaterialDefinition BlackMetalMaterial()
   {
      MaterialDefinition mat = new MaterialDefinition();
      mat.setSpecularColor(new ColorDefinition(0.5f, 0.5f, 0.5f));
      mat.setDiffuseColor(new ColorDefinition(0.2f, 0.4f, 0.5f));
      mat.setShininess(6.0f);
      mat.setAmbientColor(new ColorDefinition(0.16f, 0.18f, 0.2f));

      return mat;
   }
}
