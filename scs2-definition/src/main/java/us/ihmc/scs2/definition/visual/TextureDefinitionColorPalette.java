package us.ihmc.scs2.definition.visual;

import us.ihmc.euclid.tuple2D.Point2D32;

/**
 * {@code TextureColorPalette} defines a tool that maps texture coordinates to an image. The main
 * usage is for {@link MultiColorTriangleMesh3DBuilder} that re-maps texture coordinates of the mesh
 * vertices to point to the image of the active {@link TextureDefinitionColorPalette} and to allow
 * the user to render a mesh with more than one color.
 * 
 * @author Sylvain Bertrand
 */
public interface TextureDefinitionColorPalette
{
   /**
    * Retrieves the texture coordinates of a given {@link ColorDefinition} in the image of this
    * {@link TextureDefinitionColorPalette}.
    * 
    * @param color the color to retrieve the texture coordinates of.
    * @return the corresponding texture coordinates.
    */
   Point2D32 getTextureLocation(ColorDefinition color);

   /**
    * @return the image to use with the texture coordinates computed.
    */
   TextureDefinition getTextureDefinition();
}