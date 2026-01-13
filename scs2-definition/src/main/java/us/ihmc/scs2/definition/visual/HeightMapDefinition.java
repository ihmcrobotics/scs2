package us.ihmc.scs2.definition.visual;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface HeightMapDefinition
{
   BoundingBox2DReadOnly getBoundingBox();

   double getHeightAt(double x, double y);

   Vector3DReadOnly getNormalAt(double x, double y);
}
