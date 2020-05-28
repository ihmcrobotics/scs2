package us.ihmc.scs2;

import java.util.List;

import us.ihmc.scs2.definition.geometry.BoxGeometryDefinition;
import us.ihmc.scs2.definition.geometry.ConeGeometryDefinition;
import us.ihmc.scs2.definition.geometry.CylinderGeometryDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.SphereGeometryDefinition;
import us.ihmc.scs2.definition.geometry.TorusGeometryDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.protobuf.ThreeProto;
import us.ihmc.scs2.protobuf.ThreeProto.Mesh.Builder;

public class ThreeProtoTools
{
   public static ThreeProto.Color toProtoColor(ColorDefinition colorDefinition)
   {
      if (colorDefinition == null)
         return null;
      String webColor = String.format("rgba(%d, %d, %d, %d)",
                                      (int) (colorDefinition.getRed() * 255.0),
                                      (int) (colorDefinition.getGreen() * 255.0),
                                      (int) (colorDefinition.getBlue() * 255.0),
                                      (int) (colorDefinition.getAlpha() * 255.0));
      return ThreeProto.Color.newBuilder().setWebcolor(webColor).build();
   }

   public static ThreeProto.BoxGeometry toProtoBoxGeometry(BoxGeometryDefinition boxGeometryDefinition)
   {
      if (boxGeometryDefinition == null)
         return null;
      return ThreeProto.BoxGeometry.newBuilder().setSize(EuclidProtoTools.toProtoVector3D(boxGeometryDefinition.getSize())).build();
   }

   public static ThreeProto.ConeGeometry toProtoConeGeometry(ConeGeometryDefinition coneGeometryDefinition)
   {
      if (coneGeometryDefinition == null)
         return null;
      return ThreeProto.ConeGeometry.newBuilder().setRadius(coneGeometryDefinition.getRadius()).setHeight(coneGeometryDefinition.getHeight())
                                    .setRadialSegments(32).build();
   }

   public static ThreeProto.CylinderGeometry toProtoCylinderGeometry(CylinderGeometryDefinition cylinderGeometryDefinition)
   {
      if (cylinderGeometryDefinition == null)
         return null;
      return ThreeProto.CylinderGeometry.newBuilder().setRadiusTop(cylinderGeometryDefinition.getRadius())
                                        .setRadiusBottom(cylinderGeometryDefinition.getRadius()).setHeight(cylinderGeometryDefinition.getLength())
                                        .setRadialSegments(32).build();
   }

   public static ThreeProto.SphereGeometry toProtoSphereGeometry(SphereGeometryDefinition sphereGeometryDefinition)
   {
      if (sphereGeometryDefinition == null)
         return null;
      return ThreeProto.SphereGeometry.newBuilder().setRadius(sphereGeometryDefinition.getRadius()).setWidthSegments(32).setHeightSegments(32).build();
   }

   public static ThreeProto.TorusGeometry toProtoTorusGeometry(TorusGeometryDefinition torusGeometryDefinition)
   {
      if (torusGeometryDefinition == null)
         return null;
      return ThreeProto.TorusGeometry.newBuilder().setRadius(torusGeometryDefinition.getMajorRadius()).setTube(torusGeometryDefinition.getMinorRadius())
                                     .setRadialSegments(32).setTubularSegments(32).build();
   }

   public static ThreeProto.ModelFileGeometry toProtoModelFileGeometry(ModelFileGeometryDefinition modelFileGeometryDefinition)
   {
      if (modelFileGeometryDefinition == null)
         return null;
      ThreeProto.ModelFileGeometry.Builder newBuilder = ThreeProto.ModelFileGeometry.newBuilder();
      newBuilder.setFilename(modelFileGeometryDefinition.getFileName());
      modelFileGeometryDefinition.getSubmeshes().forEach(subMesh -> newBuilder.addSubmeshes(toProtoSubMesh(subMesh)));
      modelFileGeometryDefinition.getResourceDirectories().forEach(resourceDirectory -> newBuilder.addResourceDirectories(resourceDirectory));
      newBuilder.setScale(EuclidProtoTools.toProtoVector3D(modelFileGeometryDefinition.getScale()));
      return newBuilder.build();
   }

   public static ThreeProto.ModelFileGeometry.SubMesh toProtoSubMesh(ModelFileGeometryDefinition.SubMesh subMesh)
   {
      if (subMesh == null)
         return null;
      return ThreeProto.ModelFileGeometry.SubMesh.newBuilder().setName(subMesh.getName()).setCenter(subMesh.getCenter()).build();
   }

   public static ThreeProto.Mesh toProtoMesh(VisualDefinition visualDefinition)
   {
      if (visualDefinition == null)
         return null;
      Builder protoMeshBuilder = ThreeProto.Mesh.newBuilder();
      if (visualDefinition.getName() != null)
         protoMeshBuilder.setMeshId(visualDefinition.getName());
      protoMeshBuilder.setColor(toProtoColor(visualDefinition.getMaterialDefinition().getDiffuseColorDefinition()));
      if (visualDefinition.getOriginPose() != null)
         protoMeshBuilder.setPose(EuclidProtoTools.toProtoPose3D(visualDefinition.getOriginPose()));

      GeometryDefinition geometryDefinition = visualDefinition.getGeometryDefinition();
      if (geometryDefinition instanceof BoxGeometryDefinition)
         protoMeshBuilder.setBoxGeometry(toProtoBoxGeometry((BoxGeometryDefinition) geometryDefinition));
      else if (geometryDefinition instanceof ConeGeometryDefinition)
         protoMeshBuilder.setConeGeometry(toProtoConeGeometry((ConeGeometryDefinition) geometryDefinition));
      else if (geometryDefinition instanceof CylinderGeometryDefinition)
         protoMeshBuilder.setCylinderGeometry(toProtoCylinderGeometry((CylinderGeometryDefinition) geometryDefinition));
      else if (geometryDefinition instanceof SphereGeometryDefinition)
         protoMeshBuilder.setSphereGeometry(toProtoSphereGeometry((SphereGeometryDefinition) geometryDefinition));
      else if (geometryDefinition instanceof TorusGeometryDefinition)
         protoMeshBuilder.setTorusGeometry(toProtoTorusGeometry((TorusGeometryDefinition) geometryDefinition));
      else if (geometryDefinition instanceof ModelFileGeometryDefinition)
         protoMeshBuilder.setModelFileGeometry(toProtoModelFileGeometry((ModelFileGeometryDefinition) geometryDefinition));
      else
         System.err.println("Geometry not supported: " + geometryDefinition.getClass().getSimpleName());

      return protoMeshBuilder.build();
   }

   public static ThreeProto.MeshGroup toProtoMeshGroup(String groupId, List<VisualDefinition> visualDefinitions)
   {
      if (visualDefinitions == null)
         return null;
      ThreeProto.MeshGroup.Builder builder = ThreeProto.MeshGroup.newBuilder();
      builder.setGroupId(groupId);
      visualDefinitions.forEach(visualDefinition -> builder.addMeshes(toProtoMesh(visualDefinition)));
      return builder.build();
   }
}
