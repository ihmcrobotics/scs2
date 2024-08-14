package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.List;

import jakarta.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFGeometry implements URDFItem
{
   private URDFBox box;
   private URDFCylinder cylinder;
   private URDFSphere sphere;
   private URDFMesh mesh;

   @XmlElement(name = "box")
   public void setBox(URDFBox box)
   {
      this.box = box;
   }

   @XmlElement(name = "cylinder")
   public void setCylinder(URDFCylinder cylinder)
   {
      this.cylinder = cylinder;
   }

   @XmlElement(name = "sphere")
   public void setSphere(URDFSphere sphere)
   {
      this.sphere = sphere;
   }

   @XmlElement(name = "mesh")
   public void setMesh(URDFMesh mesh)
   {
      this.mesh = mesh;
   }

   public URDFBox getBox()
   {
      return box;
   }

   public URDFCylinder getCylinder()
   {
      return cylinder;
   }

   public URDFSphere getSphere()
   {
      return sphere;
   }

   public URDFMesh getMesh()
   {
      return mesh;
   }

   @Override
   public String getContentAsString()
   {
      return format("[box: %s, cylinder: %s, sphere: %s, mesh: %s]", box, cylinder, sphere, mesh);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemFilenameHolders(box, cylinder, sphere, mesh);
   }
}
