package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

public class SDFModel implements SDFItem
{
   private String name;

   private String pose;

   private List<SDFLink> links;

   private List<SDFJoint> joints;

   public String getName()
   {
      return name;
   }

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
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

   public List<SDFLink> getLinks()
   {
      return links;
   }

   @XmlElement(name = "link")
   public void setLinks(List<SDFLink> links)
   {
      this.links = links;
   }

   public List<SDFJoint> getJoints()
   {
      return joints;
   }

   @XmlElement(name = "joint")
   public void setJoints(List<SDFJoint> joints)
   {
      this.joints = joints;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, pose: %s, links: %s, joints: %s]", name, pose, links, joints);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemListsURIHolders(links, joints);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }
}