package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
@XmlType(propOrder = {"name",
                      "type",
                      "origin",
                      "axis",
                      "parent",
                      "child",
                      "calibration",
                      "dynamics",
                      "limit",
                      "mimic",
                      "safetyController",
                      "actuatedJointIndex",
                      "subJoints",
                      "subLinks"})
public class URDFJoint implements URDFItem
{
   public enum URDFJointType
   {
      continuous, revolute, prismatic, fixed, floating, planar, cross_four_bar, revolute_twins;

      public static URDFJointType parse(String value)
      {
         if (value == null)
            return null;
         for (URDFJointType type : values())
         {
            if (type.name().equals(value))
               return type;
         }
         return null;
      }
   }

   private String name;
   private String type;
   private URDFOrigin origin;
   private URDFLinkReference parent;
   private URDFLinkReference child;
   private URDFAxis axis;
   private URDFCalibration calibration;
   private URDFDynamics dynamics;
   private URDFLimit limit;
   private URDFMimic mimic;
   private URDFSafetyController safetyController;

   /**
    * The sub-joints are used to create complex joints with internal kinematics like a
    * {@link us.ihmc.mecano.multiBodySystem.CrossFourBarJoint} or a
    * {@link us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint}
    */
   private List<URDFJoint> subJoints;
   /**
    * The sub-links are used to create complex joints with internal kinematics like a
    * {@link us.ihmc.mecano.multiBodySystem.CrossFourBarJoint} or a
    * {@link us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint}
    */
   private List<URDFLink> subLinks;

   /**
    * Indicates which sub-joint is the torque source for complex joints with internal kinematics like a
    * {@link us.ihmc.mecano.multiBodySystem.CrossFourBarJoint} or a
    * {@link us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint}.
    */
   private String actuatedJointIndex;

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlAttribute(name = "type")
   public void setType(String type)
   {
      this.type = type;
   }

   public void setType(URDFJointType type)
   {
      setType(type.name());
   }

   @XmlElement(name = "origin")
   public void setOrigin(URDFOrigin origin)
   {
      this.origin = origin;
   }

   @XmlElement(name = "parent")
   public void setParent(URDFLinkReference parent)
   {
      this.parent = parent;
   }

   @XmlElement(name = "child")
   public void setChild(URDFLinkReference child)
   {
      this.child = child;
   }

   @XmlElement(name = "axis")
   public void setAxis(URDFAxis axis)
   {
      this.axis = axis;
   }

   @XmlElement(name = "calibration")
   public void setCalibration(URDFCalibration calibration)
   {
      this.calibration = calibration;
   }

   @XmlElement(name = "dynamics")
   public void setDynamics(URDFDynamics dynamics)
   {
      this.dynamics = dynamics;
   }

   @XmlElement(name = "limit")
   public void setLimit(URDFLimit limit)
   {
      this.limit = limit;
   }

   @XmlElement(name = "mimic")
   public void setMimic(URDFMimic mimic)
   {
      this.mimic = mimic;
   }

   @XmlElement(name = "safety_controller")
   public void setSafetyController(URDFSafetyController safetyController)
   {
      this.safetyController = safetyController;
   }

   @XmlElement(name = "sub_joint")
   public void setSubJoints(List<URDFJoint> subJoints)
   {
      this.subJoints = subJoints;
   }

   @XmlElement(name = "sub_link")
   public void setSubLinks(List<URDFLink> subLinks)
   {
      this.subLinks = subLinks;
   }

   @XmlElement(name = "actuated_joint_index")
   public void setActuatedJointIndex(String actuatedJointIndex)
   {
      this.actuatedJointIndex = actuatedJointIndex;
   }

   public String getName()
   {
      return name;
   }

   public String getType()
   {
      return type;
   }

   public URDFOrigin getOrigin()
   {
      return origin;
   }

   public URDFLinkReference getParent()
   {
      return parent;
   }

   public URDFLinkReference getChild()
   {
      return child;
   }

   public URDFAxis getAxis()
   {
      return axis;
   }

   public URDFCalibration getCalibration()
   {
      return calibration;
   }

   public URDFDynamics getDynamics()
   {
      return dynamics;
   }

   public URDFLimit getLimit()
   {
      return limit;
   }

   public URDFMimic getMimic()
   {
      return mimic;
   }

   public URDFSafetyController getSafetyController()
   {
      return safetyController;
   }

   public List<URDFJoint> getSubJoints()
   {
      return subJoints;
   }

   public List<URDFLink> getSubLinks()
   {
      return subLinks;
   }

   public String getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, type: %s, origin: %s, parent: %s, child: %s, axis: %s, calibration: %s, dynamics: %s, limit: %s, mimic: %s, safetyController: %s]",
                    name,
                    type,
                    origin,
                    parent,
                    child,
                    axis,
                    calibration,
                    dynamics,
                    limit,
                    mimic,
                    safetyController,
                    subJoints,
                    subLinks);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      List<URDFFilenameHolder> filenameHolders = URDFItem.combineItemFilenameHolders(origin,
                                                                                     parent,
                                                                                     child,
                                                                                     axis,
                                                                                     calibration,
                                                                                     dynamics,
                                                                                     limit,
                                                                                     mimic,
                                                                                     safetyController);
      filenameHolders.addAll(URDFItem.combineItemListsFilenameHolders(subJoints, subLinks));
      return filenameHolders;
   }
}
