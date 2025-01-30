package us.ihmc.scs2.definition.robot;

import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlRootElement;

import us.ihmc.scs2.definition.state.JointState;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.PlanarJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.SphericalJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;

@XmlRootElement(name = "RobotState")
public class RobotStateDefinition
{
   private String robotName;
   private List<JointStateEntry> jointStateEntries;

   public RobotStateDefinition()
   {
   }

   @XmlAttribute
   public void setRobotName(String robotName)
   {
      this.robotName = robotName;
   }

   @XmlElement(name = "jointStateEntry")
   public void setJointStateEntries(List<JointStateEntry> jointStateEntries)
   {
      this.jointStateEntries = jointStateEntries;
   }

   public String getRobotName()
   {
      return robotName;
   }

   public List<JointStateEntry> getJointStateEntries()
   {
      return jointStateEntries;
   }

   public static class JointStateEntry
   {
      private String jointName;
      private JointStateBasics jointState;

      public JointStateEntry()
      {
      }

      public JointStateEntry(String jointName, JointStateBasics jointState)
      {
         this.jointName = jointName;
         this.jointState = jointState;
      }

      @XmlAttribute
      public void setJointName(String jointName)
      {
         this.jointName = jointName;
      }

      @XmlElements({@XmlElement(name = "JointState", type = JointState.class),
                    @XmlElement(name = "OneDoFJointState", type = OneDoFJointState.class),
                    @XmlElement(name = "PlanarJointState", type = PlanarJointState.class),
                    @XmlElement(name = "SixDoFJointState", type = SixDoFJointState.class),
                    @XmlElement(name = "SphericalJointState", type = SphericalJointState.class)})
      public void setJointState(JointStateBasics jointState)
      {
         this.jointState = jointState;
      }

      public String getJointName()
      {
         return jointName;
      }

      public JointStateBasics getJointState()
      {
         return jointState;
      }

      @Override
      public String toString()
      {
         return "jointName: " + jointName + ", jointState: " + jointState;
      }
   }

   @Override
   public String toString()
   {
      return "robotName: " + robotName + ", jointStateEntries: " + jointStateEntries;
   }
}
