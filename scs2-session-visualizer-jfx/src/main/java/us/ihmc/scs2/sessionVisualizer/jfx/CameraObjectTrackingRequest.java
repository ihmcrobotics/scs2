package us.ihmc.scs2.sessionVisualizer.jfx;

import javafx.scene.Node;

public class CameraObjectTrackingRequest
{
   private String robotName;
   private String rigidBodyName;
   private Node node;

   public CameraObjectTrackingRequest()
   {
   }

   public CameraObjectTrackingRequest(String robotName, String rigidBodyName)
   {
      this.robotName = robotName;
      this.rigidBodyName = rigidBodyName;
   }

   public CameraObjectTrackingRequest(Node node)
   {
      this.node = node;
   }

   public String getRobotName()
   {
      return robotName;
   }

   public String getRigidBodyName()
   {
      return rigidBodyName;
   }

   public Node getNode()
   {
      return node;
   }

   public void setRobotName(String robotName)
   {
      this.robotName = robotName;
   }

   public void setRigidBodyName(String rigidBodyName)
   {
      this.rigidBodyName = rigidBodyName;
   }

   public void setNode(Node node)
   {
      this.node = node;
   }

   @Override
   public String toString()
   {
      return "[robotName=" + robotName + ", rigidBodyName=" + rigidBodyName + ", node=" + node + "]";
   }
}
