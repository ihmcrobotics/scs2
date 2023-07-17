package us.ihmc.scs2.sessionVisualizer.jfx;

import javafx.scene.Node;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler.TrackingTargetType;

public class Camera3DRequest
{
   private TrackingTargetType trackingTargetType;

   private String robotNameToTrack;
   private String rigidBodyNameToTrack;
   private Node nodeToTrack;

   private YoTuple3DDefinition coordinatesToTrack;

   public Camera3DRequest()
   {
   }

   public static Camera3DRequest trackRobot(String robotName, String rigidBodyName)
   {
      Camera3DRequest request = new Camera3DRequest();
      request.robotNameToTrack = robotName;
      request.rigidBodyNameToTrack = rigidBodyName;
      return request;
   }

   public static Camera3DRequest trackNode(Node node)
   {
      Camera3DRequest request = new Camera3DRequest();
      request.trackingTargetType = TrackingTargetType.Node;
      request.nodeToTrack = node;
      return request;
   }

   public static Camera3DRequest trackCoordinates(YoTuple3DDefinition tuple3DDefinition)
   {
      Camera3DRequest request = new Camera3DRequest();
      request.trackingTargetType = TrackingTargetType.YoCoordinates;
      request.coordinatesToTrack = tuple3DDefinition;
      return request;
   }

   public TrackingTargetType getTrackingTargetType()
   {
      return trackingTargetType;
   }

   public String getRobotName()
   {
      return robotNameToTrack;
   }

   public String getRigidBodyName()
   {
      return rigidBodyNameToTrack;
   }

   public Node getNode()
   {
      return nodeToTrack;
   }

   public YoTuple3DDefinition getCoordinatesToTrack()
   {
      return coordinatesToTrack;
   }

   public void setRobotName(String robotName)
   {
      this.robotNameToTrack = robotName;
   }

   public void setRigidBodyName(String rigidBodyName)
   {
      this.rigidBodyNameToTrack = rigidBodyName;
   }

   public void setNode(Node node)
   {
      this.nodeToTrack = node;
   }

   @Override
   public String toString()
   {
      return "[trackingTargetType=" + trackingTargetType + ", robotNameToTrack=" + robotNameToTrack + ", rigidBodyNameToTrack=" + rigidBodyNameToTrack
             + ", nodeToTrack=" + nodeToTrack + ", coordinatesToTrack=" + coordinatesToTrack + "]";
   }

}
