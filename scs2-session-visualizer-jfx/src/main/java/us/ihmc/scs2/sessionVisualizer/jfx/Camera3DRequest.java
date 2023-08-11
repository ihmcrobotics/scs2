package us.ihmc.scs2.sessionVisualizer.jfx;

import javafx.scene.Node;
import us.ihmc.scs2.definition.camera.YoLevelOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.camera.YoOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraControlMode;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler.TrackingTargetType;

public class Camera3DRequest
{
   private FocalPointRequest focalPointRequest;
   private CameraControlRequest cameraControlRequest;

   public Camera3DRequest()
   {
   }

   public Camera3DRequest(FocalPointRequest focalPointRequest, CameraControlRequest cameraControlRequest)
   {
      this.focalPointRequest = focalPointRequest;
      this.cameraControlRequest = cameraControlRequest;
   }

   public Camera3DRequest(CameraControlRequest cameraControlRequest)
   {
      this.cameraControlRequest = cameraControlRequest;
   }

   public Camera3DRequest(FocalPointRequest focalPointRequest)
   {
      this.focalPointRequest = focalPointRequest;
   }

   public FocalPointRequest getFocalPointRequest()
   {
      return focalPointRequest;
   }

   public CameraControlRequest getCameraControlRequest()
   {
      return cameraControlRequest;
   }

   public void setFocalPointRequest(FocalPointRequest focalPointRequest)
   {
      this.focalPointRequest = focalPointRequest;
   }

   public void setCameraControlRequest(CameraControlRequest cameraControlRequest)
   {
      this.cameraControlRequest = cameraControlRequest;
   }

   @Override
   public String toString()
   {
      return "[focalPointRequest=" + focalPointRequest + ",\ncameraControlRequest=" + cameraControlRequest + "]";
   }

   public static class FocalPointRequest
   {
      private TrackingTargetType trackingTargetType;

      private String robotNameToTrack;
      private String rigidBodyNameToTrack;
      private Node nodeToTrack;

      private YoTuple3DDefinition coordinatesToTrack;

      public static FocalPointRequest trackRobot(String robotName, String rigidBodyName)
      {
         FocalPointRequest request = new FocalPointRequest();
         request.robotNameToTrack = robotName;
         request.rigidBodyNameToTrack = rigidBodyName;
         return request;
      }

      public static FocalPointRequest trackNode(Node node)
      {
         FocalPointRequest request = new FocalPointRequest();
         request.trackingTargetType = TrackingTargetType.Node;
         request.nodeToTrack = node;
         return request;
      }

      public static FocalPointRequest trackCoordinates(YoTuple3DDefinition tuple3DDefinition)
      {
         FocalPointRequest request = new FocalPointRequest();
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

   public static class CameraControlRequest
   {
      private CameraControlMode controlMode;
      private YoTuple3DDefinition positionToTrack;
      private YoOrbitalCoordinateDefinition orbitToTrack;
      private YoLevelOrbitalCoordinateDefinition levelOrbitToTrack;

      public static CameraControlRequest trackPosition(YoTuple3DDefinition positionToTrack)
      {
         CameraControlRequest request = new CameraControlRequest();
         request.controlMode = CameraControlMode.Position;
         request.positionToTrack = positionToTrack;
         return request;
      }

      public static CameraControlRequest trackOrbit(YoOrbitalCoordinateDefinition orbitToTrack)
      {
         CameraControlRequest request = new CameraControlRequest();
         request.controlMode = CameraControlMode.Orbital;
         request.orbitToTrack = orbitToTrack;
         return request;
      }

      public static CameraControlRequest trackLevelOrbit(YoLevelOrbitalCoordinateDefinition levelOrbitToTrack)
      {
         CameraControlRequest request = new CameraControlRequest();
         request.controlMode = CameraControlMode.LevelOrbital;
         request.levelOrbitToTrack = levelOrbitToTrack;
         return request;
      }

      public CameraControlMode getControlMode()
      {
         return controlMode;
      }

      public YoTuple3DDefinition getPositionToTrack()
      {
         return positionToTrack;
      }

      public YoOrbitalCoordinateDefinition getOrbitToTrack()
      {
         return orbitToTrack;
      }

      public YoLevelOrbitalCoordinateDefinition getLevelOrbitToTrack()
      {
         return levelOrbitToTrack;
      }

      public void setControlMode(CameraControlMode controlMode)
      {
         this.controlMode = controlMode;
      }

      public void setPositionToTrack(YoTuple3DDefinition positionToTrack)
      {
         this.positionToTrack = positionToTrack;
      }

      public void setOrbitToTrack(YoOrbitalCoordinateDefinition orbitToTrack)
      {
         this.orbitToTrack = orbitToTrack;
      }

      public void setLevelOrbitToTrack(YoLevelOrbitalCoordinateDefinition levelOrbitToTrack)
      {
         this.levelOrbitToTrack = levelOrbitToTrack;
      }

      @Override
      public String toString()
      {
         return "[controlMode=" + controlMode + ", positionToTrack=" + positionToTrack + ", orbitToTrack=" + orbitToTrack + ", levelOrbitToTrack="
               + levelOrbitToTrack + "]";
      }
   }
}
