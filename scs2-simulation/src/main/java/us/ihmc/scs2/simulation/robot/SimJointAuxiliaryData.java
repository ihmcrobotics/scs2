package us.ihmc.scs2.simulation.robot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.sensors.SimCameraSensor;
import us.ihmc.scs2.simulation.robot.sensors.SimIMUSensor;
import us.ihmc.scs2.simulation.robot.sensors.SimWrenchSensor;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.scs2.simulation.robot.trackers.KinematicPoint;

public class SimJointAuxiliaryData
{
   private final SimJointBasics joint;

   private final List<KinematicPoint> kinematicPoints = new ArrayList<>();
   private final List<ExternalWrenchPoint> externalWrenchPoints = new ArrayList<>();
   private final List<GroundContactPoint> groundContactPoints = new ArrayList<>();

   private final List<SimIMUSensor> imuSensors = new ArrayList<>();
   private final List<SimWrenchSensor> wrenchSensors = new ArrayList<>();
   private final List<SimCameraSensor> cameraSensors = new ArrayList<>();

   public SimJointAuxiliaryData(SimJointBasics joint)
   {
      this.joint = joint;
   }

   public void update(RobotPhysicsOutput physicsOutput)
   {
      for (int i = 0; i < kinematicPoints.size(); i++)
         kinematicPoints.get(i).update();
      for (int i = 0; i < externalWrenchPoints.size(); i++)
         externalWrenchPoints.get(i).update();
      for (int i = 0; i < groundContactPoints.size(); i++)
         groundContactPoints.get(i).update();
      for (int i = 0; i < imuSensors.size(); i++)
         imuSensors.get(i).update(physicsOutput);
      for (int i = 0; i < wrenchSensors.size(); i++)
         wrenchSensors.get(i).update(physicsOutput);
      for (int i = 0; i < cameraSensors.size(); i++)
         cameraSensors.get(i).update(physicsOutput);
         
      //kinematicPoints.forEach(kp -> kp.update());
      //externalWrenchPoints.forEach(ewp -> ewp.update());
      //groundContactPoints.forEach(gcp -> gcp.update());
      //imuSensors.forEach(imu -> imu.update(physicsOutput));
      //wrenchSensors.forEach(wrench -> wrench.update(physicsOutput));
      //cameraSensors.forEach(camera -> camera.update(physicsOutput));
   }

   public KinematicPoint addKinematicPoint(KinematicPointDefinition definition)
   {
      KinematicPoint kinematicPoint = new KinematicPoint(definition, joint);
      kinematicPoints.add(kinematicPoint);
      return kinematicPoint;
   }

   public ExternalWrenchPoint addExternalWrenchPoint(ExternalWrenchPointDefinition definition)
   {
      ExternalWrenchPoint externalWrenchPoint = new ExternalWrenchPoint(definition, joint);
      externalWrenchPoints.add(externalWrenchPoint);
      return externalWrenchPoint;
   }

   public GroundContactPoint addGroundContactPoint(GroundContactPointDefinition definition)
   {
      GroundContactPoint groundContactPoint = new GroundContactPoint(definition, joint);
      groundContactPoints.add(groundContactPoint);
      return groundContactPoint;
   }

   public SimIMUSensor addIMUSensor(IMUSensorDefinition definition)
   {
      SimIMUSensor newSensor = new SimIMUSensor(definition, joint);
      imuSensors.add(newSensor);
      return newSensor;
   }

   public SimWrenchSensor addWrenchSensor(WrenchSensorDefinition definition)
   {
      SimWrenchSensor newSensor = new SimWrenchSensor(definition, joint);
      wrenchSensors.add(newSensor);
      return newSensor;
   }

   public SimCameraSensor addCameraSensor(CameraSensorDefinition definition)
   {
      SimCameraSensor newSensor = new SimCameraSensor(definition, joint);
      cameraSensors.add(newSensor);
      return newSensor;
   }

   public SimJointBasics getJoint()
   {
      return joint;
   }

   public List<KinematicPoint> getKinematicPoints()
   {
      return kinematicPoints;
   }

   public List<ExternalWrenchPoint> getExternalWrenchPoints()
   {
      return externalWrenchPoints;
   }

   public List<GroundContactPoint> getGroundContactPoints()
   {
      return groundContactPoints;
   }

   public List<SimIMUSensor> getIMUSensors()
   {
      return imuSensors;
   }

   public List<SimWrenchSensor> getWrenchSensors()
   {
      return wrenchSensors;
   }

   public List<SimCameraSensor> getCameraSensors()
   {
      return cameraSensors;
   }
}
