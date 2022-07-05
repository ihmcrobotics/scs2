package us.ihmc.scs2.simulation.robot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
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

   public void updateFrames()
   {
      kinematicPoints.forEach(kp -> kp.getFrame().update());
      externalWrenchPoints.forEach(ewp -> ewp.getFrame().update());
      groundContactPoints.forEach(gcp -> gcp.getFrame().update());

      imuSensors.forEach(imu -> imu.getFrame().update());
      wrenchSensors.forEach(wrench -> wrench.getFrame().update());
      cameraSensors.forEach(camera -> camera.getFrame().update());
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
   }

   public KinematicPoint addKinematicPoint(String name)
   {
      return addKinematicPoint(new KinematicPointDefinition(name));
   }

   public KinematicPoint addKinematicPoint(String name, Tuple3DReadOnly offset)
   {
      return addKinematicPoint(new KinematicPointDefinition(name, offset));
   }

   public KinematicPoint addKinematicPoint(KinematicPointDefinition definition)
   {
      KinematicPoint kinematicPoint = new KinematicPoint(definition, joint);
      kinematicPoints.add(kinematicPoint);
      return kinematicPoint;
   }

   public ExternalWrenchPoint addExternalWrenchPoint(String name)
   {
      return addExternalWrenchPoint(new ExternalWrenchPointDefinition(name));
   }

   public ExternalWrenchPoint addExternalWrenchPoint(String name, Tuple3DReadOnly offset)
   {
      return addExternalWrenchPoint(new ExternalWrenchPointDefinition(name, offset));
   }

   public ExternalWrenchPoint addExternalWrenchPoint(ExternalWrenchPointDefinition definition)
   {
      ExternalWrenchPoint externalWrenchPoint = new ExternalWrenchPoint(definition, joint);
      externalWrenchPoints.add(externalWrenchPoint);
      return externalWrenchPoint;
   }

   public GroundContactPoint addGroundContactPoint(String name)
   {
      return addGroundContactPoint(new GroundContactPointDefinition(name));
   }

   public GroundContactPoint addGroundContactPoint(String name, Tuple3DReadOnly offset)
   {
      return addGroundContactPoint(new GroundContactPointDefinition(name, offset));
   }

   public GroundContactPoint addGroundContactPoint(GroundContactPointDefinition definition)
   {
      GroundContactPoint groundContactPoint = new GroundContactPoint(definition, joint);
      groundContactPoints.add(groundContactPoint);
      return groundContactPoint;
   }

   public SimIMUSensor addIMUSensor(String name)
   {
      return addIMUSensor(new IMUSensorDefinition(name));
   }

   public SimIMUSensor addIMUSensor(String name, Tuple3DReadOnly offset)
   {
      return addIMUSensor(new IMUSensorDefinition(name, offset));
   }

   public SimIMUSensor addIMUSensor(IMUSensorDefinition definition)
   {
      SimIMUSensor newSensor = new SimIMUSensor(definition, joint);
      imuSensors.add(newSensor);
      return newSensor;
   }

   public SimWrenchSensor addWrenchSensor(String name)
   {
      return addWrenchSensor(new WrenchSensorDefinition(name));
   }

   public SimWrenchSensor addWrenchSensor(String name, Tuple3DReadOnly offset)
   {
      return addWrenchSensor(new WrenchSensorDefinition(name, offset));
   }

   public SimWrenchSensor addWrenchSensor(WrenchSensorDefinition definition)
   {
      SimWrenchSensor newSensor = new SimWrenchSensor(definition, joint);
      wrenchSensors.add(newSensor);
      return newSensor;
   }

   public SimCameraSensor addCameraSensor(String name)
   {
      return addCameraSensor(new CameraSensorDefinition(name));
   }

   public SimCameraSensor addCameraSensor(String name, Tuple3DReadOnly offset)
   {
      return addCameraSensor(new CameraSensorDefinition(name, offset));
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
