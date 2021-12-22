package us.ihmc.scs2.definition.state;

import java.util.EnumSet;
import java.util.Set;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public class JointState extends JointStateBase implements JointStateBasics
{
   private final Set<JointStateType> availableStates = EnumSet.noneOf(JointStateType.class);
   private final DMatrixRMaj configuration;
   private final DMatrixRMaj velocity;
   private final DMatrixRMaj acceleration;
   private final DMatrixRMaj effort;

   private final int configurationSize;
   private final int degreesOfFreedom;

   public JointState(int configurationSize, int degreesOfFreedom)
   {
      this.configurationSize = configurationSize;
      this.degreesOfFreedom = degreesOfFreedom;
      configuration = new DMatrixRMaj(configurationSize, 1);
      velocity = new DMatrixRMaj(degreesOfFreedom, 1);
      acceleration = new DMatrixRMaj(degreesOfFreedom, 1);
      effort = new DMatrixRMaj(degreesOfFreedom, 1);
   }

   public JointState(JointState other)
   {
      configurationSize = other.configurationSize;
      degreesOfFreedom = other.degreesOfFreedom;
      configuration = new DMatrixRMaj(other.configuration);
      velocity = new DMatrixRMaj(other.velocity);
      acceleration = new DMatrixRMaj(other.acceleration);
      effort = new DMatrixRMaj(other.effort);
      availableStates.addAll(other.availableStates);
   }

   public JointState(JointStateReadOnly other)
   {
      this(other.getConfigurationSize(), other.getDegreesOfFreedom());
      set(other);
   }

   @Override
   public void clear()
   {
      availableStates.clear();
   }

   @Override
   public void set(JointStateReadOnly other)
   {
      if (other.getConfigurationSize() != configurationSize || other.getDegreesOfFreedom() != degreesOfFreedom)
         throw new IllegalArgumentException("Dimension mismatch");

      clear();
      if (other.hasOutputFor(JointStateType.CONFIGURATION))
      {
         other.getConfiguration(0, configuration);
         availableStates.add(JointStateType.CONFIGURATION);
      }
      if (other.hasOutputFor(JointStateType.VELOCITY))
      {
         other.getVelocity(0, velocity);
         availableStates.add(JointStateType.VELOCITY);
      }
      if (other.hasOutputFor(JointStateType.ACCELERATION))
      {
         other.getAcceleration(0, acceleration);
         availableStates.add(JointStateType.ACCELERATION);
      }
      if (other.hasOutputFor(JointStateType.EFFORT))
      {
         other.getEffort(0, effort);
         availableStates.add(JointStateType.EFFORT);
      }
   }

   @Override
   public int getConfigurationSize()
   {
      return configurationSize;
   }

   @Override
   public int getDegreesOfFreedom()
   {
      return degreesOfFreedom;
   }

   @Override
   public void setConfiguration(JointReadOnly joint)
   {
      checkConfigurationSize(joint);
      availableStates.add(JointStateType.CONFIGURATION);
      joint.getJointConfiguration(0, configuration);
   }

   @Override
   public int setConfiguration(int startRow, DMatrix configuration)
   {
      CommonOps_DDRM.extract(configuration, startRow, startRow + getConfigurationSize(), 0, 1, this.configuration);
      availableStates.add(JointStateType.CONFIGURATION);
      return startRow + getConfigurationSize();
   }

   @Override
   public void setVelocity(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      availableStates.add(JointStateType.VELOCITY);
      velocity.reshape(joint.getDegreesOfFreedom(), 1);
      joint.getJointVelocity(0, velocity);
   }

   @Override
   public int setVelocity(int startRow, DMatrix velocity)
   {
      CommonOps_DDRM.extract(velocity, startRow, startRow + getDegreesOfFreedom(), 0, 1, this.velocity);
      availableStates.add(JointStateType.VELOCITY);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setAcceleration(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      availableStates.add(JointStateType.ACCELERATION);
      acceleration.reshape(joint.getDegreesOfFreedom(), 1);
      joint.getJointAcceleration(0, acceleration);
   }

   @Override
   public int setAcceleration(int startRow, DMatrix acceleration)
   {
      CommonOps_DDRM.extract(acceleration, startRow, startRow + getDegreesOfFreedom(), 0, 1, this.acceleration);
      availableStates.add(JointStateType.ACCELERATION);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setEffort(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      availableStates.add(JointStateType.EFFORT);
      effort.reshape(joint.getDegreesOfFreedom(), 1);
      joint.getJointTau(0, effort);
   }

   @Override
   public int setEffort(int startRow, DMatrix effort)
   {
      CommonOps_DDRM.extract(effort, startRow, startRow + getDegreesOfFreedom(), 0, 1, this.effort);
      availableStates.add(JointStateType.EFFORT);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public boolean hasOutputFor(JointStateType query)
   {
      return availableStates.contains(query);
   }

   @Override
   public int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      CommonOps_DDRM.insert(configuration, configurationToPack, startRow, 0);
      return startRow + configuration.getNumRows();
   }

   @Override
   public int getVelocity(int startRow, DMatrix velocityToPack)
   {
      CommonOps_DDRM.insert(velocity, velocityToPack, startRow, 0);
      return startRow + velocity.getNumRows();
   }

   @Override
   public int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      CommonOps_DDRM.insert(acceleration, accelerationToPack, startRow, 0);
      return startRow + acceleration.getNumRows();
   }

   @Override
   public int getEffort(int startRow, DMatrix effortToPack)
   {
      CommonOps_DDRM.insert(effort, effortToPack, startRow, 0);
      return startRow + effort.getNumRows();
   }

   @Override
   public void getConfiguration(JointBasics jointToUpdate)
   {
      jointToUpdate.setJointConfiguration(0, configuration);
   }

   @Override
   public void getVelocity(JointBasics jointToUpdate)
   {
      jointToUpdate.setJointVelocity(0, velocity);
   }

   @Override
   public void getAcceleration(JointBasics jointToUpdate)
   {
      jointToUpdate.setJointAcceleration(0, acceleration);
   }

   @Override
   public void getEffort(JointBasics jointToUpdate)
   {
      jointToUpdate.setJointTau(0, effort);
   }

   public DMatrixRMaj getConfiguration()
   {
      return configuration;
   }

   public DMatrixRMaj getVelocity()
   {
      return velocity;
   }

   public DMatrixRMaj getAcceleration()
   {
      return acceleration;
   }

   public DMatrixRMaj getEffort()
   {
      return effort;
   }

   @Override
   public JointState copy()
   {
      return new JointState(this);
   }
}
