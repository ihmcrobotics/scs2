package us.ihmc.scs2.definition.state;

import java.util.Arrays;
import java.util.Objects;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public class JointState extends JointStateBase implements JointStateBasics
{
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
      configuration = createNonReshapableMatrix(configurationSize);
      velocity = createNonReshapableMatrix(degreesOfFreedom);
      acceleration = createNonReshapableMatrix(degreesOfFreedom);
      effort = createNonReshapableMatrix(degreesOfFreedom);
      clear();
   }

   public JointState(JointState other)
   {
      this(other.configurationSize, other.degreesOfFreedom);
      set(other);
   }

   private DMatrixRMaj createNonReshapableMatrix(int numRows)
   {
      return new DMatrixRMaj(numRows, 1)
      {
         private static final long serialVersionUID = 1921050577949310146L;

         @Override
         public void reshape(int numRows, int numCols, boolean saveValues)
         {
            if (numRows != getNumRows())
               throw new IllegalStateException("Cannot reshape joint state matrices.");
         }
      };
   }

   public JointState(JointStateReadOnly other)
   {
      this(other.getConfigurationSize(), other.getDegreesOfFreedom());
      set(other);
   }

   @Override
   public void clear()
   {
      CommonOps_DDRM.fill(configuration, Double.NaN);
      CommonOps_DDRM.fill(velocity, Double.NaN);
      CommonOps_DDRM.fill(acceleration, Double.NaN);
      CommonOps_DDRM.fill(effort, Double.NaN);
   }

   @Override
   public void set(JointStateReadOnly other)
   {
      if (other.getConfigurationSize() != configurationSize || other.getDegreesOfFreedom() != degreesOfFreedom)
         throw new IllegalArgumentException("Dimension mismatch");

      if (other.hasOutputFor(JointStateType.CONFIGURATION))
         other.getConfiguration(0, configuration);
      else
         CommonOps_DDRM.fill(configuration, Double.NaN);

      if (other.hasOutputFor(JointStateType.VELOCITY))
         other.getVelocity(0, velocity);
      else
         CommonOps_DDRM.fill(velocity, Double.NaN);

      if (other.hasOutputFor(JointStateType.ACCELERATION))
         other.getAcceleration(0, acceleration);
      else
         CommonOps_DDRM.fill(acceleration, Double.NaN);

      if (other.hasOutputFor(JointStateType.EFFORT))
         other.getEffort(0, effort);
      else
         CommonOps_DDRM.fill(effort, Double.NaN);
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
      joint.getJointConfiguration(0, configuration);
   }

   @Override
   public int setConfiguration(int startRow, DMatrix configuration)
   {
      CommonOps_DDRM.extract(configuration, startRow, startRow + getConfigurationSize(), 0, 1, this.configuration);
      return startRow + getConfigurationSize();
   }

   @Override
   public void setVelocity(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      velocity.reshape(joint.getDegreesOfFreedom(), 1);
      joint.getJointVelocity(0, velocity);
   }

   @Override
   public int setVelocity(int startRow, DMatrix velocity)
   {
      CommonOps_DDRM.extract(velocity, startRow, startRow + getDegreesOfFreedom(), 0, 1, this.velocity);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setAcceleration(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      acceleration.reshape(joint.getDegreesOfFreedom(), 1);
      joint.getJointAcceleration(0, acceleration);
   }

   @Override
   public int setAcceleration(int startRow, DMatrix acceleration)
   {
      CommonOps_DDRM.extract(acceleration, startRow, startRow + getDegreesOfFreedom(), 0, 1, this.acceleration);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public void setEffort(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      effort.reshape(joint.getDegreesOfFreedom(), 1);
      joint.getJointTau(0, effort);
   }

   @Override
   public int setEffort(int startRow, DMatrix effort)
   {
      CommonOps_DDRM.extract(effort, startRow, startRow + getDegreesOfFreedom(), 0, 1, this.effort);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   public boolean hasOutputFor(JointStateType query)
   {
      switch (query)
      {
         case CONFIGURATION:
            return !MatrixFeatures_DDRM.hasNaN(configuration);
         case VELOCITY:
            return !MatrixFeatures_DDRM.hasNaN(velocity);
         case ACCELERATION:
            return !MatrixFeatures_DDRM.hasNaN(acceleration);
         case EFFORT:
            return !MatrixFeatures_DDRM.hasNaN(effort);
         default:
            throw new IllegalStateException("Should not get here.");
      }
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

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, configuration.getData());
      bits = EuclidHashCodeTools.addToHashCode(bits, velocity.getData());
      bits = EuclidHashCodeTools.addToHashCode(bits, acceleration.getData());
      bits = EuclidHashCodeTools.addToHashCode(bits, effort.getData());
      bits = EuclidHashCodeTools.addToHashCode(bits, configurationSize);
      bits = EuclidHashCodeTools.addToHashCode(bits, degreesOfFreedom);

      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      JointState other = (JointState) object;

      if (!Objects.equals(configuration, other.configuration))
         return false;
      if (!Objects.equals(velocity, other.velocity))
         return false;
      if (!Objects.equals(acceleration, other.acceleration))
         return false;
      if (!Objects.equals(effort, other.effort))
         return false;
      if (configurationSize != other.configurationSize)
         return false;
      if (degreesOfFreedom != other.degreesOfFreedom)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      String ret = degreesOfFreedom + "-DoF joint state";
      if (hasOutputFor(JointStateType.CONFIGURATION))
         ret += ", configuration: " + Arrays.toString(configuration.getData());
      if (hasOutputFor(JointStateType.VELOCITY))
         ret += ", velocity: " + Arrays.toString(velocity.getData());
      if (hasOutputFor(JointStateType.ACCELERATION))
         ret += ", acceleration: " + Arrays.toString(acceleration.getData());
      if (hasOutputFor(JointStateType.EFFORT))
         ret += ", effort: " + Arrays.toString(effort.getData());
      return ret;
   }
}
