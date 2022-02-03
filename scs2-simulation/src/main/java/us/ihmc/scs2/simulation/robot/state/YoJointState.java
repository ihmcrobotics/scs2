package us.ihmc.scs2.simulation.robot.state;

import java.util.function.Function;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.state.JointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoJointState implements JointStateBasics
{
   private final YoDouble[] configuration;
   private final YoDouble[] velocity;
   private final YoDouble[] acceleration;
   private final YoDouble[] effort;

   private final int configurationSize;
   private final int degreesOfFreedom;

   private final DMatrixRMaj temp;

   public YoJointState(String namePrefix, String nameSuffix, int configurationSize, int defreesOfFreedom, YoRegistry registry)
   {
      if (namePrefix == null)
         namePrefix = "";
      else if (!namePrefix.isEmpty() && !namePrefix.endsWith("_"))
         namePrefix += "_";

      if (nameSuffix == null)
         nameSuffix = "";
      else if (!nameSuffix.isEmpty() && !nameSuffix.startsWith("_"))
         nameSuffix = "_" + nameSuffix;

      this.configurationSize = configurationSize;
      this.degreesOfFreedom = defreesOfFreedom;

      configuration = new YoDouble[configurationSize];
      velocity = new YoDouble[defreesOfFreedom];
      acceleration = new YoDouble[defreesOfFreedom];
      effort = new YoDouble[defreesOfFreedom];

      for (int i = 0; i < configurationSize; i++)
      {
         configuration[i] = new YoDouble(namePrefix + "q_" + Integer.toString(i) + nameSuffix, registry);
      }

      for (int i = 0; i < defreesOfFreedom; i++)
      {
         velocity[i] = new YoDouble(namePrefix + "qd_" + Integer.toString(i) + nameSuffix, registry);
         acceleration[i] = new YoDouble(namePrefix + "qdd_" + Integer.toString(i) + nameSuffix, registry);
         effort[i] = new YoDouble(namePrefix + "tau_" + Integer.toString(i) + nameSuffix, registry);
      }

      temp = new DMatrixRMaj(configurationSize, 1);
   }

   @Override
   public void clear()
   {
      clearConfiguration();
      clearVelocity();
      clearAcceleration();
      clearEffort();
   }

   private void clearEffort()
   {
      for (YoDouble variable : effort)
         variable.setToNaN();
   }

   private void clearAcceleration()
   {
      for (YoDouble variable : acceleration)
         variable.setToNaN();
   }

   private void clearVelocity()
   {
      for (YoDouble variable : velocity)
         variable.setToNaN();
   }

   private void clearConfiguration()
   {
      for (YoDouble variable : configuration)
         variable.setToNaN();
   }

   @Override
   public void set(JointStateReadOnly other)
   {
      if (other.getConfigurationSize() != configurationSize || other.getDegreesOfFreedom() != degreesOfFreedom)
         throw new IllegalArgumentException("Dimension mismatch");

      if (other.hasOutputFor(JointStateType.CONFIGURATION))
      {
         temp.reshape(configurationSize, 1);
         other.getConfiguration(0, temp);
         setConfiguration(0, temp);
      }
      else
      {
         clearConfiguration();
      }

      if (other.hasOutputFor(JointStateType.VELOCITY))
      {
         temp.reshape(degreesOfFreedom, 1);
         other.getVelocity(0, temp);
         setVelocity(0, temp);
      }
      else
      {
         clearVelocity();
      }

      if (other.hasOutputFor(JointStateType.ACCELERATION))
      {
         temp.reshape(degreesOfFreedom, 1);
         other.getAcceleration(0, temp);
         setAcceleration(0, temp);
      }
      else
      {
         clearAcceleration();
      }

      if (other.hasOutputFor(JointStateType.EFFORT))
      {
         temp.reshape(degreesOfFreedom, 1);
         other.getEffort(0, temp);
         setEffort(0, temp);
      }
      else
      {
         clearEffort();
      }
   }

   @Override
   public int getConfigurationSize()
   {
      return configurationSize;
   }

   public int getDegreesOfFreedom()
   {
      return degreesOfFreedom;
   }

   @Override
   public void setConfiguration(JointReadOnly joint)
   {
      checkConfigurationSize(joint);
      temp.reshape(configurationSize, 1);
      joint.getJointConfiguration(0, temp);
      setConfiguration(0, temp);
   }

   @Override
   public int setConfiguration(int startRow, DMatrix configuration)
   {
      checkConfigurationSize(startRow, configuration);
      copyDMatrixIntoYoVariables(startRow, configuration, this.configuration);
      return startRow + configurationSize;
   }

   @Override
   public void setVelocity(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      temp.reshape(degreesOfFreedom, 1);
      joint.getJointVelocity(0, temp);
      setVelocity(0, temp);
   }

   @Override
   public int setVelocity(int startRow, DMatrix velocity)
   {
      checkDegreesOfFreedom(startRow, velocity);
      copyDMatrixIntoYoVariables(startRow, velocity, this.velocity);
      return startRow + degreesOfFreedom;
   }

   @Override
   public void setAcceleration(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      temp.reshape(degreesOfFreedom, 1);
      joint.getJointAcceleration(0, temp);
      setAcceleration(0, temp);
   }

   @Override
   public int setAcceleration(int startRow, DMatrix acceleration)
   {
      checkDegreesOfFreedom(startRow, acceleration);
      copyDMatrixIntoYoVariables(startRow, acceleration, this.acceleration);
      return startRow + degreesOfFreedom;
   }

   @Override
   public void setEffort(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint);
      temp.reshape(degreesOfFreedom, 1);
      joint.getJointTau(0, temp);
      setEffort(0, temp);
   }

   @Override
   public int setEffort(int startRow, DMatrix effort)
   {
      checkDegreesOfFreedom(startRow, effort);
      copyDMatrixIntoYoVariables(startRow, effort, this.effort);
      return startRow + degreesOfFreedom;
   }

   @Override
   public boolean hasOutputFor(JointStateType query)
   {
      switch (query)
      {
         case CONFIGURATION:
            return !containsNaN(configuration);
         case VELOCITY:
            return !containsNaN(velocity);
         case ACCELERATION:
            return !containsNaN(acceleration);
         case EFFORT:
            return !containsNaN(effort);
         default:
            throw new IllegalStateException("Should not get here.");
      }
   }

   @Override
   public void getConfiguration(JointBasics jointToUpdate)
   {
      temp.reshape(configurationSize, 1);
      getConfiguration(0, temp);
      jointToUpdate.setJointConfiguration(0, temp);
   }

   @Override
   public int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      copyYoVariablesIntoDMatrix(configuration, startRow, configurationToPack);
      return startRow + configurationSize;
   }

   @Override
   public void getVelocity(JointBasics jointToUpdate)
   {
      temp.reshape(degreesOfFreedom, 1);
      getVelocity(0, temp);
      jointToUpdate.setJointVelocity(0, temp);
   }

   @Override
   public int getVelocity(int startRow, DMatrix velocityToPack)
   {
      copyYoVariablesIntoDMatrix(velocity, startRow, velocityToPack);
      return startRow + degreesOfFreedom;
   }

   @Override
   public void getAcceleration(JointBasics jointToUpdate)
   {
      temp.reshape(degreesOfFreedom, 1);
      getAcceleration(0, temp);
      jointToUpdate.setJointAcceleration(0, temp);
   }

   @Override
   public int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      copyYoVariablesIntoDMatrix(acceleration, startRow, accelerationToPack);
      return startRow + degreesOfFreedom;
   }

   @Override
   public void getEffort(JointBasics jointToUpdate)
   {
      temp.reshape(degreesOfFreedom, 1);
      getEffort(0, temp);
      jointToUpdate.setJointTau(0, temp);
   }

   @Override
   public int getEffort(int startRow, DMatrix effortToPack)
   {
      copyYoVariablesIntoDMatrix(effort, startRow, effortToPack);
      return startRow + degreesOfFreedom;
   }

   @Override
   public JointState copy()
   {
      return new JointState(this);
   }

   @Override
   public String toString()
   {
      String ret = degreesOfFreedom + "-DoF joint state";

      Function<YoDouble, String> elementToStringFunction = v -> String.format(EuclidCoreIOTools.DEFAULT_FORMAT, v.getValue());

      if (hasOutputFor(JointStateType.CONFIGURATION))
         ret += EuclidCoreIOTools.getArrayString(", configuration: [", "]", ", ", configuration, elementToStringFunction);
      if (hasOutputFor(JointStateType.VELOCITY))
         ret += EuclidCoreIOTools.getArrayString(", velocity: [", "]", ", ", velocity, elementToStringFunction);
      if (hasOutputFor(JointStateType.ACCELERATION))
         ret += EuclidCoreIOTools.getArrayString(", acceleration: [", "]", ", ", acceleration, elementToStringFunction);
      if (hasOutputFor(JointStateType.EFFORT))
         ret += EuclidCoreIOTools.getArrayString(", effort: [", "]", ", ", effort, elementToStringFunction);
      return ret;
   }

   private static boolean containsNaN(YoDouble[] variables)
   {
      for (int i = 0; i < variables.length; i++)
      {
         if (variables[i].isNaN())
            return true;
      }
      return false;
   }

   private static void copyDMatrixIntoYoVariables(int startRow, DMatrix source, YoDouble[] destination)
   {
      for (int i = 0; i < destination.length; i++)
      {
         destination[i].set(source.get(startRow++, 0));
      }
   }

   private static void copyYoVariablesIntoDMatrix(YoDouble[] source, int startRow, DMatrix destination)
   {
      for (int i = 0; i < source.length; i++)
      {
         destination.set(startRow++, 0, source[i].getValue());
      }
   }
}
