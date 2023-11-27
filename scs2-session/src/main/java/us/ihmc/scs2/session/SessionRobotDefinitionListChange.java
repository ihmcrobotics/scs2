package us.ihmc.scs2.session;

import us.ihmc.scs2.definition.robot.RobotDefinition;

import java.io.File;

public class SessionRobotDefinitionListChange
{
   private final SessionRobotDefinitionListChangeType changeType;
   private final File newRobotModelFile;
   private final RobotDefinition addedRobotDefinition;
   private final RobotDefinition removedRobotDefinition;

   public static SessionRobotDefinitionListChange add(RobotDefinition addedRobotDefinition)
   {
      return new SessionRobotDefinitionListChange(SessionRobotDefinitionListChangeType.ADD, null, addedRobotDefinition, null);
   }

   public static SessionRobotDefinitionListChange add(File newRobotModelFile)
   {
      return new SessionRobotDefinitionListChange(SessionRobotDefinitionListChangeType.ADD, newRobotModelFile, null, null);
   }

   public static SessionRobotDefinitionListChange remove(RobotDefinition removedRobotDefinition)
   {
      return new SessionRobotDefinitionListChange(SessionRobotDefinitionListChangeType.REMOVE, null, null, removedRobotDefinition);
   }

   public static SessionRobotDefinitionListChange replace(RobotDefinition addedRobotDefinition, RobotDefinition removedRobotDefinition)
   {
      return new SessionRobotDefinitionListChange(SessionRobotDefinitionListChangeType.REPLACE, null, addedRobotDefinition, removedRobotDefinition);
   }

   public static SessionRobotDefinitionListChange replace(File newRobotModelFile, RobotDefinition removedRobotDefinition)
   {
      return new SessionRobotDefinitionListChange(SessionRobotDefinitionListChangeType.REPLACE, newRobotModelFile, null, removedRobotDefinition);
   }

   private SessionRobotDefinitionListChange(SessionRobotDefinitionListChangeType changeType,
                                            File newRobotModelFile,
                                            RobotDefinition addedRobotDefinition,
                                            RobotDefinition removedRobotDefinition)
   {
      this.changeType = changeType;
      this.addedRobotDefinition = addedRobotDefinition;
      this.removedRobotDefinition = removedRobotDefinition;
      this.newRobotModelFile = newRobotModelFile;
   }

   public SessionRobotDefinitionListChangeType getChangeType()
   {
      return changeType;
   }

   /**
    * Returns {@code null} when the change is a removal, otherwise returns the robot model file for creating the new robot definition that is to be replaced or
    * added.
    * <p>
    * Note that only the {@code Session} is allowed to create a new robot definition from a model file.
    * </p>
    *
    * @return the new robot model file or {@code null}.
    */
   public File getNewRobotModelFile()
   {
      return newRobotModelFile;
   }

   /**
    * Returns {@code null} when the change is a removal, otherwise returns the robot definition that is being replaced or added.
    *
    * @return the new robot definition or {@code null}.
    */
   public RobotDefinition getAddedRobotDefinition()
   {
      return addedRobotDefinition;
   }

   /**
    * Returns {@code null} when the change is an addition, otherwise returns the robot definition that is being replaced or removed.
    *
    * @return the old robot definition or {@code null}.
    */
   public RobotDefinition getRemovedRobotDefinition()
   {
      return removedRobotDefinition;
   }

   public enum SessionRobotDefinitionListChangeType
   {
      ADD, REMOVE, REPLACE;
   }
}
