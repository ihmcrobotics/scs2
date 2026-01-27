package us.ihmc.scs2.session.log;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.List;

public interface LogDataReaderInterface
{
   void seek(int position);

   boolean read();

   double getCurrentRobotTime();

   int getCurrentLogPosition();

   LogProperties getLogProperties();

   java.io.File getLogDirectory();

   double getDt();

   YoRegistry getLocalYoRegistry();

   YoRegistry getLogRootRegistry();

   YoGraphicsListRegistry getLogSCS1YoGraphics();

   List<YoGraphicGroupDefinition> getLogSCS2YoGraphics();

   int getNumberOfEntries();

   YoLong getTimestamp();

   void removeTimestampListeners();

   long getRelativeTimestamp(int position);

   List<YoVariable> getYoVariablesList();

   List<JointState> getJointStates();
}
