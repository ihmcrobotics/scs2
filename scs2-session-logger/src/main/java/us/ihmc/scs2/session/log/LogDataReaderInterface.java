package us.ihmc.scs2.session.log;

import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public interface LogDataReaderInterface
{
   void seek(int position);

   boolean read();

   double getCurrentRobotTime();

   int getCurrentLogPosition();

   LogPropertiesReader getLogProperties();

   double getDt();

   YoRegistry getYoRegistry();

   int getNumberOfEntries();

   YoLong getTimestamp();
}
