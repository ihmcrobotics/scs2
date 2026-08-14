package us.ihmc.scs2.sessionVisualizer.jfx;

import javafx.stage.Window;
import javafx.util.Pair;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.ImmutableTriple;
import us.ihmc.scs2.definition.yoChart.YoChartConfigurationDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoEntry.YoEntryListDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoSlider.*;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.ChartTable2D.ChartTable2DSize;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.SearchEngines;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.NewTerrainVisualRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.Topic;
import us.ihmc.scs2.sessionVisualizer.jfx.session.BindSynchronizingVariablesRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenAddLogRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest;

import java.io.File;
import java.util.List;

/**
 * Declares every topic used as an internal pub/sub channel across the JavaFX UI layer (see
 * {@link us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager}). Grouped into nested classes by
 * feature area, mirroring the layout this API has always had.
 */
public class SessionVisualizerMessagerAPI
{
   public static final Topic<Boolean> DisableUserControls = new Topic<>("DisableUserControls");
   public static final Topic<SceneVideoRecordingRequest> SceneVideoRecordingRequest = new Topic<>("SceneVideoRecordingRequest");
   public static final Topic<Camera3DRequest> Camera3DRequest = new Topic<>("Camera3DRequest");
   public static final Topic<Object> TakeSnapshot = new Topic<>("TakeSnapshot");
   public static final Topic<Object> RegisterRecordable = new Topic<>("RegisterRecordable");
   public static final Topic<Object> ForgetRecordable = new Topic<>("ForgetRecordable");
   public static final Topic<Boolean> ShowAdvancedControls = new Topic<>("ShowAdvancedControls");
   public static final Topic<Boolean> ShowOverheadPlotter = new Topic<>("ShowOverheadPlotter");
   public static final Topic<NewRobotVisualRequest> RobotVisualRequest = new Topic<>("RobotVisualRequest");
   public static final Topic<NewTerrainVisualRequest> TerrainVisualRequest = new Topic<>("TerrainVisualRequest");
   public static final Topic<NewWindowRequest> OpenWindowRequest = new Topic<>("OpenWindowRequest");
   public static final Topic<Boolean> SessionVisualizerCloseRequest = new Topic<>("SessionVisualizerCloseRequest");
   public static final Topic<Integer> ControlsNumberPrecision = new Topic<>("ControlsNumberPrecision"); // TODO Not the greatest topic name, nor the best place.
   public static final Topic<File> SessionVisualizerConfigurationLoadRequest = new Topic<>("SessionVisualizerConfigurationLoadRequest");
   public static final Topic<Boolean> SessionVisualizerDefaultConfigurationLoadRequest = new Topic<>("SessionVisualizerDefaultConfigurationLoadRequest");
   public static final Topic<File> SessionVisualizerConfigurationSaveRequest = new Topic<>("SessionVisualizerConfigurationSaveRequest");
   public static final Topic<Boolean> SessionVisualizerDefaultConfigurationSaveRequest = new Topic<>("SessionVisualizerDefaultConfigurationSaveRequest");
   public static final Topic<SessionDataFilterParameters> SessionDataFilterParametersAddRequest = new Topic<>("SessionDataFilterParametersAddRequest");

   public static class KeyFrame
   {
      public static final Topic<Object> ToggleKeyFrame = new Topic<>("KeyFrame.ToggleKeyFrame");
      public static final Topic<Object> GoToNextKeyFrame = new Topic<>("KeyFrame.GoToNextKeyFrame");
      public static final Topic<Object> GoToPreviousKeyFrame = new Topic<>("KeyFrame.GoToPreviousKeyFrame");
      public static final Topic<Object> RequestCurrentKeyFrames = new Topic<>("KeyFrame.RequestCurrentKeyFrames");

      public static final Topic<int[]> CurrentKeyFrames = new Topic<>("KeyFrame.CurrentKeyFrames");
   }

   public static class YoSearch
   {
      public static final Topic<SearchEngines> YoSearchEngine = new Topic<>("YoSearch.YoSearchEngine");
      public static final Topic<Integer> YoSearchMaxListSize = new Topic<>("YoSearch.YoSearchMaxListSize");
      public static final Topic<File> YoCompositePatternLoadRequest = new Topic<>("YoSearch.YoCompositePatternLoadRequest");
      public static final Topic<File> YoCompositePatternSaveRequest = new Topic<>("YoSearch.YoCompositePatternSaveRequest");
      public static final Topic<List<String>> YoCompositePatternSelected = new Topic<>("YoSearch.YoCompositePatternSelected");
      public static final Topic<Boolean> YoCompositeRefreshAll = new Topic<>("YoSearch.YoCompositeRefreshAll");
      public static final Topic<Boolean> ShowSCS2YoVariables = new Topic<>("YoSearch.ShowSCS2YoVariables");
      public static final Topic<YoNameDisplay> YoVariableNameDisplay = new Topic<>("YoSearch.YoVariableNameDisplay");
   }

   public static class YoGraphic
   {
      public static final Topic<File> YoGraphicSaveRequest = new Topic<>("YoGraphic.YoGraphicSaveRequest");
      public static final Topic<File> YoGraphicLoadRequest = new Topic<>("YoGraphic.YoGraphicLoadRequest");

      public static final Topic<String> RemoveYoGraphicRequest = new Topic<>("YoGraphic.RemoveYoGraphicRequest");
      public static final Topic<Pair<String, Boolean>> SetYoGraphicVisibleRequest = new Topic<>("YoGraphic.SetYoGraphicVisibleRequest");
      public static final Topic<YoGraphicDefinition> AddYoGraphicRequest = new Topic<>("YoGraphic.AddYoGraphicRequest");
      public static final Topic<YoTuple2DDefinition> Plotter2DTrackCoordinateRequest = new Topic<>("YoGraphic.Plotter2DTrackCoordinateRequest");
   }

   public static class YoChart
   {
      public static final Topic<Pair<Window, Double>> YoChartZoomFactor = new Topic<>("YoChart.YoChartZoomFactor");
      public static final Topic<Pair<Window, Boolean>> YoChartRequestZoomIn = new Topic<>("YoChart.YoChartRequestZoomIn");
      public static final Topic<Pair<Window, Boolean>> YoChartRequestZoomOut = new Topic<>("YoChart.YoChartRequestZoomOut");
      public static final Topic<Pair<Window, Integer>> YoChartRequestShift = new Topic<>("YoChart.YoChartRequestShift");
      public static final Topic<Pair<Window, Boolean>> YoChartShowYAxis = new Topic<>("YoChart.YoChartShowYAxis");
      public static final Topic<Pair<Window, File>> YoChartGroupSaveConfiguration = new Topic<>("YoChart.YoChartGroupSaveConfiguration");
      public static final Topic<Pair<Window, File>> YoChartGroupLoadConfiguration = new Topic<>("YoChart.YoChartGroupLoadConfiguration");
      public static final Topic<Pair<Window, ChartTable2DSize>> YoChartGroupResize = new Topic<>("YoChart.YoChartGroupResize");

      public static final Topic<ImmutablePair<String, YoChartConfigurationDefinition>> YoChartListAdd = new Topic<>("YoChart.YoChartListAdd");
   }

   public static class YoEntry
   {
      public static final Topic<YoEntryListDefinition> YoEntryListAdd = new Topic<>("YoEntry.YoEntryListAdd");
   }

   public static class YoSliderboard
   {
      public static final Topic<File> YoMultiSliderboardSave = new Topic<>("YoSliderboard.YoMultiSliderboardSave");
      public static final Topic<File> YoMultiSliderboardLoad = new Topic<>("YoSliderboard.YoMultiSliderboardLoad");
      public static final Topic<Boolean> YoMultiSliderboardClearAll = new Topic<>("YoSliderboard.YoMultiSliderboardClearAll");
      public static final Topic<YoSliderboardListDefinition> YoMultiSliderboardSet = new Topic<>("YoSliderboard.YoMultiSliderboardSet");
      public static final Topic<YoSliderboardDefinition> YoSliderboardSet = new Topic<>("YoSliderboard.YoSliderboardSet");
      public static final Topic<Pair<String, YoSliderboardType>> YoSliderboardRemove = new Topic<>("YoSliderboard.YoSliderboardRemove");

      public static final Topic<ImmutableTriple<String, YoSliderboardType, YoButtonDefinition>> YoSliderboardSetButton = new Topic<>("YoSliderboard.YoSliderboardSetButton");
      public static final Topic<ImmutableTriple<String, YoSliderboardType, YoKnobDefinition>> YoSliderboardSetKnob = new Topic<>("YoSliderboard.YoSliderboardSetKnob");
      public static final Topic<ImmutableTriple<String, YoSliderboardType, YoSliderDefinition>> YoSliderboardSetSlider = new Topic<>("YoSliderboard.YoSliderboardSetSlider");
      public static final Topic<ImmutableTriple<String, YoSliderboardType, Integer>> YoSliderboardClearButton = new Topic<>("YoSliderboard.YoSliderboardClearButton");
      public static final Topic<ImmutableTriple<String, YoSliderboardType, Integer>> YoSliderboardClearKnob = new Topic<>("YoSliderboard.YoSliderboardClearKnob");
      public static final Topic<ImmutableTriple<String, YoSliderboardType, Integer>> YoSliderboardClearSlider = new Topic<>("YoSliderboard.YoSliderboardClearSlider");
   }

   public static class SessionAPI
   {
      public static final Topic<Session> StartNewSessionRequest = new Topic<>("SessionAPI.StartNewSessionRequest");
      public static final Topic<OpenSessionControlsRequest> OpenSessionControlsRequest = new Topic<>("SessionAPI.OpenSessionControlsRequest");
      public static final Topic<OpenAddLogRequest> OpenAddLogRequest = new Topic<>("SessionAPI.OpenAddLogRequest");
      public static final Topic<File> OpenLogDirectoryRequest = new Topic<>("SessionAPI.OpenLogDirectoryRequest");
      public static final Topic<File> OpenMCAPLogFileRequest = new Topic<>("SessionAPI.OpenMCAPLogFileRequest");
      public static final Topic<BindSynchronizingVariablesRequest> BindSynchronizingVariablesRequest = new Topic<>("SessionAPI.BindSynchronizingVariablesRequest");
   }
}
