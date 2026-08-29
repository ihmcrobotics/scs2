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
import us.ihmc.scs2.session.*;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.ChartTable2D.ChartTable2DSize;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.SearchEngines;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.NewTerrainVisualRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.Topic;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenAddLogRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.BindSynchronizingVariablesRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest;

import java.io.File;
import java.util.List;

public class SessionVisualizerTopics
{
   // GUI internal topics:
   private Topic<Boolean> disableUserControls;
   private Topic<SceneVideoRecordingRequest> sceneVideoRecordingRequest;
   private Topic<Camera3DRequest> camera3DRequest;
   private Topic<Object> takeSnapshot;
   private Topic<Object> registerRecordable;
   private Topic<Object> forgetRecordable;
   private Topic<Boolean> showAdvancedControls;
   private Topic<Boolean> showOverheadPlotter;
   private Topic<NewRobotVisualRequest> robotVisualRequest;
   private Topic<NewTerrainVisualRequest> terrainVisualRequest;
   private Topic<NewWindowRequest> openWindowRequest;
   private Topic<Boolean> sessionVisualizerCloseRequest;

   private Topic<Object> toggleKeyFrame, requestCurrentKeyFrames;
   private Topic<Object> goToNextKeyFrame, goToPreviousKeyFrame;
   private Topic<int[]> currentKeyFrames;

   private Topic<SearchEngines> yoSearchEngine;
   private Topic<Integer> yoSearchMaxListSize;
   private Topic<File> yoCompositePatternLoadRequest;
   private Topic<File> yoCompositePatternSaveRequest;
   private Topic<List<String>> yoCompositeSelected;
   private Topic<Boolean> yoCompositeRefreshAll;
   private Topic<Boolean> showSCS2YoVariables;
   private Topic<YoNameDisplay> yoVariableNameDisplay;

   private Topic<File> yoGraphicLoadRequest;
   private Topic<File> yoGraphicSaveRequest;
   private Topic<String> removeYoGraphicRequest;
   private Topic<Pair<String, Boolean>> setYoGraphicVisibleRequest;
   private Topic<YoGraphicDefinition> addYoGraphicRequest;
   private Topic<YoTuple2DDefinition> plotter2DTrackCoordinateRequest;

   private Topic<Pair<Window, Double>> yoChartZoomFactor;
   private Topic<Pair<Window, Boolean>> yoChartRequestZoomIn, yoChartRequestZoomOut;
   private Topic<Pair<Window, Integer>> yoChartRequestShift;
   private Topic<Pair<Window, Boolean>> yoChartShowYAxis;

   private Topic<Pair<Window, File>> yoChartGroupSaveConfiguration;
   private Topic<Pair<Window, File>> yoChartGroupLoadConfiguration;

   private Topic<YoEntryListDefinition> yoEntryListAdd;
   private Topic<ImmutablePair<String, YoChartConfigurationDefinition>> yoChartListAdd;
   private Topic<Pair<Window, ChartTable2DSize>> yoChartGroupResize;

   private Topic<File> yoMultiSliderboardSave;
   private Topic<File> yoMultiSliderboardLoad;
   private Topic<Boolean> yoMultiSliderboardClearAll;
   private Topic<YoSliderboardListDefinition> yoMultiSliderboardSet;
   private Topic<YoSliderboardDefinition> yoSliderboardSet;
   private Topic<Pair<String, YoSliderboardType>> yoSliderboardRemove;

   private Topic<ImmutableTriple<String, YoSliderboardType, YoButtonDefinition>> yoSliderboardSetButton;
   private Topic<ImmutableTriple<String, YoSliderboardType, YoKnobDefinition>> yoSliderboardSetKnob;
   private Topic<ImmutableTriple<String, YoSliderboardType, YoSliderDefinition>> yoSliderboardSetSlider;
   private Topic<ImmutableTriple<String, YoSliderboardType, Integer>> yoSliderboardClearButton;
   private Topic<ImmutableTriple<String, YoSliderboardType, Integer>> yoSliderboardClearKnob;
   private Topic<ImmutableTriple<String, YoSliderboardType, Integer>> yoSliderboardClearSlider;

   private Topic<Integer> controlsNumberPrecision;

   private Topic<File> sessionVisualizerConfigurationLoadRequest;
   private Topic<File> sessionVisualizerConfigurationSaveRequest;
   private Topic<Boolean> sessionVisualizerDefaultConfigurationLoadRequest;
   private Topic<Boolean> sessionVisualizerDefaultConfigurationSaveRequest;

   private Topic<SessionDataFilterParameters> sessionDataFilterParametersAddRequest;

   // Session topics
   private Topic<Session> startNewSessionRequest;
   private Topic<OpenSessionControlsRequest> openSessionControlsRequest;
   private Topic<OpenAddLogRequest> openAddLogRequest;
   private Topic<File> openLogDirectoryRequest;
   private Topic<File> openMCAPLogFileRequest;
   private Topic<BindSynchronizingVariablesRequest> bindSynchronizingVariablesRequest;

   public void setupTopics()
   {
      disableUserControls = SessionVisualizerMessagerAPI.DisableUserControls;
      sceneVideoRecordingRequest = SessionVisualizerMessagerAPI.SceneVideoRecordingRequest;
      camera3DRequest = SessionVisualizerMessagerAPI.Camera3DRequest;
      takeSnapshot = SessionVisualizerMessagerAPI.TakeSnapshot;
      registerRecordable = SessionVisualizerMessagerAPI.RegisterRecordable;
      forgetRecordable = SessionVisualizerMessagerAPI.ForgetRecordable;
      showAdvancedControls = SessionVisualizerMessagerAPI.ShowAdvancedControls;
      showOverheadPlotter = SessionVisualizerMessagerAPI.ShowOverheadPlotter;
      robotVisualRequest = SessionVisualizerMessagerAPI.RobotVisualRequest;
      terrainVisualRequest = SessionVisualizerMessagerAPI.TerrainVisualRequest;
      openWindowRequest = SessionVisualizerMessagerAPI.OpenWindowRequest;
      sessionVisualizerCloseRequest = SessionVisualizerMessagerAPI.SessionVisualizerCloseRequest;

      toggleKeyFrame = SessionVisualizerMessagerAPI.KeyFrame.ToggleKeyFrame;
      requestCurrentKeyFrames = SessionVisualizerMessagerAPI.KeyFrame.RequestCurrentKeyFrames;
      goToNextKeyFrame = SessionVisualizerMessagerAPI.KeyFrame.GoToNextKeyFrame;
      goToPreviousKeyFrame = SessionVisualizerMessagerAPI.KeyFrame.GoToPreviousKeyFrame;
      currentKeyFrames = SessionVisualizerMessagerAPI.KeyFrame.CurrentKeyFrames;

      yoSearchEngine = SessionVisualizerMessagerAPI.YoSearch.YoSearchEngine;
      yoSearchMaxListSize = SessionVisualizerMessagerAPI.YoSearch.YoSearchMaxListSize;
      yoCompositePatternLoadRequest = SessionVisualizerMessagerAPI.YoSearch.YoCompositePatternLoadRequest;
      yoCompositePatternSaveRequest = SessionVisualizerMessagerAPI.YoSearch.YoCompositePatternSaveRequest;
      yoCompositeSelected = SessionVisualizerMessagerAPI.YoSearch.YoCompositePatternSelected;
      yoCompositeRefreshAll = SessionVisualizerMessagerAPI.YoSearch.YoCompositeRefreshAll;
      showSCS2YoVariables = SessionVisualizerMessagerAPI.YoSearch.ShowSCS2YoVariables;
      yoVariableNameDisplay = SessionVisualizerMessagerAPI.YoSearch.YoVariableNameDisplay;

      yoGraphicLoadRequest = SessionVisualizerMessagerAPI.YoGraphic.YoGraphicLoadRequest;
      yoGraphicSaveRequest = SessionVisualizerMessagerAPI.YoGraphic.YoGraphicSaveRequest;
      removeYoGraphicRequest = SessionVisualizerMessagerAPI.YoGraphic.RemoveYoGraphicRequest;
      setYoGraphicVisibleRequest = SessionVisualizerMessagerAPI.YoGraphic.SetYoGraphicVisibleRequest;
      addYoGraphicRequest = SessionVisualizerMessagerAPI.YoGraphic.AddYoGraphicRequest;
      plotter2DTrackCoordinateRequest = SessionVisualizerMessagerAPI.YoGraphic.Plotter2DTrackCoordinateRequest;

      yoChartZoomFactor = SessionVisualizerMessagerAPI.YoChart.YoChartZoomFactor;
      yoChartRequestZoomIn = SessionVisualizerMessagerAPI.YoChart.YoChartRequestZoomIn;
      yoChartRequestZoomOut = SessionVisualizerMessagerAPI.YoChart.YoChartRequestZoomOut;
      yoChartRequestShift = SessionVisualizerMessagerAPI.YoChart.YoChartRequestShift;
      yoChartShowYAxis = SessionVisualizerMessagerAPI.YoChart.YoChartShowYAxis;
      yoChartGroupSaveConfiguration = SessionVisualizerMessagerAPI.YoChart.YoChartGroupSaveConfiguration;
      yoChartGroupLoadConfiguration = SessionVisualizerMessagerAPI.YoChart.YoChartGroupLoadConfiguration;

      yoEntryListAdd = SessionVisualizerMessagerAPI.YoEntry.YoEntryListAdd;
      yoChartListAdd = SessionVisualizerMessagerAPI.YoChart.YoChartListAdd;
      yoChartGroupResize = SessionVisualizerMessagerAPI.YoChart.YoChartGroupResize;

      yoMultiSliderboardSave = SessionVisualizerMessagerAPI.YoSliderboard.YoMultiSliderboardSave;
      yoMultiSliderboardLoad = SessionVisualizerMessagerAPI.YoSliderboard.YoMultiSliderboardLoad;
      yoMultiSliderboardClearAll = SessionVisualizerMessagerAPI.YoSliderboard.YoMultiSliderboardClearAll;
      yoMultiSliderboardSet = SessionVisualizerMessagerAPI.YoSliderboard.YoMultiSliderboardSet;
      yoSliderboardSet = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardSet;
      yoSliderboardRemove = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardRemove;
      yoSliderboardSetButton = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardSetButton;
      yoSliderboardSetKnob = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardSetKnob;
      yoSliderboardSetSlider = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardSetSlider;
      yoSliderboardClearButton = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardClearButton;
      yoSliderboardClearKnob = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardClearKnob;
      yoSliderboardClearSlider = SessionVisualizerMessagerAPI.YoSliderboard.YoSliderboardClearSlider;

      controlsNumberPrecision = SessionVisualizerMessagerAPI.ControlsNumberPrecision;

      sessionVisualizerConfigurationLoadRequest = SessionVisualizerMessagerAPI.SessionVisualizerConfigurationLoadRequest;
      sessionVisualizerConfigurationSaveRequest = SessionVisualizerMessagerAPI.SessionVisualizerConfigurationSaveRequest;
      sessionVisualizerDefaultConfigurationLoadRequest = SessionVisualizerMessagerAPI.SessionVisualizerDefaultConfigurationLoadRequest;
      sessionVisualizerDefaultConfigurationSaveRequest = SessionVisualizerMessagerAPI.SessionVisualizerDefaultConfigurationSaveRequest;

      sessionDataFilterParametersAddRequest = SessionVisualizerMessagerAPI.SessionDataFilterParametersAddRequest;

      startNewSessionRequest = SessionVisualizerMessagerAPI.SessionAPI.StartNewSessionRequest;
      openSessionControlsRequest = SessionVisualizerMessagerAPI.SessionAPI.OpenSessionControlsRequest;
      openAddLogRequest = SessionVisualizerMessagerAPI.SessionAPI.OpenAddLogRequest;
      openLogDirectoryRequest = SessionVisualizerMessagerAPI.SessionAPI.OpenLogDirectoryRequest;
      openMCAPLogFileRequest = SessionVisualizerMessagerAPI.SessionAPI.OpenMCAPLogFileRequest;
      bindSynchronizingVariablesRequest = SessionVisualizerMessagerAPI.SessionAPI.BindSynchronizingVariablesRequest;
   }

   public Topic<Boolean> getDisableUserControls()
   {
      return disableUserControls;
   }

   public Topic<SceneVideoRecordingRequest> getSceneVideoRecordingRequest()
   {
      return sceneVideoRecordingRequest;
   }

   public Topic<Camera3DRequest> getCamera3DRequest()
   {
      return camera3DRequest;
   }

   public Topic<Object> getTakeSnapshot()
   {
      return takeSnapshot;
   }

   public Topic<Object> getRegisterRecordable()
   {
      return registerRecordable;
   }

   public Topic<Object> getForgetRecordable()
   {
      return forgetRecordable;
   }

   public Topic<Boolean> getShowAdvancedControls()
   {
      return showAdvancedControls;
   }

   public Topic<Boolean> getShowOverheadPlotter()
   {
      return showOverheadPlotter;
   }

   public Topic<NewRobotVisualRequest> getRobotVisualRequest()
   {
      return robotVisualRequest;
   }

   public Topic<NewTerrainVisualRequest> getTerrainVisualRequest()
   {
      return terrainVisualRequest;
   }

   public Topic<NewWindowRequest> getOpenWindowRequest()
   {
      return openWindowRequest;
   }

   public Topic<Boolean> getSessionVisualizerCloseRequest()
   {
      return sessionVisualizerCloseRequest;
   }

   public Topic<Object> getToggleKeyFrame()
   {
      return toggleKeyFrame;
   }

   public Topic<Object> getRequestCurrentKeyFrames()
   {
      return requestCurrentKeyFrames;
   }

   public Topic<Object> getGoToNextKeyFrame()
   {
      return goToNextKeyFrame;
   }

   public Topic<Object> getGoToPreviousKeyFrame()
   {
      return goToPreviousKeyFrame;
   }

   public Topic<int[]> getCurrentKeyFrames()
   {
      return currentKeyFrames;
   }

   public Topic<SearchEngines> getYoSearchEngine()
   {
      return yoSearchEngine;
   }

   public Topic<Integer> getYoSearchMaxListSize()
   {
      return yoSearchMaxListSize;
   }

   public Topic<File> getYoCompositePatternLoadRequest()
   {
      return yoCompositePatternLoadRequest;
   }

   public Topic<File> getYoCompositePatternSaveRequest()
   {
      return yoCompositePatternSaveRequest;
   }

   public Topic<List<String>> getYoCompositeSelected()
   {
      return yoCompositeSelected;
   }

   public Topic<Boolean> getYoCompositeRefreshAll()
   {
      return yoCompositeRefreshAll;
   }

   public Topic<Boolean> getShowSCS2YoVariables()
   {
      return showSCS2YoVariables;
   }

   public Topic<YoNameDisplay> getYoVariableNameDisplay()
   {
      return yoVariableNameDisplay;
   }

   public Topic<File> getYoGraphicLoadRequest()
   {
      return yoGraphicLoadRequest;
   }

   public Topic<File> getYoGraphicSaveRequest()
   {
      return yoGraphicSaveRequest;
   }

   public Topic<String> getRemoveYoGraphicRequest()
   {
      return removeYoGraphicRequest;
   }

   public Topic<Pair<String, Boolean>> getSetYoGraphicVisibleRequest()
   {
      return setYoGraphicVisibleRequest;
   }

   public Topic<YoGraphicDefinition> getAddYoGraphicRequest()
   {
      return addYoGraphicRequest;
   }

   public Topic<YoTuple2DDefinition> getPlotter2DTrackCoordinateRequest()
   {
      return plotter2DTrackCoordinateRequest;
   }

   public Topic<Pair<Window, Double>> getYoChartZoomFactor()
   {
      return yoChartZoomFactor;
   }

   public Topic<Pair<Window, Boolean>> getYoChartRequestZoomIn()
   {
      return yoChartRequestZoomIn;
   }

   public Topic<Pair<Window, Boolean>> getYoChartRequestZoomOut()
   {
      return yoChartRequestZoomOut;
   }

   public Topic<Pair<Window, Integer>> getYoChartRequestShift()
   {
      return yoChartRequestShift;
   }

   public Topic<Pair<Window, Boolean>> getYoChartShowYAxis()
   {
      return yoChartShowYAxis;
   }

   public Topic<Pair<Window, File>> getYoChartGroupLoadConfiguration()
   {
      return yoChartGroupLoadConfiguration;
   }

   public Topic<Pair<Window, File>> getYoChartGroupSaveConfiguration()
   {
      return yoChartGroupSaveConfiguration;
   }

   public Topic<Pair<Window, ChartTable2DSize>> getYoChartGroupResize()
   {
      return yoChartGroupResize;
   }

   public Topic<YoEntryListDefinition> getYoEntryListAdd()
   {
      return yoEntryListAdd;
   }

   public Topic<ImmutablePair<String, YoChartConfigurationDefinition>> getYoChartListAdd()
   {
      return yoChartListAdd;
   }

   public Topic<File> getYoMultiSliderboardSave()
   {
      return yoMultiSliderboardSave;
   }

   public Topic<File> getYoMultiSliderboardLoad()
   {
      return yoMultiSliderboardLoad;
   }

   public Topic<Boolean> getYoMultiSliderboardClearAll()
   {
      return yoMultiSliderboardClearAll;
   }

   public Topic<YoSliderboardListDefinition> getYoMultiSliderboardSet()
   {
      return yoMultiSliderboardSet;
   }

   public Topic<YoSliderboardDefinition> getYoSliderboardSet()
   {
      return yoSliderboardSet;
   }

   public Topic<Pair<String, YoSliderboardType>> getYoSliderboardRemove()
   {
      return yoSliderboardRemove;
   }

   public Topic<ImmutableTriple<String, YoSliderboardType, YoButtonDefinition>> getYoSliderboardSetButton()
   {
      return yoSliderboardSetButton;
   }

   public Topic<ImmutableTriple<String, YoSliderboardType, YoKnobDefinition>> getYoSliderboardSetKnob()
   {
      return yoSliderboardSetKnob;
   }

   public Topic<ImmutableTriple<String, YoSliderboardType, YoSliderDefinition>> getYoSliderboardSetSlider()
   {
      return yoSliderboardSetSlider;
   }

   public Topic<ImmutableTriple<String, YoSliderboardType, Integer>> getYoSliderboardClearButton()
   {
      return yoSliderboardClearButton;
   }

   public Topic<ImmutableTriple<String, YoSliderboardType, Integer>> getYoSliderboardClearKnob()
   {
      return yoSliderboardClearKnob;
   }

   public Topic<ImmutableTriple<String, YoSliderboardType, Integer>> getYoSliderboardClearSlider()
   {
      return yoSliderboardClearSlider;
   }

   public Topic<Integer> getControlsNumberPrecision()
   {
      return controlsNumberPrecision;
   }

   public Topic<File> getSessionVisualizerConfigurationLoadRequest()
   {
      return sessionVisualizerConfigurationLoadRequest;
   }

   public Topic<File> getSessionVisualizerConfigurationSaveRequest()
   {
      return sessionVisualizerConfigurationSaveRequest;
   }

   public Topic<Boolean> getSessionVisualizerDefaultConfigurationLoadRequest()
   {
      return sessionVisualizerDefaultConfigurationLoadRequest;
   }

   public Topic<Boolean> getSessionVisualizerDefaultConfigurationSaveRequest()
   {
      return sessionVisualizerDefaultConfigurationSaveRequest;
   }

   public Topic<SessionDataFilterParameters> getSessionDataFilterParametersAddRequest()
   {
      return sessionDataFilterParametersAddRequest;
   }

   public Topic<Session> getStartNewSessionRequest()
   {
      return startNewSessionRequest;
   }

   public Topic<OpenSessionControlsRequest> getOpenSessionControlsRequest()
   {
      return openSessionControlsRequest;
   }

   public Topic<OpenAddLogRequest> getOpenAddLogRequest()
   {
      return openAddLogRequest;
   }

   public Topic<File> getOpenLogDirectoryRequest()
   {
      return openLogDirectoryRequest;
   }

   public Topic<File> getOpenMCAPLogFileRequest()
   {
      return openMCAPLogFileRequest;
   }

   public Topic<BindSynchronizingVariablesRequest> getBindSynchronizingVariablesRequest()
   {
      return bindSynchronizingVariablesRequest;
   }
}
