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
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category APIRoot = apiFactory.createRootCategory("SessionVisualizer");
   private static final CategoryTheme Register = apiFactory.createCategoryTheme("Register");
   private static final CategoryTheme Forget = apiFactory.createCategoryTheme("Forget");
   private static final CategoryTheme Controls = apiFactory.createCategoryTheme("Controls");
   private static final CategoryTheme Advanced = apiFactory.createCategoryTheme("Advanced");
   private static final CategoryTheme OverheadPlotter = apiFactory.createCategoryTheme("OverheadPlotter");
   private static final CategoryTheme Perception = apiFactory.createCategoryTheme("Perception");
   private static final CategoryTheme HeightMap = apiFactory.createCategoryTheme("HeightMap");
   private static final CategoryTheme Group = apiFactory.createCategoryTheme("Group");
   private static final CategoryTheme Configuration = apiFactory.createCategoryTheme("Configuration");
   private static final CategoryTheme Default = apiFactory.createCategoryTheme("Default");
   private static final CategoryTheme Camera = apiFactory.createCategoryTheme("Camera");
   private static final CategoryTheme Track = apiFactory.createCategoryTheme("Track");
   private static final CategoryTheme Video = apiFactory.createCategoryTheme("Video");
   private static final CategoryTheme User = apiFactory.createCategoryTheme("User");
   private static final CategoryTheme Debug = apiFactory.createCategoryTheme("Debug");
   private static final CategoryTheme Robot = apiFactory.createCategoryTheme("Robot");
   private static final CategoryTheme Terrain = apiFactory.createCategoryTheme("Terrain");
   private static final CategoryTheme Visual = apiFactory.createCategoryTheme("Visual");
   private static final CategoryTheme SessionData = apiFactory.createCategoryTheme("SessionData");
   private static final CategoryTheme Filter = apiFactory.createCategoryTheme("Filter");

   private static final TopicTheme Toggle = apiFactory.createTopicTheme("Toggle");
   private static final TopicTheme Next = apiFactory.createTopicTheme("Next");
   private static final TopicTheme Previous = apiFactory.createTopicTheme("Previous");
   private static final TopicTheme Snapshot = apiFactory.createTopicTheme("Snapshot");
   private static final TopicTheme Recordable = apiFactory.createTypedTopicTheme("Recordable");
   private static final TopicTheme Request = apiFactory.createTopicTheme("Request");
   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TopicTheme Show = apiFactory.createTypedTopicTheme("Show");
   private static final TopicTheme Load = apiFactory.createTopicTheme("load");
   private static final TopicTheme Save = apiFactory.createTopicTheme("save");
   private static final TopicTheme Close = apiFactory.createTopicTheme("close");
   private static final TopicTheme Open = apiFactory.createTopicTheme("open");
   private static final TopicTheme Name = apiFactory.createTopicTheme("name");
   private static final TopicTheme Precision = apiFactory.createTopicTheme("Precision");
   private static final TopicTheme Disable = apiFactory.createTopicTheme("Disable");
   private static final TopicTheme Add = apiFactory.createTopicTheme("add");
   private static final TopicTheme Set = apiFactory.createTopicTheme("set");
   private static final TopicTheme Remove = apiFactory.createTopicTheme("remove");
   private static final TopicTheme Visible = apiFactory.createTopicTheme("visible");
   private static final TopicTheme Resize = apiFactory.createTopicTheme("resize");

   public static final Topic<Boolean> DisableUserControls = APIRoot.child(User).child(Controls).topic(Disable);
   public static final Topic<SceneVideoRecordingRequest> SceneVideoRecordingRequest = APIRoot.child(Video).topic(Request);
   public static final Topic<Camera3DRequest> Camera3DRequest = APIRoot.child(Camera).child(Configuration).topic(Request);
   public static final Topic<Object> TakeSnapshot = APIRoot.topic(Snapshot);
   public static final Topic<Object> RegisterRecordable = APIRoot.child(Register).topic(Recordable);
   public static final Topic<Object> ForgetRecordable = APIRoot.child(Forget).topic(Recordable);
   public static final Topic<Boolean> ShowAdvancedControls = APIRoot.child(Controls).child(Advanced).topic(Show);
   public static final Topic<Boolean> ShowOverheadPlotter = APIRoot.child(OverheadPlotter).topic(Show);
   public static final Topic<Boolean> ShowHeightMap = APIRoot.child(Perception).child(HeightMap).topic(Show);
   public static final Topic<NewRobotVisualRequest> RobotVisualRequest = APIRoot.child(Robot).child(Visual).topic(Request);
   public static final Topic<NewTerrainVisualRequest> TerrainVisualRequest = APIRoot.child(Terrain).child(Visual).topic(Request);
   public static final Topic<NewWindowRequest> OpenWindowRequest = APIRoot.topic(Open);
   public static final Topic<Boolean> SessionVisualizerCloseRequest = APIRoot.topic(Close);
   public static final Topic<Integer> ControlsNumberPrecision = APIRoot.child(Controls)
                                                                       .topic(Precision); // TODO Not the greatest topic name, nor the best place.
   public static final Topic<File> SessionVisualizerConfigurationLoadRequest = APIRoot.child(Configuration).topic(Load);
   public static final Topic<Boolean> SessionVisualizerDefaultConfigurationLoadRequest = APIRoot.child(Configuration).child(Default).topic(Load);
   public static final Topic<File> SessionVisualizerConfigurationSaveRequest = APIRoot.child(Configuration).topic(Save);
   public static final Topic<Boolean> SessionVisualizerDefaultConfigurationSaveRequest = APIRoot.child(Configuration).child(Default).topic(Save);
   public static final Topic<SessionDataFilterParameters> SessionDataFilterParametersAddRequest = APIRoot.child(SessionData).child(Filter).topic(Add);

   static
   { // Ensure that the KeyFrame is loaded before closing the API.
      new KeyFrame();
      new YoSearch();
      new YoGraphic();
      new YoChart();
      new YoEntry();
      new YoSliderboard();
      new SessionAPI();
   }

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
