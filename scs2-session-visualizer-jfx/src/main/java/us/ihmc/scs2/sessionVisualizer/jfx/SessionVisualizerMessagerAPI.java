package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;
import java.util.List;

import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoEntry.YoEntryListDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.SearchEngines;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.NewTerrainVisualRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest;

public class SessionVisualizerMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category APIRoot = apiFactory.createRootCategory("SessionVisualizer");
   private static final CategoryTheme Register = apiFactory.createCategoryTheme("Register");
   private static final CategoryTheme Forget = apiFactory.createCategoryTheme("Forget");
   private static final CategoryTheme Controls = apiFactory.createCategoryTheme("Controls");
   private static final CategoryTheme Advanced = apiFactory.createCategoryTheme("Advanced");
   private static final CategoryTheme OverheadPlotter = apiFactory.createCategoryTheme("OverheadPlotter");
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
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
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

   public static final Topic<Boolean> DisableUserControls = APIRoot.child(User).child(Controls).topic(Disable);
   public static final Topic<SceneVideoRecordingRequest> SceneVideoRecordingRequest = APIRoot.child(Video).topic(Request);
   public static final Topic<CameraObjectTrackingRequest> CameraTrackObject = APIRoot.child(Camera).child(Track).topic(Request);
   public static final Topic<Object> TakeSnapshot = APIRoot.topic(Snapshot);
   public static final Topic<Object> RegisterRecordable = APIRoot.child(Register).topic(Recordable);
   public static final Topic<Object> ForgetRecordable = APIRoot.child(Forget).topic(Recordable);
   public static final Topic<Boolean> ShowAdvancedControls = APIRoot.child(Controls).child(Advanced).topic(Show);
   public static final Topic<Boolean> ShowOverheadPlotter = APIRoot.child(OverheadPlotter).topic(Show);
   public static final Topic<NewRobotVisualRequest> RobotVisualRequest = APIRoot.child(Robot).child(Visual).topic(Request);
   public static final Topic<NewTerrainVisualRequest> TerrainVisualRequest = APIRoot.child(Terrain).child(Visual).topic(Request);
   public static final Topic<NewWindowRequest> OpenWindowRequest = APIRoot.topic(Open);
   public static final Topic<Boolean> SessionVisualizerCloseRequest = APIRoot.topic(Close);
   public static final Topic<Integer> ControlsNumberPrecision = APIRoot.child(Controls).topic(Precision); // TODO Not the greatest topic name, nor the best place.
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
      private static final CategoryTheme KeyFrame = apiFactory.createCategoryTheme("KeyFrame");

      private static final TopicTheme Current = apiFactory.createTopicTheme("Current");

      public static final Topic<Object> ToggleKeyFrame = APIRoot.child(KeyFrame).topic(Toggle);
      public static final Topic<Object> GoToNextKeyFrame = APIRoot.child(KeyFrame).topic(Next);
      public static final Topic<Object> GoToPreviousKeyFrame = APIRoot.child(KeyFrame).topic(Previous);
      public static final Topic<Object> RequestCurrentKeyFrames = APIRoot.child(KeyFrame).topic(Request);

      public static final Topic<int[]> CurrentKeyFrames = APIRoot.child(KeyFrame).topic(Current);
   }

   public static class YoSearch
   {
      private static final CategoryTheme YoSearch = apiFactory.createCategoryTheme("YoSearch");
      private static final CategoryTheme YoCompositePattern = apiFactory.createCategoryTheme("YoCompositePattern");

      private static final TypedTopicTheme<SearchEngines> SearchEngine = apiFactory.createTypedTopicTheme("SearchEngine");
      private static final TopicTheme Selected = apiFactory.createTopicTheme("Selected");
      private static final TopicTheme Refresh = apiFactory.createTopicTheme("Refresh");

      public static final Topic<SearchEngines> YoSearchEngine = APIRoot.child(YoSearch).topic(SearchEngine);
      public static final Topic<Integer> YoSearchMaxListSize = APIRoot.child(YoSearch).topic(Size);
      public static final Topic<File> YoCompositePatternLoadRequest = APIRoot.child(YoSearch).child(YoCompositePattern).topic(Load);
      public static final Topic<File> YoCompositePatternSaveRequest = APIRoot.child(YoSearch).child(YoCompositePattern).topic(Save);
      public static final Topic<List<String>> YoCompositePatternSelected = APIRoot.child(YoSearch).child(YoCompositePattern).topic(Selected);
      public static final Topic<Boolean> YoCompositeRefreshAll = APIRoot.child(YoSearch).child(YoCompositePattern).topic(Refresh);
      public static final Topic<Boolean> ShowSCS2YoVariables = APIRoot.child(YoSearch).child(Debug).topic(Show);
   }

   public static class YoGraphic
   {
      private static final CategoryTheme YoGraphic = apiFactory.createCategoryTheme("YoGraphic");
      private static final CategoryTheme Plotter2D = apiFactory.createCategoryTheme("Plotter2D");

      public static final Topic<File> YoGraphicSaveRequest = APIRoot.child(YoGraphic).topic(Save);
      public static final Topic<File> YoGraphicLoadRequest = APIRoot.child(YoGraphic).topic(Load);

      public static final Topic<String> RemoveYoGraphicRequest = APIRoot.child(YoGraphic).topic(Remove);
      public static final Topic<Pair<String, Boolean>> SetYoGraphicVisibleRequest = APIRoot.child(YoGraphic).topic(Visible);
      public static final Topic<YoGraphicDefinition> AddYoGraphicRequest = APIRoot.child(YoGraphic).topic(Add);
      public static final Topic<YoTuple2DDefinition> Plotter2DTrackCoordinateRequest = APIRoot.child(YoGraphic).child(Plotter2D).child(Track).topic(Request);
   }

   public static class YoChart
   {
      private static final CategoryTheme YoChart = apiFactory.createCategoryTheme("YoChart");
      private static final CategoryTheme Zoom = apiFactory.createCategoryTheme("Zoom");
      private static final CategoryTheme In = apiFactory.createCategoryTheme("In");
      private static final CategoryTheme Out = apiFactory.createCategoryTheme("Out");
      private static final CategoryTheme Shift = apiFactory.createCategoryTheme("Shift");

      private static final TopicTheme Factor = apiFactory.createTopicTheme("Factor");

      public static final Topic<Pair<Window, Double>> YoChartZoomFactor = APIRoot.child(YoChart).child(Zoom).topic(Factor);
      public static final Topic<Pair<Window, Boolean>> YoChartRequestZoomIn = APIRoot.child(YoChart).child(Zoom).child(In).topic(Request);
      public static final Topic<Pair<Window, Boolean>> YoChartRequestZoomOut = APIRoot.child(YoChart).child(Zoom).child(Out).topic(Request);
      public static final Topic<Pair<Window, Integer>> YoChartRequestShift = APIRoot.child(YoChart).child(Shift).topic(Request);
      public static final Topic<Pair<Window, File>> YoChartGroupSaveConfiguration = APIRoot.child(YoChart).child(Group).child(Configuration).topic(Save);
      public static final Topic<Pair<Window, File>> YoChartGroupLoadConfiguration = APIRoot.child(YoChart).child(Group).child(Configuration).topic(Load);
      public static final Topic<Pair<Window, String>> YoChartGroupName = APIRoot.child(YoChart).child(Group).topic(Name);
   }

   public static class YoEntry
   {
      private static final CategoryTheme YoEntry = apiFactory.createCategoryTheme("YoEntry");
      private static final CategoryTheme List = apiFactory.createCategoryTheme("List");

      public static final Topic<YoEntryListDefinition> YoEntryListAdd = APIRoot.child(YoEntry).child(List).topic(Add);
   }

   public static class YoSliderboard
   {
      private static final CategoryTheme YoSliderboard = apiFactory.createCategoryTheme("YoSliderboard");
      private static final CategoryTheme Multi = apiFactory.createCategoryTheme("Multi");
      private static final CategoryTheme Single = apiFactory.createCategoryTheme("Single");
      private static final CategoryTheme Button = apiFactory.createCategoryTheme("Button");
      private static final CategoryTheme Knob = apiFactory.createCategoryTheme("Knob");
      private static final CategoryTheme Slider = apiFactory.createCategoryTheme("Slider");

      public static final Topic<File> YoMultiSliderboardSave = APIRoot.child(YoSliderboard).child(Multi).child(Configuration).topic(Save);
      public static final Topic<File> YoMultiSliderboardLoad = APIRoot.child(YoSliderboard).child(Multi).child(Configuration).topic(Load);
      public static final Topic<Boolean> YoMultiSliderboardClearAll = APIRoot.child(Multi).child(YoSliderboard).topic(Remove);
      public static final Topic<YoSliderboardListDefinition> YoMultiSliderboardSet = APIRoot.child(Multi).child(YoSliderboard).topic(Set);
      public static final Topic<YoSliderboardDefinition> YoSliderboardSet = APIRoot.child(YoSliderboard).child(Single).topic(Set);
      public static final Topic<String> YoSliderboardRemove = APIRoot.child(YoSliderboard).child(Single).topic(Remove);

      public static final Topic<Pair<String, YoButtonDefinition>> YoSliderboardSetButton = APIRoot.child(YoSliderboard).child(Single).child(Button).topic(Set);
      public static final Topic<Pair<String, YoKnobDefinition>> YoSliderboardSetKnob = APIRoot.child(YoSliderboard).child(Single).child(Knob).topic(Set);
      public static final Topic<Pair<String, YoSliderDefinition>> YoSliderboardSetSlider = APIRoot.child(YoSliderboard).child(Single).child(Slider).topic(Set);
      public static final Topic<Pair<String, Integer>> YoSliderboardClearButton = APIRoot.child(YoSliderboard).child(Single).child(Button).topic(Remove);
      public static final Topic<Pair<String, Integer>> YoSliderboardClearKnob = APIRoot.child(YoSliderboard).child(Single).child(Knob).topic(Remove);
      public static final Topic<Pair<String, Integer>> YoSliderboardClearSlider = APIRoot.child(YoSliderboard).child(Single).child(Slider).topic(Remove);
   }

   public static class SessionAPI
   {
      private static final CategoryTheme Session = apiFactory.createCategoryTheme("Session");
      private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");

      public static final Topic<Session> StartNewSessionRequest = APIRoot.child(Session).child(Start).topic(Request);
      public static final Topic<OpenSessionControlsRequest> OpenSessionControlsRequest = APIRoot.child(Session).child(Controls).topic(Request);
   }

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
