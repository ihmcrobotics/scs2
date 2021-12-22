package us.ihmc.scs2.session;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.FillBufferRequest;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

public class YoSharedBufferMessagerAPI
{
   private YoSharedBufferMessagerAPI()
   {
   }

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category root = apiFactory.createRootCategory("YoSharedBufferAPI");

   private static final CategoryTheme Index = apiFactory.createCategoryTheme("Index");
   private static final CategoryTheme Increment = apiFactory.createCategoryTheme("Increment");
   private static final CategoryTheme Decrement = apiFactory.createCategoryTheme("Decrement");
   private static final CategoryTheme Initialize = apiFactory.createCategoryTheme("Initialize");

   private static final TypedTopicTheme<Integer> Current = apiFactory.createTypedTopicTheme("Current");
   private static final TypedTopicTheme<Integer> InPoint = apiFactory.createTypedTopicTheme("InPoint");
   private static final TypedTopicTheme<Integer> OutPoint = apiFactory.createTypedTopicTheme("OutPoint");
   private static final TypedTopicTheme<YoBufferPropertiesReadOnly> Properties = apiFactory.createTypedTopicTheme("Properties");

   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TypedTopicTheme<CropBufferRequest> Crop = apiFactory.createTypedTopicTheme("Crop");
   private static final TypedTopicTheme<FillBufferRequest> Fill = apiFactory.createTypedTopicTheme("Fill");

   public static final Topic<Integer> CurrentIndexRequest = root.child(Index).topic(Current);
   public static final Topic<Integer> IncrementCurrentIndexRequest = root.child(Index).child(Increment).topic(Current);
   public static final Topic<Integer> DecrementCurrentIndexRequest = root.child(Index).child(Decrement).topic(Current);
   public static final Topic<Integer> InPointIndexRequest = root.child(Index).topic(InPoint);
   public static final Topic<Integer> OutPointIndexRequest = root.child(Index).topic(OutPoint);
   public static final Topic<CropBufferRequest> CropRequest = root.topic(Crop);
   public static final Topic<FillBufferRequest> FillRequest = root.topic(Fill);
   public static final Topic<Integer> InitializeBufferSize = root.child(Initialize).topic(Size);
   public static final Topic<Integer> CurrentBufferSizeRequest = root.topic(Size);
   public static final Topic<YoBufferPropertiesReadOnly> CurrentBufferProperties = root.topic(Properties);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
