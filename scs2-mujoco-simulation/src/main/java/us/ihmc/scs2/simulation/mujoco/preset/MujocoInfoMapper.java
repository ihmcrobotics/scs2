package us.ihmc.scs2.simulation.mujoco.preset;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

// @formatter:off
@Properties(value =
@Platform(value = "linux",
      includepath = {"../../install/mujoco/include"},
      // mujoco.h is the umbrella header but JavaCPP only emits Java classes for symbols that
      // come from headers explicitly listed here. The per-struct headers must each be listed so
      // mjModel/mjData/mjSpec/mjVFS/mjLROpt/etc. become real Java types instead of dangling
      // references. mjxmacro.h is intentionally omitted -- it is X-macro preprocessor magic for
      // dispatch tables and JavaCPP cannot make sense of it.
      include = {"mujoco/mjtnum.h",
                 "mujoco/mjexport.h",
                 "mujoco/mjsan.h",
                 "mujoco/mjmacro.h",
                 "mujoco/mjthread.h",
                 "mujoco/mjplugin.h",
                 "mujoco/mjmodel.h",
                 "mujoco/mjdata.h",
                 "mujoco/mjvisualize.h",
                 "mujoco/mjrender.h",
                 "mujoco/mjui.h",
                 "mujoco/mjspec.h",
                 "mujoco/mujoco.h"},
      linkpath = "../../install/mujoco/lib",
      link = "mujoco",
      preload = {"mujoco", "jniMujoco"}
),
      target = "us.ihmc.scs2.simulation.mujoco.Mujoco"
)
// @formatter:on

public class MujocoInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
      // mjtNum is a typedef alias for double in standard MuJoCo builds. Expose it as double on the Java side so
      // DoublePointer maps naturally and there is no surprise when reading qpos/qvel/ctrl arrays.
      infoMap.put(new Info("mjtNum").cast().valueTypes("double").pointerTypes("DoublePointer"));
      infoMap.put(new Info("mjtByte").cast().valueTypes("byte").pointerTypes("BytePointer"));

      // mjspec.h defines C++ std-alias types (mjString = std::string, mjStringVec = vector<string>,
      // mjByteVec = vector<std::byte>, etc.). JavaCPP can't cleanly round-trip these through JNI
      // -- it tries to apply the StringAdapter pattern and ends up generating invalid C++. The
      // mjSpec model-editing API is out of scope for v1, so expose these as opaque Pointers and
      // skip the setter logic. Callers that want mjSpec can re-enable individually with proper
      // type mappings later.
      infoMap.put(new Info("mjString", "mjStringVec", "mjIntVec", "mjIntVecVec", "mjFloatVec",
                           "mjByteVec", "mjDoubleVec", "mjFloatVecVec")
            .cast().pointerTypes("Pointer"));

      // MuJoCo headers include the MUJOCO_API and MJAPI macros on every public symbol. They must be visible as
      // empty so the JavaCPP parser does not choke on them.
      infoMap.put(new Info("MUJOCO_API", "MJAPI").cppTypes().annotations());

      // MuJoCo's headers typedef the implementation structs (e.g. `struct mjModel_ { ... };
      // typedef struct mjModel_ mjModel;`). JavaCPP emits two parallel Java classes for each pair:
      // the full struct with field accessors (e.g. `mjModel_`) and an `@Opaque` pointer alias
      // (e.g. `mjModel`). The MuJoCo public functions return/accept the opaque alias, which means
      // a caller of `mj_loadXML(...)` cannot reach the `qpos` / `jnt_qposadr` fields. Tell
      // JavaCPP to use the full struct as the Java type for both names so callers see fields
      // directly on whatever `mj_loadXML` and friends return.
      infoMap.put(new Info("mjModel", "mjModel_").pointerTypes("mjModel"));
      infoMap.put(new Info("mjData", "mjData_").pointerTypes("mjData"));
      infoMap.put(new Info("mjOption", "mjOption_").pointerTypes("mjOption"));
      infoMap.put(new Info("mjVFS", "mjVFS_").pointerTypes("mjVFS"));
      infoMap.put(new Info("mjContact", "mjContact_").pointerTypes("mjContact"));
      infoMap.put(new Info("mjLROpt", "mjLROpt_").pointerTypes("mjLROpt"));
      infoMap.put(new Info("mjvScene", "mjvScene_").pointerTypes("mjvScene"));
      infoMap.put(new Info("mjvCamera", "mjvCamera_").pointerTypes("mjvCamera"));
      infoMap.put(new Info("mjvOption", "mjvOption_").pointerTypes("mjvOption"));
      infoMap.put(new Info("mjvGeom", "mjvGeom_").pointerTypes("mjvGeom"));
      infoMap.put(new Info("mjvPerturb", "mjvPerturb_").pointerTypes("mjvPerturb"));
      infoMap.put(new Info("mjvFigure", "mjvFigure_").pointerTypes("mjvFigure"));
      infoMap.put(new Info("mjvLight", "mjvLight_").pointerTypes("mjvLight"));
      infoMap.put(new Info("mjrContext", "mjrContext_").pointerTypes("mjrContext"));
      infoMap.put(new Info("mjrRect", "mjrRect_").pointerTypes("mjrRect"));
      infoMap.put(new Info("mjStatistic", "mjStatistic_").pointerTypes("mjStatistic"));
      infoMap.put(new Info("mjVisual", "mjVisual_").pointerTypes("mjVisual"));
      infoMap.put(new Info("mjuiState", "mjuiState_").pointerTypes("mjuiState"));
      infoMap.put(new Info("mjuiThemeSpacing", "mjuiThemeSpacing_").pointerTypes("mjuiThemeSpacing"));
      infoMap.put(new Info("mjuiThemeColor", "mjuiThemeColor_").pointerTypes("mjuiThemeColor"));
      infoMap.put(new Info("mjuiItem", "mjuiItem_").pointerTypes("mjuiItem"));
      infoMap.put(new Info("mjuiSection", "mjuiSection_").pointerTypes("mjuiSection"));
      infoMap.put(new Info("mjUI", "mjUI_").pointerTypes("mjUI"));
      infoMap.put(new Info("mjThreadPool", "mjThreadPool_").pointerTypes("mjThreadPool"));
      infoMap.put(new Info("mjTask", "mjTask_").pointerTypes("mjTask"));
      infoMap.put(new Info("mjResource", "mjResource_").pointerTypes("mjResource"));

      // The mju_user_* callbacks in mujoco.h are inline function-pointer extern declarations
      // (e.g. `MJAPI extern void (*mju_user_warning)(const char*);`). JavaCPP's parser can't
      // extract a clean Java name from that syntax for the void-returning ones and ends up
      // emitting a method literally named "void", which then fails javac. Skip the whole block
      // by line range -- the four lines are contiguous in upstream mujoco.h (start:
      // mju_user_error, end: mju_user_free). JavaCPP keys file Info entries by basename.
      infoMap.put(new Info("mujoco.h")
            .linePatterns(".*mju_user_error.*", ".*mju_user_free.*").skip());

      // The plugin / threadpool / file-system callback function pointers cannot be expressed as
      // Java methods out of the box; skip them in v1. Callers that need them can re-enable by
      // adding @Cast wrappers.
      infoMap.put(new Info("mjPLUGIN_LIB_INIT", "mjPRIVATE__getPluginConfig").skip());

      // mjplugin.h defines plugin entry-point glue macros (mjDLLMAIN = DllMain on Windows,
      // mjEXTERNC = extern "C"). They aren't meant to surface as Java constants -- the RHS is
      // C-only and javac chokes on it. Skip them.
      infoMap.put(new Info("mjDLLMAIN", "mjEXTERNC").skip());

      // mujoco.h aliases the C standard math functions under an mju_ prefix when running in
      // double precision (#define mju_sqrt sqrt, etc). The RHS is a libm symbol, not anything
      // JavaCPP can emit as a Java constant. Callers should use java.lang.Math instead.
      infoMap.put(new Info("mju_sqrt", "mju_exp", "mju_sin", "mju_cos", "mju_tan",
                           "mju_asin", "mju_acos", "mju_atan2", "mju_tanh", "mju_pow",
                           "mju_abs", "mju_log", "mju_log10", "mju_floor", "mju_ceil").skip());

      // mjexport.h defines DLL visibility attribute macros (__attribute__((visibility("default"))))
      // that JavaCPP otherwise tries to emit as integer Java constants. They must be no-op
      // pass-through (same recipe as MJAPI), NOT skip -- on Linux MJAPI expands to
      // MUJOCO_HELPER_DLL_IMPORT, so a skip here would drop every public function declaration.
      infoMap.put(new Info("MUJOCO_HELPER_DLL_IMPORT", "MUJOCO_HELPER_DLL_EXPORT",
                           "MUJOCO_HELPER_DLL_LOCAL").cppTypes().annotations());

      // mj_markStack / mj_freeStack are scratchpad-stack helpers declared in BOTH mjsan.h (as
      // static inline) and mujoco.h (as MJAPI extern). JavaCPP treats the second as a duplicate
      // and renames it with a doubled underscore (mj__markStack / mj__freeStack). We must skip
      // both the original AND the renamed copy. Not needed for v1 simulation either way.
      infoMap.put(new Info("mj_markStack", "mj_freeStack",
                           "mj__markStack", "mj__freeStack").skip());

      // The mjcb_* callback function-pointer globals (mjcb_passive, mjcb_control, etc.) are
      // exposed as int getters/setters because their types (mjfGeneric, mjfConFilt, etc.) are
      // opaque function-pointer typedefs that JavaCPP can't synthesise. The generated C++ then
      // tries to assign an int into a function-pointer variable. Skip the globals; users who
      // need control callbacks can register them via a separate dedicated wrapper later.
      infoMap.put(new Info("mjcb_passive", "mjcb_control", "mjcb_contactfilter", "mjcb_sensor",
                           "mjcb_time", "mjcb_act_dyn", "mjcb_act_gain", "mjcb_act_bias",
                           "mjCOLLISIONFUNC").skip());
   }
}
