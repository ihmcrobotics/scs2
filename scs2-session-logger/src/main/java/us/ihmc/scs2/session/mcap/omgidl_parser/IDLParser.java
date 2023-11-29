// Generated from IDL.g4 by ANTLR 4.13.1
package us.ihmc.scs2.session.mcap.omgidl_parser;
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast", "CheckReturnValue"})
public class IDLParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.13.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		INTEGER_LITERAL=1, OCTAL_LITERAL=2, HEX_LITERAL=3, FLOATING_PT_LITERAL=4, 
		FIXED_PT_LITERAL=5, WIDE_CHARACTER_LITERAL=6, CHARACTER_LITERAL=7, WIDE_STRING_LITERAL=8, 
		STRING_LITERAL=9, BOOLEAN_LITERAL=10, SEMICOLON=11, COLON=12, COMMA=13, 
		LEFT_BRACE=14, RIGHT_BRACE=15, LEFT_BRACKET=16, RIGHT_BRACKET=17, LEFT_SQUARE_BRACKET=18, 
		RIGHT_SQUARE_BRACKET=19, TILDE=20, SLASH=21, LEFT_ANG_BRACKET=22, RIGHT_ANG_BRACKET=23, 
		STAR=24, PLUS=25, MINUS=26, CARET=27, AMPERSAND=28, PIPE=29, EQUAL=30, 
		PERCENT=31, DOUBLE_COLON=32, RIGHT_SHIFT=33, LEFT_SHIFT=34, AT=35, KW_SETRAISES=36, 
		KW_OUT=37, KW_EMITS=38, KW_STRING=39, KW_SWITCH=40, KW_PUBLISHES=41, KW_TYPEDEF=42, 
		KW_USES=43, KW_PRIMARYKEY=44, KW_CUSTOM=45, KW_OCTET=46, KW_SEQUENCE=47, 
		KW_IMPORT=48, KW_STRUCT=49, KW_NATIVE=50, KW_READONLY=51, KW_FINDER=52, 
		KW_RAISES=53, KW_VOID=54, KW_PRIVATE=55, KW_EVENTTYPE=56, KW_WCHAR=57, 
		KW_IN=58, KW_DEFAULT=59, KW_PUBLIC=60, KW_SHORT=61, KW_LONG=62, KW_ENUM=63, 
		KW_WSTRING=64, KW_CONTEXT=65, KW_HOME=66, KW_FACTORY=67, KW_EXCEPTION=68, 
		KW_GETRAISES=69, KW_CONST=70, KW_VALUEBASE=71, KW_VALUETYPE=72, KW_SUPPORTS=73, 
		KW_MODULE=74, KW_OBJECT=75, KW_TRUNCATABLE=76, KW_UNSIGNED=77, KW_FIXED=78, 
		KW_UNION=79, KW_ONEWAY=80, KW_ANY=81, KW_CHAR=82, KW_CASE=83, KW_FLOAT=84, 
		KW_BOOLEAN=85, KW_MULTIPLE=86, KW_ABSTRACT=87, KW_INOUT=88, KW_PROVIDES=89, 
		KW_CONSUMES=90, KW_DOUBLE=91, KW_TYPEPREFIX=92, KW_TYPEID=93, KW_ATTRIBUTE=94, 
		KW_LOCAL=95, KW_MANAGES=96, KW_INTERFACE=97, KW_COMPONENT=98, KW_SET=99, 
		KW_MAP=100, KW_BITFIELD=101, KW_BITSET=102, KW_BITMASK=103, KW_INT8=104, 
		KW_UINT8=105, KW_INT16=106, KW_UINT16=107, KW_INT32=108, KW_UINT32=109, 
		KW_INT64=110, KW_UINT64=111, KW_AT_ANNOTATION=112, ID=113, WS=114, COMMENT=115, 
		LINE_COMMENT=116;
	public static final int
		RULE_specification = 0, RULE_definition = 1, RULE_module = 2, RULE_interface_or_forward_decl = 3, 
		RULE_interface_decl = 4, RULE_forward_decl = 5, RULE_interface_header = 6, 
		RULE_interface_body = 7, RULE_export_ = 8, RULE_interface_inheritance_spec = 9, 
		RULE_interface_name = 10, RULE_a_scoped_name = 11, RULE_scoped_name = 12, 
		RULE_value = 13, RULE_value_forward_decl = 14, RULE_value_box_decl = 15, 
		RULE_value_abs_decl = 16, RULE_value_decl = 17, RULE_value_header = 18, 
		RULE_value_inheritance_spec = 19, RULE_value_name = 20, RULE_value_element = 21, 
		RULE_state_member = 22, RULE_init_decl = 23, RULE_init_param_decls = 24, 
		RULE_init_param_decl = 25, RULE_init_param_attribute = 26, RULE_const_decl = 27, 
		RULE_const_type = 28, RULE_const_exp = 29, RULE_or_expr = 30, RULE_xor_expr = 31, 
		RULE_and_expr = 32, RULE_shift_expr = 33, RULE_add_expr = 34, RULE_mult_expr = 35, 
		RULE_unary_expr = 36, RULE_unary_operator = 37, RULE_primary_expr = 38, 
		RULE_literal = 39, RULE_positive_int_const = 40, RULE_type_decl = 41, 
		RULE_type_declarator = 42, RULE_type_spec = 43, RULE_simple_type_spec = 44, 
		RULE_bitfield_type_spec = 45, RULE_base_type_spec = 46, RULE_template_type_spec = 47, 
		RULE_constr_type_spec = 48, RULE_simple_declarators = 49, RULE_declarators = 50, 
		RULE_declarator = 51, RULE_simple_declarator = 52, RULE_complex_declarator = 53, 
		RULE_floating_pt_type = 54, RULE_integer_type = 55, RULE_signed_int = 56, 
		RULE_signed_tiny_int = 57, RULE_signed_short_int = 58, RULE_signed_long_int = 59, 
		RULE_signed_longlong_int = 60, RULE_unsigned_int = 61, RULE_unsigned_tiny_int = 62, 
		RULE_unsigned_short_int = 63, RULE_unsigned_long_int = 64, RULE_unsigned_longlong_int = 65, 
		RULE_char_type = 66, RULE_wide_char_type = 67, RULE_boolean_type = 68, 
		RULE_octet_type = 69, RULE_any_type = 70, RULE_object_type = 71, RULE_annotation_decl = 72, 
		RULE_annotation_def = 73, RULE_annotation_header = 74, RULE_annotation_inheritance_spec = 75, 
		RULE_annotation_body = 76, RULE_annotation_member = 77, RULE_annotation_forward_dcl = 78, 
		RULE_bitset_type = 79, RULE_bitfield = 80, RULE_bitfield_spec = 81, RULE_bitmask_type = 82, 
		RULE_bit_values = 83, RULE_struct_type = 84, RULE_member_list = 85, RULE_member = 86, 
		RULE_union_type = 87, RULE_switch_type_spec = 88, RULE_switch_body = 89, 
		RULE_case_stmt = 90, RULE_case_label = 91, RULE_element_spec = 92, RULE_enum_type = 93, 
		RULE_enumerator = 94, RULE_sequence_type = 95, RULE_set_type = 96, RULE_map_type = 97, 
		RULE_string_type = 98, RULE_wide_string_type = 99, RULE_array_declarator = 100, 
		RULE_fixed_array_size = 101, RULE_attr_decl = 102, RULE_except_decl = 103, 
		RULE_op_decl = 104, RULE_op_attribute = 105, RULE_op_type_spec = 106, 
		RULE_parameter_decls = 107, RULE_param_decl = 108, RULE_param_attribute = 109, 
		RULE_raises_expr = 110, RULE_context_expr = 111, RULE_param_type_spec = 112, 
		RULE_fixed_pt_type = 113, RULE_fixed_pt_const_type = 114, RULE_value_base_type = 115, 
		RULE_constr_forward_decl = 116, RULE_import_decl = 117, RULE_imported_scope = 118, 
		RULE_type_id_decl = 119, RULE_type_prefix_decl = 120, RULE_readonly_attr_spec = 121, 
		RULE_readonly_attr_declarator = 122, RULE_attr_spec = 123, RULE_attr_declarator = 124, 
		RULE_attr_raises_expr = 125, RULE_get_excep_expr = 126, RULE_set_excep_expr = 127, 
		RULE_exception_list = 128, RULE_component = 129, RULE_component_forward_decl = 130, 
		RULE_component_decl = 131, RULE_component_header = 132, RULE_supported_interface_spec = 133, 
		RULE_component_inheritance_spec = 134, RULE_component_body = 135, RULE_component_export = 136, 
		RULE_provides_decl = 137, RULE_interface_type = 138, RULE_uses_decl = 139, 
		RULE_emits_decl = 140, RULE_publishes_decl = 141, RULE_consumes_decl = 142, 
		RULE_home_decl = 143, RULE_home_header = 144, RULE_home_inheritance_spec = 145, 
		RULE_primary_key_spec = 146, RULE_home_body = 147, RULE_home_export = 148, 
		RULE_factory_decl = 149, RULE_finder_decl = 150, RULE_event = 151, RULE_event_forward_decl = 152, 
		RULE_event_abs_decl = 153, RULE_event_decl = 154, RULE_event_header = 155, 
		RULE_annapps = 156, RULE_annotation_appl = 157, RULE_annotation_appl_params = 158, 
		RULE_annotation_appl_param = 159, RULE_identifier = 160;
	private static String[] makeRuleNames() {
		return new String[] {
			"specification", "definition", "module", "interface_or_forward_decl", 
			"interface_decl", "forward_decl", "interface_header", "interface_body", 
			"export_", "interface_inheritance_spec", "interface_name", "a_scoped_name", 
			"scoped_name", "value", "value_forward_decl", "value_box_decl", "value_abs_decl", 
			"value_decl", "value_header", "value_inheritance_spec", "value_name", 
			"value_element", "state_member", "init_decl", "init_param_decls", "init_param_decl", 
			"init_param_attribute", "const_decl", "const_type", "const_exp", "or_expr", 
			"xor_expr", "and_expr", "shift_expr", "add_expr", "mult_expr", "unary_expr", 
			"unary_operator", "primary_expr", "literal", "positive_int_const", "type_decl", 
			"type_declarator", "type_spec", "simple_type_spec", "bitfield_type_spec", 
			"base_type_spec", "template_type_spec", "constr_type_spec", "simple_declarators", 
			"declarators", "declarator", "simple_declarator", "complex_declarator", 
			"floating_pt_type", "integer_type", "signed_int", "signed_tiny_int", 
			"signed_short_int", "signed_long_int", "signed_longlong_int", "unsigned_int", 
			"unsigned_tiny_int", "unsigned_short_int", "unsigned_long_int", "unsigned_longlong_int", 
			"char_type", "wide_char_type", "boolean_type", "octet_type", "any_type", 
			"object_type", "annotation_decl", "annotation_def", "annotation_header", 
			"annotation_inheritance_spec", "annotation_body", "annotation_member", 
			"annotation_forward_dcl", "bitset_type", "bitfield", "bitfield_spec", 
			"bitmask_type", "bit_values", "struct_type", "member_list", "member", 
			"union_type", "switch_type_spec", "switch_body", "case_stmt", "case_label", 
			"element_spec", "enum_type", "enumerator", "sequence_type", "set_type", 
			"map_type", "string_type", "wide_string_type", "array_declarator", "fixed_array_size", 
			"attr_decl", "except_decl", "op_decl", "op_attribute", "op_type_spec", 
			"parameter_decls", "param_decl", "param_attribute", "raises_expr", "context_expr", 
			"param_type_spec", "fixed_pt_type", "fixed_pt_const_type", "value_base_type", 
			"constr_forward_decl", "import_decl", "imported_scope", "type_id_decl", 
			"type_prefix_decl", "readonly_attr_spec", "readonly_attr_declarator", 
			"attr_spec", "attr_declarator", "attr_raises_expr", "get_excep_expr", 
			"set_excep_expr", "exception_list", "component", "component_forward_decl", 
			"component_decl", "component_header", "supported_interface_spec", "component_inheritance_spec", 
			"component_body", "component_export", "provides_decl", "interface_type", 
			"uses_decl", "emits_decl", "publishes_decl", "consumes_decl", "home_decl", 
			"home_header", "home_inheritance_spec", "primary_key_spec", "home_body", 
			"home_export", "factory_decl", "finder_decl", "event", "event_forward_decl", 
			"event_abs_decl", "event_decl", "event_header", "annapps", "annotation_appl", 
			"annotation_appl_params", "annotation_appl_param", "identifier"
		};
	}
	public static final String[] ruleNames = makeRuleNames();

	private static String[] makeLiteralNames() {
		return new String[] {
			null, null, null, null, null, null, null, null, null, null, null, "';'", 
			"':'", "','", "'{'", "'}'", "'('", "')'", "'['", "']'", "'~'", "'/'", 
			"'<'", "'>'", "'*'", "'+'", "'-'", "'^'", "'&'", "'|'", "'='", "'%'", 
			"'::'", "'>>'", "'<<'", "'@'", "'setraises'", "'out'", "'emits'", "'string'", 
			"'switch'", "'publishes'", "'typedef'", "'uses'", "'primarykey'", "'custom'", 
			"'octet'", "'sequence'", "'import'", "'struct'", "'native'", "'readonly'", 
			"'finder'", "'raises'", "'void'", "'private'", "'eventtype'", "'wchar'", 
			"'in'", "'default'", "'public'", "'short'", "'long'", "'enum'", "'wstring'", 
			"'context'", "'home'", "'factory'", "'exception'", "'getraises'", "'const'", 
			"'ValueBase'", "'valuetype'", "'supports'", "'module'", "'Object'", "'truncatable'", 
			"'unsigned'", "'fixed'", "'union'", "'oneway'", "'any'", "'char'", "'case'", 
			"'float'", "'boolean'", "'multiple'", "'abstract'", "'inout'", "'provides'", 
			"'consumes'", "'double'", "'typeprefix'", "'typeid'", "'attribute'", 
			"'local'", "'manages'", "'interface'", "'component'", "'set'", "'map'", 
			"'bitfield'", "'bitset'", "'bitmask'", "'int8'", "'uint8'", "'int16'", 
			"'uint16'", "'int32'", "'uint32'", "'int64'", "'uint64'", "'@annotation'"
		};
	}
	private static final String[] _LITERAL_NAMES = makeLiteralNames();
	private static String[] makeSymbolicNames() {
		return new String[] {
			null, "INTEGER_LITERAL", "OCTAL_LITERAL", "HEX_LITERAL", "FLOATING_PT_LITERAL", 
			"FIXED_PT_LITERAL", "WIDE_CHARACTER_LITERAL", "CHARACTER_LITERAL", "WIDE_STRING_LITERAL", 
			"STRING_LITERAL", "BOOLEAN_LITERAL", "SEMICOLON", "COLON", "COMMA", "LEFT_BRACE", 
			"RIGHT_BRACE", "LEFT_BRACKET", "RIGHT_BRACKET", "LEFT_SQUARE_BRACKET", 
			"RIGHT_SQUARE_BRACKET", "TILDE", "SLASH", "LEFT_ANG_BRACKET", "RIGHT_ANG_BRACKET", 
			"STAR", "PLUS", "MINUS", "CARET", "AMPERSAND", "PIPE", "EQUAL", "PERCENT", 
			"DOUBLE_COLON", "RIGHT_SHIFT", "LEFT_SHIFT", "AT", "KW_SETRAISES", "KW_OUT", 
			"KW_EMITS", "KW_STRING", "KW_SWITCH", "KW_PUBLISHES", "KW_TYPEDEF", "KW_USES", 
			"KW_PRIMARYKEY", "KW_CUSTOM", "KW_OCTET", "KW_SEQUENCE", "KW_IMPORT", 
			"KW_STRUCT", "KW_NATIVE", "KW_READONLY", "KW_FINDER", "KW_RAISES", "KW_VOID", 
			"KW_PRIVATE", "KW_EVENTTYPE", "KW_WCHAR", "KW_IN", "KW_DEFAULT", "KW_PUBLIC", 
			"KW_SHORT", "KW_LONG", "KW_ENUM", "KW_WSTRING", "KW_CONTEXT", "KW_HOME", 
			"KW_FACTORY", "KW_EXCEPTION", "KW_GETRAISES", "KW_CONST", "KW_VALUEBASE", 
			"KW_VALUETYPE", "KW_SUPPORTS", "KW_MODULE", "KW_OBJECT", "KW_TRUNCATABLE", 
			"KW_UNSIGNED", "KW_FIXED", "KW_UNION", "KW_ONEWAY", "KW_ANY", "KW_CHAR", 
			"KW_CASE", "KW_FLOAT", "KW_BOOLEAN", "KW_MULTIPLE", "KW_ABSTRACT", "KW_INOUT", 
			"KW_PROVIDES", "KW_CONSUMES", "KW_DOUBLE", "KW_TYPEPREFIX", "KW_TYPEID", 
			"KW_ATTRIBUTE", "KW_LOCAL", "KW_MANAGES", "KW_INTERFACE", "KW_COMPONENT", 
			"KW_SET", "KW_MAP", "KW_BITFIELD", "KW_BITSET", "KW_BITMASK", "KW_INT8", 
			"KW_UINT8", "KW_INT16", "KW_UINT16", "KW_INT32", "KW_UINT32", "KW_INT64", 
			"KW_UINT64", "KW_AT_ANNOTATION", "ID", "WS", "COMMENT", "LINE_COMMENT"
		};
	}
	private static final String[] _SYMBOLIC_NAMES = makeSymbolicNames();
	public static final Vocabulary VOCABULARY = new VocabularyImpl(_LITERAL_NAMES, _SYMBOLIC_NAMES);

	/**
	 * @deprecated Use {@link #VOCABULARY} instead.
	 */
	@Deprecated
	public static final String[] tokenNames;
	static {
		tokenNames = new String[_SYMBOLIC_NAMES.length];
		for (int i = 0; i < tokenNames.length; i++) {
			tokenNames[i] = VOCABULARY.getLiteralName(i);
			if (tokenNames[i] == null) {
				tokenNames[i] = VOCABULARY.getSymbolicName(i);
			}

			if (tokenNames[i] == null) {
				tokenNames[i] = "<INVALID>";
			}
		}
	}

	@Override
	@Deprecated
	public String[] getTokenNames() {
		return tokenNames;
	}

	@Override

	public Vocabulary getVocabulary() {
		return VOCABULARY;
	}

	@Override
	public String getGrammarFileName() { return "IDL.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public IDLParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	@SuppressWarnings("CheckReturnValue")
	public static class SpecificationContext extends ParserRuleContext {
		public TerminalNode EOF() { return getToken(IDLParser.EOF, 0); }
		public List<Import_declContext> import_decl() {
			return getRuleContexts(Import_declContext.class);
		}
		public Import_declContext import_decl(int i) {
			return getRuleContext(Import_declContext.class,i);
		}
		public List<DefinitionContext> definition() {
			return getRuleContexts(DefinitionContext.class);
		}
		public DefinitionContext definition(int i) {
			return getRuleContext(DefinitionContext.class,i);
		}
		public SpecificationContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_specification; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSpecification(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSpecification(this);
		}
	}

	public final SpecificationContext specification() throws RecognitionException {
		SpecificationContext _localctx = new SpecificationContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_specification);
		int _la;
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(325);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,0,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(322);
					import_decl();
					}
					} 
				}
				setState(327);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,0,_ctx);
			}
			setState(329); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(328);
				definition();
				}
				}
				setState(331); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( (((_la) & ~0x3f) == 0 && ((1L << _la) & -9149585976178245632L) != 0) || ((((_la - 66)) & ~0x3f) == 0 && ((1L << (_la - 66)) & 70582085362005L) != 0) );
			setState(333);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class DefinitionContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Type_declContext type_decl() {
			return getRuleContext(Type_declContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public Const_declContext const_decl() {
			return getRuleContext(Const_declContext.class,0);
		}
		public Except_declContext except_decl() {
			return getRuleContext(Except_declContext.class,0);
		}
		public Interface_or_forward_declContext interface_or_forward_decl() {
			return getRuleContext(Interface_or_forward_declContext.class,0);
		}
		public ModuleContext module() {
			return getRuleContext(ModuleContext.class,0);
		}
		public ValueContext value() {
			return getRuleContext(ValueContext.class,0);
		}
		public Type_id_declContext type_id_decl() {
			return getRuleContext(Type_id_declContext.class,0);
		}
		public Type_prefix_declContext type_prefix_decl() {
			return getRuleContext(Type_prefix_declContext.class,0);
		}
		public EventContext event() {
			return getRuleContext(EventContext.class,0);
		}
		public ComponentContext component() {
			return getRuleContext(ComponentContext.class,0);
		}
		public Home_declContext home_decl() {
			return getRuleContext(Home_declContext.class,0);
		}
		public Annotation_declContext annotation_decl() {
			return getRuleContext(Annotation_declContext.class,0);
		}
		public DefinitionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_definition; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterDefinition(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitDefinition(this);
		}
	}

	public final DefinitionContext definition() throws RecognitionException {
		DefinitionContext _localctx = new DefinitionContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_definition);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(335);
			annapps();
			setState(372);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,2,_ctx) ) {
			case 1:
				{
				setState(336);
				type_decl();
				setState(337);
				match(SEMICOLON);
				}
				break;
			case 2:
				{
				setState(339);
				const_decl();
				setState(340);
				match(SEMICOLON);
				}
				break;
			case 3:
				{
				setState(342);
				except_decl();
				setState(343);
				match(SEMICOLON);
				}
				break;
			case 4:
				{
				setState(345);
				interface_or_forward_decl();
				setState(346);
				match(SEMICOLON);
				}
				break;
			case 5:
				{
				setState(348);
				module();
				setState(349);
				match(SEMICOLON);
				}
				break;
			case 6:
				{
				setState(351);
				value();
				setState(352);
				match(SEMICOLON);
				}
				break;
			case 7:
				{
				setState(354);
				type_id_decl();
				setState(355);
				match(SEMICOLON);
				}
				break;
			case 8:
				{
				setState(357);
				type_prefix_decl();
				setState(358);
				match(SEMICOLON);
				}
				break;
			case 9:
				{
				setState(360);
				event();
				setState(361);
				match(SEMICOLON);
				}
				break;
			case 10:
				{
				setState(363);
				component();
				setState(364);
				match(SEMICOLON);
				}
				break;
			case 11:
				{
				setState(366);
				home_decl();
				setState(367);
				match(SEMICOLON);
				}
				break;
			case 12:
				{
				setState(369);
				annotation_decl();
				setState(370);
				match(SEMICOLON);
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ModuleContext extends ParserRuleContext {
		public TerminalNode KW_MODULE() { return getToken(IDLParser.KW_MODULE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<DefinitionContext> definition() {
			return getRuleContexts(DefinitionContext.class);
		}
		public DefinitionContext definition(int i) {
			return getRuleContext(DefinitionContext.class,i);
		}
		public ModuleContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_module; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterModule(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitModule(this);
		}
	}

	public final ModuleContext module() throws RecognitionException {
		ModuleContext _localctx = new ModuleContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_module);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(374);
			match(KW_MODULE);
			setState(375);
			identifier();
			setState(376);
			match(LEFT_BRACE);
			setState(378); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(377);
				definition();
				}
				}
				setState(380); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( (((_la) & ~0x3f) == 0 && ((1L << _la) & -9149585976178245632L) != 0) || ((((_la - 66)) & ~0x3f) == 0 && ((1L << (_la - 66)) & 70582085362005L) != 0) );
			setState(382);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Interface_or_forward_declContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Interface_declContext interface_decl() {
			return getRuleContext(Interface_declContext.class,0);
		}
		public Forward_declContext forward_decl() {
			return getRuleContext(Forward_declContext.class,0);
		}
		public Interface_or_forward_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_interface_or_forward_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInterface_or_forward_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInterface_or_forward_decl(this);
		}
	}

	public final Interface_or_forward_declContext interface_or_forward_decl() throws RecognitionException {
		Interface_or_forward_declContext _localctx = new Interface_or_forward_declContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_interface_or_forward_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(384);
			annapps();
			setState(387);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,4,_ctx) ) {
			case 1:
				{
				setState(385);
				interface_decl();
				}
				break;
			case 2:
				{
				setState(386);
				forward_decl();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Interface_declContext extends ParserRuleContext {
		public Interface_headerContext interface_header() {
			return getRuleContext(Interface_headerContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public Interface_bodyContext interface_body() {
			return getRuleContext(Interface_bodyContext.class,0);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public Interface_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_interface_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInterface_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInterface_decl(this);
		}
	}

	public final Interface_declContext interface_decl() throws RecognitionException {
		Interface_declContext _localctx = new Interface_declContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_interface_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(389);
			interface_header();
			setState(390);
			match(LEFT_BRACE);
			setState(391);
			interface_body();
			setState(392);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Forward_declContext extends ParserRuleContext {
		public TerminalNode KW_INTERFACE() { return getToken(IDLParser.KW_INTERFACE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode KW_ABSTRACT() { return getToken(IDLParser.KW_ABSTRACT, 0); }
		public TerminalNode KW_LOCAL() { return getToken(IDLParser.KW_LOCAL, 0); }
		public Forward_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_forward_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterForward_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitForward_decl(this);
		}
	}

	public final Forward_declContext forward_decl() throws RecognitionException {
		Forward_declContext _localctx = new Forward_declContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_forward_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(395);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_ABSTRACT || _la==KW_LOCAL) {
				{
				setState(394);
				_la = _input.LA(1);
				if ( !(_la==KW_ABSTRACT || _la==KW_LOCAL) ) {
				_errHandler.recoverInline(this);
				}
				else {
					if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
					_errHandler.reportMatch(this);
					consume();
				}
				}
			}

			setState(397);
			match(KW_INTERFACE);
			setState(398);
			identifier();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Interface_headerContext extends ParserRuleContext {
		public TerminalNode KW_INTERFACE() { return getToken(IDLParser.KW_INTERFACE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Interface_inheritance_specContext interface_inheritance_spec() {
			return getRuleContext(Interface_inheritance_specContext.class,0);
		}
		public TerminalNode KW_ABSTRACT() { return getToken(IDLParser.KW_ABSTRACT, 0); }
		public TerminalNode KW_LOCAL() { return getToken(IDLParser.KW_LOCAL, 0); }
		public Interface_headerContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_interface_header; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInterface_header(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInterface_header(this);
		}
	}

	public final Interface_headerContext interface_header() throws RecognitionException {
		Interface_headerContext _localctx = new Interface_headerContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_interface_header);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(401);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_ABSTRACT || _la==KW_LOCAL) {
				{
				setState(400);
				_la = _input.LA(1);
				if ( !(_la==KW_ABSTRACT || _la==KW_LOCAL) ) {
				_errHandler.recoverInline(this);
				}
				else {
					if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
					_errHandler.reportMatch(this);
					consume();
				}
				}
			}

			setState(403);
			match(KW_INTERFACE);
			setState(404);
			identifier();
			setState(406);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COLON) {
				{
				setState(405);
				interface_inheritance_spec();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Interface_bodyContext extends ParserRuleContext {
		public List<Export_Context> export_() {
			return getRuleContexts(Export_Context.class);
		}
		public Export_Context export_(int i) {
			return getRuleContext(Export_Context.class,i);
		}
		public Interface_bodyContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_interface_body; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInterface_body(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInterface_body(this);
		}
	}

	public final Interface_bodyContext interface_body() throws RecognitionException {
		Interface_bodyContext _localctx = new Interface_bodyContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_interface_body);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(411);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -2139697417753198592L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844152069138641L) != 0)) {
				{
				{
				setState(408);
				export_();
				}
				}
				setState(413);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Export_Context extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Type_declContext type_decl() {
			return getRuleContext(Type_declContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public Const_declContext const_decl() {
			return getRuleContext(Const_declContext.class,0);
		}
		public Except_declContext except_decl() {
			return getRuleContext(Except_declContext.class,0);
		}
		public Attr_declContext attr_decl() {
			return getRuleContext(Attr_declContext.class,0);
		}
		public Op_declContext op_decl() {
			return getRuleContext(Op_declContext.class,0);
		}
		public Type_id_declContext type_id_decl() {
			return getRuleContext(Type_id_declContext.class,0);
		}
		public Type_prefix_declContext type_prefix_decl() {
			return getRuleContext(Type_prefix_declContext.class,0);
		}
		public Export_Context(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_export_; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterExport_(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitExport_(this);
		}
	}

	public final Export_Context export_() throws RecognitionException {
		Export_Context _localctx = new Export_Context(_ctx, getState());
		enterRule(_localctx, 16, RULE_export_);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(414);
			annapps();
			setState(436);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_TYPEDEF:
			case KW_STRUCT:
			case KW_NATIVE:
			case KW_ENUM:
			case KW_UNION:
			case KW_BITSET:
			case KW_BITMASK:
				{
				setState(415);
				type_decl();
				setState(416);
				match(SEMICOLON);
				}
				break;
			case KW_CONST:
				{
				setState(418);
				const_decl();
				setState(419);
				match(SEMICOLON);
				}
				break;
			case KW_EXCEPTION:
				{
				setState(421);
				except_decl();
				setState(422);
				match(SEMICOLON);
				}
				break;
			case KW_READONLY:
			case KW_ATTRIBUTE:
				{
				setState(424);
				attr_decl();
				setState(425);
				match(SEMICOLON);
				}
				break;
			case DOUBLE_COLON:
			case AT:
			case KW_STRING:
			case KW_OCTET:
			case KW_VOID:
			case KW_WCHAR:
			case KW_SHORT:
			case KW_LONG:
			case KW_WSTRING:
			case KW_VALUEBASE:
			case KW_OBJECT:
			case KW_UNSIGNED:
			case KW_ONEWAY:
			case KW_ANY:
			case KW_CHAR:
			case KW_FLOAT:
			case KW_BOOLEAN:
			case KW_DOUBLE:
			case KW_INT8:
			case KW_UINT8:
			case KW_INT16:
			case KW_UINT16:
			case KW_INT32:
			case KW_UINT32:
			case KW_INT64:
			case KW_UINT64:
			case ID:
				{
				setState(427);
				op_decl();
				setState(428);
				match(SEMICOLON);
				}
				break;
			case KW_TYPEID:
				{
				setState(430);
				type_id_decl();
				setState(431);
				match(SEMICOLON);
				}
				break;
			case KW_TYPEPREFIX:
				{
				setState(433);
				type_prefix_decl();
				setState(434);
				match(SEMICOLON);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Interface_inheritance_specContext extends ParserRuleContext {
		public TerminalNode COLON() { return getToken(IDLParser.COLON, 0); }
		public List<Interface_nameContext> interface_name() {
			return getRuleContexts(Interface_nameContext.class);
		}
		public Interface_nameContext interface_name(int i) {
			return getRuleContext(Interface_nameContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Interface_inheritance_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_interface_inheritance_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInterface_inheritance_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInterface_inheritance_spec(this);
		}
	}

	public final Interface_inheritance_specContext interface_inheritance_spec() throws RecognitionException {
		Interface_inheritance_specContext _localctx = new Interface_inheritance_specContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_interface_inheritance_spec);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(438);
			match(COLON);
			setState(439);
			interface_name();
			setState(444);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(440);
				match(COMMA);
				setState(441);
				interface_name();
				}
				}
				setState(446);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Interface_nameContext extends ParserRuleContext {
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public Interface_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_interface_name; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInterface_name(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInterface_name(this);
		}
	}

	public final Interface_nameContext interface_name() throws RecognitionException {
		Interface_nameContext _localctx = new Interface_nameContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_interface_name);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(447);
			a_scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class A_scoped_nameContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public A_scoped_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_a_scoped_name; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterA_scoped_name(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitA_scoped_name(this);
		}
	}

	public final A_scoped_nameContext a_scoped_name() throws RecognitionException {
		A_scoped_nameContext _localctx = new A_scoped_nameContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_a_scoped_name);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(449);
			annapps();
			setState(450);
			scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Scoped_nameContext extends ParserRuleContext {
		public List<TerminalNode> ID() { return getTokens(IDLParser.ID); }
		public TerminalNode ID(int i) {
			return getToken(IDLParser.ID, i);
		}
		public List<TerminalNode> DOUBLE_COLON() { return getTokens(IDLParser.DOUBLE_COLON); }
		public TerminalNode DOUBLE_COLON(int i) {
			return getToken(IDLParser.DOUBLE_COLON, i);
		}
		public Scoped_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_scoped_name; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterScoped_name(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitScoped_name(this);
		}
	}

	public final Scoped_nameContext scoped_name() throws RecognitionException {
		Scoped_nameContext _localctx = new Scoped_nameContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_scoped_name);
		int _la;
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(453);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==DOUBLE_COLON) {
				{
				setState(452);
				match(DOUBLE_COLON);
				}
			}

			setState(455);
			match(ID);
			setState(460);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,12,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(456);
					match(DOUBLE_COLON);
					setState(457);
					match(ID);
					}
					} 
				}
				setState(462);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,12,_ctx);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ValueContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Value_declContext value_decl() {
			return getRuleContext(Value_declContext.class,0);
		}
		public Value_abs_declContext value_abs_decl() {
			return getRuleContext(Value_abs_declContext.class,0);
		}
		public Value_box_declContext value_box_decl() {
			return getRuleContext(Value_box_declContext.class,0);
		}
		public Value_forward_declContext value_forward_decl() {
			return getRuleContext(Value_forward_declContext.class,0);
		}
		public ValueContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue(this);
		}
	}

	public final ValueContext value() throws RecognitionException {
		ValueContext _localctx = new ValueContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_value);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(463);
			annapps();
			setState(468);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,13,_ctx) ) {
			case 1:
				{
				setState(464);
				value_decl();
				}
				break;
			case 2:
				{
				setState(465);
				value_abs_decl();
				}
				break;
			case 3:
				{
				setState(466);
				value_box_decl();
				}
				break;
			case 4:
				{
				setState(467);
				value_forward_decl();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_forward_declContext extends ParserRuleContext {
		public TerminalNode KW_VALUETYPE() { return getToken(IDLParser.KW_VALUETYPE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode KW_ABSTRACT() { return getToken(IDLParser.KW_ABSTRACT, 0); }
		public Value_forward_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_forward_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_forward_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_forward_decl(this);
		}
	}

	public final Value_forward_declContext value_forward_decl() throws RecognitionException {
		Value_forward_declContext _localctx = new Value_forward_declContext(_ctx, getState());
		enterRule(_localctx, 28, RULE_value_forward_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(471);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_ABSTRACT) {
				{
				setState(470);
				match(KW_ABSTRACT);
				}
			}

			setState(473);
			match(KW_VALUETYPE);
			setState(474);
			identifier();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_box_declContext extends ParserRuleContext {
		public TerminalNode KW_VALUETYPE() { return getToken(IDLParser.KW_VALUETYPE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Type_specContext type_spec() {
			return getRuleContext(Type_specContext.class,0);
		}
		public Value_box_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_box_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_box_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_box_decl(this);
		}
	}

	public final Value_box_declContext value_box_decl() throws RecognitionException {
		Value_box_declContext _localctx = new Value_box_declContext(_ctx, getState());
		enterRule(_localctx, 30, RULE_value_box_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(476);
			match(KW_VALUETYPE);
			setState(477);
			identifier();
			setState(478);
			type_spec();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_abs_declContext extends ParserRuleContext {
		public TerminalNode KW_ABSTRACT() { return getToken(IDLParser.KW_ABSTRACT, 0); }
		public TerminalNode KW_VALUETYPE() { return getToken(IDLParser.KW_VALUETYPE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Value_inheritance_specContext value_inheritance_spec() {
			return getRuleContext(Value_inheritance_specContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<Export_Context> export_() {
			return getRuleContexts(Export_Context.class);
		}
		public Export_Context export_(int i) {
			return getRuleContext(Export_Context.class,i);
		}
		public Value_abs_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_abs_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_abs_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_abs_decl(this);
		}
	}

	public final Value_abs_declContext value_abs_decl() throws RecognitionException {
		Value_abs_declContext _localctx = new Value_abs_declContext(_ctx, getState());
		enterRule(_localctx, 32, RULE_value_abs_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(480);
			match(KW_ABSTRACT);
			setState(481);
			match(KW_VALUETYPE);
			setState(482);
			identifier();
			setState(483);
			value_inheritance_spec();
			setState(484);
			match(LEFT_BRACE);
			setState(488);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -2139697417753198592L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844152069138641L) != 0)) {
				{
				{
				setState(485);
				export_();
				}
				}
				setState(490);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(491);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_declContext extends ParserRuleContext {
		public Value_headerContext value_header() {
			return getRuleContext(Value_headerContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<Value_elementContext> value_element() {
			return getRuleContexts(Value_elementContext.class);
		}
		public Value_elementContext value_element(int i) {
			return getRuleContext(Value_elementContext.class,i);
		}
		public Value_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_decl(this);
		}
	}

	public final Value_declContext value_decl() throws RecognitionException {
		Value_declContext _localctx = new Value_declContext(_ctx, getState());
		enterRule(_localctx, 34, RULE_value_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(493);
			value_header();
			setState(494);
			match(LEFT_BRACE);
			setState(498);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -950747116127387648L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844152069138649L) != 0)) {
				{
				{
				setState(495);
				value_element();
				}
				}
				setState(500);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(501);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_headerContext extends ParserRuleContext {
		public TerminalNode KW_VALUETYPE() { return getToken(IDLParser.KW_VALUETYPE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Value_inheritance_specContext value_inheritance_spec() {
			return getRuleContext(Value_inheritance_specContext.class,0);
		}
		public TerminalNode KW_CUSTOM() { return getToken(IDLParser.KW_CUSTOM, 0); }
		public Value_headerContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_header; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_header(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_header(this);
		}
	}

	public final Value_headerContext value_header() throws RecognitionException {
		Value_headerContext _localctx = new Value_headerContext(_ctx, getState());
		enterRule(_localctx, 36, RULE_value_header);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(504);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_CUSTOM) {
				{
				setState(503);
				match(KW_CUSTOM);
				}
			}

			setState(506);
			match(KW_VALUETYPE);
			setState(507);
			identifier();
			setState(508);
			value_inheritance_spec();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_inheritance_specContext extends ParserRuleContext {
		public TerminalNode COLON() { return getToken(IDLParser.COLON, 0); }
		public List<Value_nameContext> value_name() {
			return getRuleContexts(Value_nameContext.class);
		}
		public Value_nameContext value_name(int i) {
			return getRuleContext(Value_nameContext.class,i);
		}
		public TerminalNode KW_SUPPORTS() { return getToken(IDLParser.KW_SUPPORTS, 0); }
		public List<Interface_nameContext> interface_name() {
			return getRuleContexts(Interface_nameContext.class);
		}
		public Interface_nameContext interface_name(int i) {
			return getRuleContext(Interface_nameContext.class,i);
		}
		public TerminalNode KW_TRUNCATABLE() { return getToken(IDLParser.KW_TRUNCATABLE, 0); }
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Value_inheritance_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_inheritance_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_inheritance_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_inheritance_spec(this);
		}
	}

	public final Value_inheritance_specContext value_inheritance_spec() throws RecognitionException {
		Value_inheritance_specContext _localctx = new Value_inheritance_specContext(_ctx, getState());
		enterRule(_localctx, 38, RULE_value_inheritance_spec);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(522);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COLON) {
				{
				setState(510);
				match(COLON);
				setState(512);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==KW_TRUNCATABLE) {
					{
					setState(511);
					match(KW_TRUNCATABLE);
					}
				}

				setState(514);
				value_name();
				setState(519);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==COMMA) {
					{
					{
					setState(515);
					match(COMMA);
					setState(516);
					value_name();
					}
					}
					setState(521);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
			}

			setState(533);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_SUPPORTS) {
				{
				setState(524);
				match(KW_SUPPORTS);
				setState(525);
				interface_name();
				setState(530);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==COMMA) {
					{
					{
					setState(526);
					match(COMMA);
					setState(527);
					interface_name();
					}
					}
					setState(532);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_nameContext extends ParserRuleContext {
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public Value_nameContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_name; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_name(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_name(this);
		}
	}

	public final Value_nameContext value_name() throws RecognitionException {
		Value_nameContext _localctx = new Value_nameContext(_ctx, getState());
		enterRule(_localctx, 40, RULE_value_name);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(535);
			a_scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_elementContext extends ParserRuleContext {
		public Export_Context export_() {
			return getRuleContext(Export_Context.class,0);
		}
		public State_memberContext state_member() {
			return getRuleContext(State_memberContext.class,0);
		}
		public Init_declContext init_decl() {
			return getRuleContext(Init_declContext.class,0);
		}
		public Value_elementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_element; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_element(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_element(this);
		}
	}

	public final Value_elementContext value_element() throws RecognitionException {
		Value_elementContext _localctx = new Value_elementContext(_ctx, getState());
		enterRule(_localctx, 42, RULE_value_element);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(540);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,23,_ctx) ) {
			case 1:
				{
				setState(537);
				export_();
				}
				break;
			case 2:
				{
				setState(538);
				state_member();
				}
				break;
			case 3:
				{
				setState(539);
				init_decl();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class State_memberContext extends ParserRuleContext {
		public List<AnnappsContext> annapps() {
			return getRuleContexts(AnnappsContext.class);
		}
		public AnnappsContext annapps(int i) {
			return getRuleContext(AnnappsContext.class,i);
		}
		public Type_specContext type_spec() {
			return getRuleContext(Type_specContext.class,0);
		}
		public DeclaratorsContext declarators() {
			return getRuleContext(DeclaratorsContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public TerminalNode KW_PUBLIC() { return getToken(IDLParser.KW_PUBLIC, 0); }
		public TerminalNode KW_PRIVATE() { return getToken(IDLParser.KW_PRIVATE, 0); }
		public State_memberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_state_member; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterState_member(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitState_member(this);
		}
	}

	public final State_memberContext state_member() throws RecognitionException {
		State_memberContext _localctx = new State_memberContext(_ctx, getState());
		enterRule(_localctx, 44, RULE_state_member);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(542);
			annapps();
			setState(547);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_PUBLIC:
				{
				setState(543);
				match(KW_PUBLIC);
				setState(544);
				annapps();
				}
				break;
			case KW_PRIVATE:
				{
				setState(545);
				match(KW_PRIVATE);
				setState(546);
				annapps();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(549);
			type_spec();
			setState(550);
			declarators();
			setState(551);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Init_declContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public TerminalNode KW_FACTORY() { return getToken(IDLParser.KW_FACTORY, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public Init_param_declsContext init_param_decls() {
			return getRuleContext(Init_param_declsContext.class,0);
		}
		public Raises_exprContext raises_expr() {
			return getRuleContext(Raises_exprContext.class,0);
		}
		public Init_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_init_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInit_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInit_decl(this);
		}
	}

	public final Init_declContext init_decl() throws RecognitionException {
		Init_declContext _localctx = new Init_declContext(_ctx, getState());
		enterRule(_localctx, 46, RULE_init_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(553);
			annapps();
			setState(554);
			match(KW_FACTORY);
			setState(555);
			identifier();
			setState(556);
			match(LEFT_BRACKET);
			setState(558);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==AT || _la==KW_IN) {
				{
				setState(557);
				init_param_decls();
				}
			}

			setState(560);
			match(RIGHT_BRACKET);
			setState(562);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_RAISES) {
				{
				setState(561);
				raises_expr();
				}
			}

			setState(564);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Init_param_declsContext extends ParserRuleContext {
		public List<Init_param_declContext> init_param_decl() {
			return getRuleContexts(Init_param_declContext.class);
		}
		public Init_param_declContext init_param_decl(int i) {
			return getRuleContext(Init_param_declContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Init_param_declsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_init_param_decls; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInit_param_decls(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInit_param_decls(this);
		}
	}

	public final Init_param_declsContext init_param_decls() throws RecognitionException {
		Init_param_declsContext _localctx = new Init_param_declsContext(_ctx, getState());
		enterRule(_localctx, 48, RULE_init_param_decls);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(566);
			init_param_decl();
			setState(571);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(567);
				match(COMMA);
				setState(568);
				init_param_decl();
				}
				}
				setState(573);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Init_param_declContext extends ParserRuleContext {
		public List<AnnappsContext> annapps() {
			return getRuleContexts(AnnappsContext.class);
		}
		public AnnappsContext annapps(int i) {
			return getRuleContext(AnnappsContext.class,i);
		}
		public Init_param_attributeContext init_param_attribute() {
			return getRuleContext(Init_param_attributeContext.class,0);
		}
		public Param_type_specContext param_type_spec() {
			return getRuleContext(Param_type_specContext.class,0);
		}
		public Simple_declaratorContext simple_declarator() {
			return getRuleContext(Simple_declaratorContext.class,0);
		}
		public Init_param_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_init_param_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInit_param_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInit_param_decl(this);
		}
	}

	public final Init_param_declContext init_param_decl() throws RecognitionException {
		Init_param_declContext _localctx = new Init_param_declContext(_ctx, getState());
		enterRule(_localctx, 50, RULE_init_param_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(574);
			annapps();
			setState(575);
			init_param_attribute();
			setState(576);
			annapps();
			setState(577);
			param_type_spec();
			setState(578);
			annapps();
			setState(579);
			simple_declarator();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Init_param_attributeContext extends ParserRuleContext {
		public TerminalNode KW_IN() { return getToken(IDLParser.KW_IN, 0); }
		public Init_param_attributeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_init_param_attribute; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInit_param_attribute(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInit_param_attribute(this);
		}
	}

	public final Init_param_attributeContext init_param_attribute() throws RecognitionException {
		Init_param_attributeContext _localctx = new Init_param_attributeContext(_ctx, getState());
		enterRule(_localctx, 52, RULE_init_param_attribute);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(581);
			match(KW_IN);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Const_declContext extends ParserRuleContext {
		public TerminalNode KW_CONST() { return getToken(IDLParser.KW_CONST, 0); }
		public Const_typeContext const_type() {
			return getRuleContext(Const_typeContext.class,0);
		}
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode EQUAL() { return getToken(IDLParser.EQUAL, 0); }
		public Const_expContext const_exp() {
			return getRuleContext(Const_expContext.class,0);
		}
		public Const_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_const_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterConst_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitConst_decl(this);
		}
	}

	public final Const_declContext const_decl() throws RecognitionException {
		Const_declContext _localctx = new Const_declContext(_ctx, getState());
		enterRule(_localctx, 54, RULE_const_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(583);
			match(KW_CONST);
			setState(584);
			const_type();
			setState(585);
			identifier();
			setState(586);
			match(EQUAL);
			setState(587);
			const_exp();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Const_typeContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Integer_typeContext integer_type() {
			return getRuleContext(Integer_typeContext.class,0);
		}
		public Char_typeContext char_type() {
			return getRuleContext(Char_typeContext.class,0);
		}
		public Wide_char_typeContext wide_char_type() {
			return getRuleContext(Wide_char_typeContext.class,0);
		}
		public Boolean_typeContext boolean_type() {
			return getRuleContext(Boolean_typeContext.class,0);
		}
		public Floating_pt_typeContext floating_pt_type() {
			return getRuleContext(Floating_pt_typeContext.class,0);
		}
		public String_typeContext string_type() {
			return getRuleContext(String_typeContext.class,0);
		}
		public Wide_string_typeContext wide_string_type() {
			return getRuleContext(Wide_string_typeContext.class,0);
		}
		public Fixed_pt_const_typeContext fixed_pt_const_type() {
			return getRuleContext(Fixed_pt_const_typeContext.class,0);
		}
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Octet_typeContext octet_type() {
			return getRuleContext(Octet_typeContext.class,0);
		}
		public Const_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_const_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterConst_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitConst_type(this);
		}
	}

	public final Const_typeContext const_type() throws RecognitionException {
		Const_typeContext _localctx = new Const_typeContext(_ctx, getState());
		enterRule(_localctx, 56, RULE_const_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(589);
			annapps();
			setState(600);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,28,_ctx) ) {
			case 1:
				{
				setState(590);
				integer_type();
				}
				break;
			case 2:
				{
				setState(591);
				char_type();
				}
				break;
			case 3:
				{
				setState(592);
				wide_char_type();
				}
				break;
			case 4:
				{
				setState(593);
				boolean_type();
				}
				break;
			case 5:
				{
				setState(594);
				floating_pt_type();
				}
				break;
			case 6:
				{
				setState(595);
				string_type();
				}
				break;
			case 7:
				{
				setState(596);
				wide_string_type();
				}
				break;
			case 8:
				{
				setState(597);
				fixed_pt_const_type();
				}
				break;
			case 9:
				{
				setState(598);
				scoped_name();
				}
				break;
			case 10:
				{
				setState(599);
				octet_type();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Const_expContext extends ParserRuleContext {
		public Or_exprContext or_expr() {
			return getRuleContext(Or_exprContext.class,0);
		}
		public Const_expContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_const_exp; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterConst_exp(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitConst_exp(this);
		}
	}

	public final Const_expContext const_exp() throws RecognitionException {
		Const_expContext _localctx = new Const_expContext(_ctx, getState());
		enterRule(_localctx, 58, RULE_const_exp);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(602);
			or_expr();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Or_exprContext extends ParserRuleContext {
		public List<Xor_exprContext> xor_expr() {
			return getRuleContexts(Xor_exprContext.class);
		}
		public Xor_exprContext xor_expr(int i) {
			return getRuleContext(Xor_exprContext.class,i);
		}
		public List<TerminalNode> PIPE() { return getTokens(IDLParser.PIPE); }
		public TerminalNode PIPE(int i) {
			return getToken(IDLParser.PIPE, i);
		}
		public Or_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_or_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterOr_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitOr_expr(this);
		}
	}

	public final Or_exprContext or_expr() throws RecognitionException {
		Or_exprContext _localctx = new Or_exprContext(_ctx, getState());
		enterRule(_localctx, 60, RULE_or_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(604);
			xor_expr();
			setState(609);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==PIPE) {
				{
				{
				setState(605);
				match(PIPE);
				setState(606);
				xor_expr();
				}
				}
				setState(611);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Xor_exprContext extends ParserRuleContext {
		public List<And_exprContext> and_expr() {
			return getRuleContexts(And_exprContext.class);
		}
		public And_exprContext and_expr(int i) {
			return getRuleContext(And_exprContext.class,i);
		}
		public List<TerminalNode> CARET() { return getTokens(IDLParser.CARET); }
		public TerminalNode CARET(int i) {
			return getToken(IDLParser.CARET, i);
		}
		public Xor_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_xor_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterXor_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitXor_expr(this);
		}
	}

	public final Xor_exprContext xor_expr() throws RecognitionException {
		Xor_exprContext _localctx = new Xor_exprContext(_ctx, getState());
		enterRule(_localctx, 62, RULE_xor_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(612);
			and_expr();
			setState(617);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==CARET) {
				{
				{
				setState(613);
				match(CARET);
				setState(614);
				and_expr();
				}
				}
				setState(619);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class And_exprContext extends ParserRuleContext {
		public List<Shift_exprContext> shift_expr() {
			return getRuleContexts(Shift_exprContext.class);
		}
		public Shift_exprContext shift_expr(int i) {
			return getRuleContext(Shift_exprContext.class,i);
		}
		public List<TerminalNode> AMPERSAND() { return getTokens(IDLParser.AMPERSAND); }
		public TerminalNode AMPERSAND(int i) {
			return getToken(IDLParser.AMPERSAND, i);
		}
		public And_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_and_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnd_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnd_expr(this);
		}
	}

	public final And_exprContext and_expr() throws RecognitionException {
		And_exprContext _localctx = new And_exprContext(_ctx, getState());
		enterRule(_localctx, 64, RULE_and_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(620);
			shift_expr();
			setState(625);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==AMPERSAND) {
				{
				{
				setState(621);
				match(AMPERSAND);
				setState(622);
				shift_expr();
				}
				}
				setState(627);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Shift_exprContext extends ParserRuleContext {
		public List<Add_exprContext> add_expr() {
			return getRuleContexts(Add_exprContext.class);
		}
		public Add_exprContext add_expr(int i) {
			return getRuleContext(Add_exprContext.class,i);
		}
		public List<TerminalNode> RIGHT_SHIFT() { return getTokens(IDLParser.RIGHT_SHIFT); }
		public TerminalNode RIGHT_SHIFT(int i) {
			return getToken(IDLParser.RIGHT_SHIFT, i);
		}
		public List<TerminalNode> LEFT_SHIFT() { return getTokens(IDLParser.LEFT_SHIFT); }
		public TerminalNode LEFT_SHIFT(int i) {
			return getToken(IDLParser.LEFT_SHIFT, i);
		}
		public Shift_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_shift_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterShift_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitShift_expr(this);
		}
	}

	public final Shift_exprContext shift_expr() throws RecognitionException {
		Shift_exprContext _localctx = new Shift_exprContext(_ctx, getState());
		enterRule(_localctx, 66, RULE_shift_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(628);
			add_expr();
			setState(633);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==RIGHT_SHIFT || _la==LEFT_SHIFT) {
				{
				{
				setState(629);
				_la = _input.LA(1);
				if ( !(_la==RIGHT_SHIFT || _la==LEFT_SHIFT) ) {
				_errHandler.recoverInline(this);
				}
				else {
					if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
					_errHandler.reportMatch(this);
					consume();
				}
				setState(630);
				add_expr();
				}
				}
				setState(635);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Add_exprContext extends ParserRuleContext {
		public List<Mult_exprContext> mult_expr() {
			return getRuleContexts(Mult_exprContext.class);
		}
		public Mult_exprContext mult_expr(int i) {
			return getRuleContext(Mult_exprContext.class,i);
		}
		public List<TerminalNode> PLUS() { return getTokens(IDLParser.PLUS); }
		public TerminalNode PLUS(int i) {
			return getToken(IDLParser.PLUS, i);
		}
		public List<TerminalNode> MINUS() { return getTokens(IDLParser.MINUS); }
		public TerminalNode MINUS(int i) {
			return getToken(IDLParser.MINUS, i);
		}
		public Add_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_add_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAdd_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAdd_expr(this);
		}
	}

	public final Add_exprContext add_expr() throws RecognitionException {
		Add_exprContext _localctx = new Add_exprContext(_ctx, getState());
		enterRule(_localctx, 68, RULE_add_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(636);
			mult_expr();
			setState(641);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==PLUS || _la==MINUS) {
				{
				{
				setState(637);
				_la = _input.LA(1);
				if ( !(_la==PLUS || _la==MINUS) ) {
				_errHandler.recoverInline(this);
				}
				else {
					if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
					_errHandler.reportMatch(this);
					consume();
				}
				setState(638);
				mult_expr();
				}
				}
				setState(643);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Mult_exprContext extends ParserRuleContext {
		public List<Unary_exprContext> unary_expr() {
			return getRuleContexts(Unary_exprContext.class);
		}
		public Unary_exprContext unary_expr(int i) {
			return getRuleContext(Unary_exprContext.class,i);
		}
		public List<TerminalNode> STAR() { return getTokens(IDLParser.STAR); }
		public TerminalNode STAR(int i) {
			return getToken(IDLParser.STAR, i);
		}
		public List<TerminalNode> SLASH() { return getTokens(IDLParser.SLASH); }
		public TerminalNode SLASH(int i) {
			return getToken(IDLParser.SLASH, i);
		}
		public List<TerminalNode> PERCENT() { return getTokens(IDLParser.PERCENT); }
		public TerminalNode PERCENT(int i) {
			return getToken(IDLParser.PERCENT, i);
		}
		public Mult_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_mult_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterMult_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitMult_expr(this);
		}
	}

	public final Mult_exprContext mult_expr() throws RecognitionException {
		Mult_exprContext _localctx = new Mult_exprContext(_ctx, getState());
		enterRule(_localctx, 70, RULE_mult_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(644);
			unary_expr();
			setState(649);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & 2166358016L) != 0)) {
				{
				{
				setState(645);
				_la = _input.LA(1);
				if ( !((((_la) & ~0x3f) == 0 && ((1L << _la) & 2166358016L) != 0)) ) {
				_errHandler.recoverInline(this);
				}
				else {
					if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
					_errHandler.reportMatch(this);
					consume();
				}
				setState(646);
				unary_expr();
				}
				}
				setState(651);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Unary_exprContext extends ParserRuleContext {
		public Unary_operatorContext unary_operator() {
			return getRuleContext(Unary_operatorContext.class,0);
		}
		public Primary_exprContext primary_expr() {
			return getRuleContext(Primary_exprContext.class,0);
		}
		public Unary_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unary_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnary_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnary_expr(this);
		}
	}

	public final Unary_exprContext unary_expr() throws RecognitionException {
		Unary_exprContext _localctx = new Unary_exprContext(_ctx, getState());
		enterRule(_localctx, 72, RULE_unary_expr);
		try {
			setState(656);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case TILDE:
			case PLUS:
			case MINUS:
				enterOuterAlt(_localctx, 1);
				{
				setState(652);
				unary_operator();
				setState(653);
				primary_expr();
				}
				break;
			case INTEGER_LITERAL:
			case OCTAL_LITERAL:
			case HEX_LITERAL:
			case FLOATING_PT_LITERAL:
			case FIXED_PT_LITERAL:
			case WIDE_CHARACTER_LITERAL:
			case CHARACTER_LITERAL:
			case WIDE_STRING_LITERAL:
			case STRING_LITERAL:
			case BOOLEAN_LITERAL:
			case LEFT_BRACKET:
			case DOUBLE_COLON:
			case ID:
				enterOuterAlt(_localctx, 2);
				{
				setState(655);
				primary_expr();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Unary_operatorContext extends ParserRuleContext {
		public TerminalNode MINUS() { return getToken(IDLParser.MINUS, 0); }
		public TerminalNode PLUS() { return getToken(IDLParser.PLUS, 0); }
		public TerminalNode TILDE() { return getToken(IDLParser.TILDE, 0); }
		public Unary_operatorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unary_operator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnary_operator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnary_operator(this);
		}
	}

	public final Unary_operatorContext unary_operator() throws RecognitionException {
		Unary_operatorContext _localctx = new Unary_operatorContext(_ctx, getState());
		enterRule(_localctx, 74, RULE_unary_operator);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(658);
			_la = _input.LA(1);
			if ( !((((_la) & ~0x3f) == 0 && ((1L << _la) & 101711872L) != 0)) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Primary_exprContext extends ParserRuleContext {
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public LiteralContext literal() {
			return getRuleContext(LiteralContext.class,0);
		}
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public Const_expContext const_exp() {
			return getRuleContext(Const_expContext.class,0);
		}
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public Primary_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_primary_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterPrimary_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitPrimary_expr(this);
		}
	}

	public final Primary_exprContext primary_expr() throws RecognitionException {
		Primary_exprContext _localctx = new Primary_exprContext(_ctx, getState());
		enterRule(_localctx, 76, RULE_primary_expr);
		try {
			setState(666);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case DOUBLE_COLON:
			case ID:
				enterOuterAlt(_localctx, 1);
				{
				setState(660);
				scoped_name();
				}
				break;
			case INTEGER_LITERAL:
			case OCTAL_LITERAL:
			case HEX_LITERAL:
			case FLOATING_PT_LITERAL:
			case FIXED_PT_LITERAL:
			case WIDE_CHARACTER_LITERAL:
			case CHARACTER_LITERAL:
			case WIDE_STRING_LITERAL:
			case STRING_LITERAL:
			case BOOLEAN_LITERAL:
				enterOuterAlt(_localctx, 2);
				{
				setState(661);
				literal();
				}
				break;
			case LEFT_BRACKET:
				enterOuterAlt(_localctx, 3);
				{
				setState(662);
				match(LEFT_BRACKET);
				setState(663);
				const_exp();
				setState(664);
				match(RIGHT_BRACKET);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class LiteralContext extends ParserRuleContext {
		public TerminalNode HEX_LITERAL() { return getToken(IDLParser.HEX_LITERAL, 0); }
		public TerminalNode INTEGER_LITERAL() { return getToken(IDLParser.INTEGER_LITERAL, 0); }
		public TerminalNode OCTAL_LITERAL() { return getToken(IDLParser.OCTAL_LITERAL, 0); }
		public TerminalNode STRING_LITERAL() { return getToken(IDLParser.STRING_LITERAL, 0); }
		public TerminalNode WIDE_STRING_LITERAL() { return getToken(IDLParser.WIDE_STRING_LITERAL, 0); }
		public TerminalNode CHARACTER_LITERAL() { return getToken(IDLParser.CHARACTER_LITERAL, 0); }
		public TerminalNode WIDE_CHARACTER_LITERAL() { return getToken(IDLParser.WIDE_CHARACTER_LITERAL, 0); }
		public TerminalNode FIXED_PT_LITERAL() { return getToken(IDLParser.FIXED_PT_LITERAL, 0); }
		public TerminalNode FLOATING_PT_LITERAL() { return getToken(IDLParser.FLOATING_PT_LITERAL, 0); }
		public TerminalNode BOOLEAN_LITERAL() { return getToken(IDLParser.BOOLEAN_LITERAL, 0); }
		public LiteralContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_literal; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterLiteral(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitLiteral(this);
		}
	}

	public final LiteralContext literal() throws RecognitionException {
		LiteralContext _localctx = new LiteralContext(_ctx, getState());
		enterRule(_localctx, 78, RULE_literal);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(668);
			_la = _input.LA(1);
			if ( !((((_la) & ~0x3f) == 0 && ((1L << _la) & 2046L) != 0)) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Positive_int_constContext extends ParserRuleContext {
		public Const_expContext const_exp() {
			return getRuleContext(Const_expContext.class,0);
		}
		public Positive_int_constContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_positive_int_const; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterPositive_int_const(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitPositive_int_const(this);
		}
	}

	public final Positive_int_constContext positive_int_const() throws RecognitionException {
		Positive_int_constContext _localctx = new Positive_int_constContext(_ctx, getState());
		enterRule(_localctx, 80, RULE_positive_int_const);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(670);
			const_exp();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Type_declContext extends ParserRuleContext {
		public TerminalNode KW_TYPEDEF() { return getToken(IDLParser.KW_TYPEDEF, 0); }
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Type_declaratorContext type_declarator() {
			return getRuleContext(Type_declaratorContext.class,0);
		}
		public Struct_typeContext struct_type() {
			return getRuleContext(Struct_typeContext.class,0);
		}
		public Union_typeContext union_type() {
			return getRuleContext(Union_typeContext.class,0);
		}
		public Enum_typeContext enum_type() {
			return getRuleContext(Enum_typeContext.class,0);
		}
		public Bitset_typeContext bitset_type() {
			return getRuleContext(Bitset_typeContext.class,0);
		}
		public Bitmask_typeContext bitmask_type() {
			return getRuleContext(Bitmask_typeContext.class,0);
		}
		public TerminalNode KW_NATIVE() { return getToken(IDLParser.KW_NATIVE, 0); }
		public Simple_declaratorContext simple_declarator() {
			return getRuleContext(Simple_declaratorContext.class,0);
		}
		public Constr_forward_declContext constr_forward_decl() {
			return getRuleContext(Constr_forward_declContext.class,0);
		}
		public Type_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterType_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitType_decl(this);
		}
	}

	public final Type_declContext type_decl() throws RecognitionException {
		Type_declContext _localctx = new Type_declContext(_ctx, getState());
		enterRule(_localctx, 82, RULE_type_decl);
		try {
			setState(686);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,37,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(672);
				match(KW_TYPEDEF);
				setState(673);
				annapps();
				setState(674);
				type_declarator();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(676);
				struct_type();
				}
				break;
			case 3:
				enterOuterAlt(_localctx, 3);
				{
				setState(677);
				union_type();
				}
				break;
			case 4:
				enterOuterAlt(_localctx, 4);
				{
				setState(678);
				enum_type();
				}
				break;
			case 5:
				enterOuterAlt(_localctx, 5);
				{
				setState(679);
				bitset_type();
				}
				break;
			case 6:
				enterOuterAlt(_localctx, 6);
				{
				setState(680);
				bitmask_type();
				}
				break;
			case 7:
				enterOuterAlt(_localctx, 7);
				{
				setState(681);
				match(KW_NATIVE);
				setState(682);
				annapps();
				setState(683);
				simple_declarator();
				}
				break;
			case 8:
				enterOuterAlt(_localctx, 8);
				{
				setState(685);
				constr_forward_decl();
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Type_declaratorContext extends ParserRuleContext {
		public Type_specContext type_spec() {
			return getRuleContext(Type_specContext.class,0);
		}
		public DeclaratorsContext declarators() {
			return getRuleContext(DeclaratorsContext.class,0);
		}
		public Type_declaratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type_declarator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterType_declarator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitType_declarator(this);
		}
	}

	public final Type_declaratorContext type_declarator() throws RecognitionException {
		Type_declaratorContext _localctx = new Type_declaratorContext(_ctx, getState());
		enterRule(_localctx, 84, RULE_type_declarator);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(688);
			type_spec();
			setState(689);
			declarators();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Type_specContext extends ParserRuleContext {
		public Simple_type_specContext simple_type_spec() {
			return getRuleContext(Simple_type_specContext.class,0);
		}
		public Constr_type_specContext constr_type_spec() {
			return getRuleContext(Constr_type_specContext.class,0);
		}
		public Type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterType_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitType_spec(this);
		}
	}

	public final Type_specContext type_spec() throws RecognitionException {
		Type_specContext _localctx = new Type_specContext(_ctx, getState());
		enterRule(_localctx, 86, RULE_type_spec);
		try {
			setState(693);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case DOUBLE_COLON:
			case KW_STRING:
			case KW_OCTET:
			case KW_SEQUENCE:
			case KW_WCHAR:
			case KW_SHORT:
			case KW_LONG:
			case KW_WSTRING:
			case KW_VALUEBASE:
			case KW_OBJECT:
			case KW_UNSIGNED:
			case KW_FIXED:
			case KW_ANY:
			case KW_CHAR:
			case KW_FLOAT:
			case KW_BOOLEAN:
			case KW_DOUBLE:
			case KW_SET:
			case KW_MAP:
			case KW_INT8:
			case KW_UINT8:
			case KW_INT16:
			case KW_UINT16:
			case KW_INT32:
			case KW_UINT32:
			case KW_INT64:
			case KW_UINT64:
			case ID:
				enterOuterAlt(_localctx, 1);
				{
				setState(691);
				simple_type_spec();
				}
				break;
			case KW_STRUCT:
			case KW_ENUM:
			case KW_UNION:
			case KW_BITSET:
			case KW_BITMASK:
				enterOuterAlt(_localctx, 2);
				{
				setState(692);
				constr_type_spec();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Simple_type_specContext extends ParserRuleContext {
		public Base_type_specContext base_type_spec() {
			return getRuleContext(Base_type_specContext.class,0);
		}
		public Template_type_specContext template_type_spec() {
			return getRuleContext(Template_type_specContext.class,0);
		}
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Simple_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_simple_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSimple_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSimple_type_spec(this);
		}
	}

	public final Simple_type_specContext simple_type_spec() throws RecognitionException {
		Simple_type_specContext _localctx = new Simple_type_specContext(_ctx, getState());
		enterRule(_localctx, 88, RULE_simple_type_spec);
		try {
			setState(698);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_OCTET:
			case KW_WCHAR:
			case KW_SHORT:
			case KW_LONG:
			case KW_VALUEBASE:
			case KW_OBJECT:
			case KW_UNSIGNED:
			case KW_ANY:
			case KW_CHAR:
			case KW_FLOAT:
			case KW_BOOLEAN:
			case KW_DOUBLE:
			case KW_INT8:
			case KW_UINT8:
			case KW_INT16:
			case KW_UINT16:
			case KW_INT32:
			case KW_UINT32:
			case KW_INT64:
			case KW_UINT64:
				enterOuterAlt(_localctx, 1);
				{
				setState(695);
				base_type_spec();
				}
				break;
			case KW_STRING:
			case KW_SEQUENCE:
			case KW_WSTRING:
			case KW_FIXED:
			case KW_SET:
			case KW_MAP:
				enterOuterAlt(_localctx, 2);
				{
				setState(696);
				template_type_spec();
				}
				break;
			case DOUBLE_COLON:
			case ID:
				enterOuterAlt(_localctx, 3);
				{
				setState(697);
				scoped_name();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Bitfield_type_specContext extends ParserRuleContext {
		public Integer_typeContext integer_type() {
			return getRuleContext(Integer_typeContext.class,0);
		}
		public Boolean_typeContext boolean_type() {
			return getRuleContext(Boolean_typeContext.class,0);
		}
		public Octet_typeContext octet_type() {
			return getRuleContext(Octet_typeContext.class,0);
		}
		public Bitfield_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bitfield_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBitfield_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBitfield_type_spec(this);
		}
	}

	public final Bitfield_type_specContext bitfield_type_spec() throws RecognitionException {
		Bitfield_type_specContext _localctx = new Bitfield_type_specContext(_ctx, getState());
		enterRule(_localctx, 90, RULE_bitfield_type_spec);
		try {
			setState(703);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_SHORT:
			case KW_LONG:
			case KW_UNSIGNED:
			case KW_INT8:
			case KW_UINT8:
			case KW_INT16:
			case KW_UINT16:
			case KW_INT32:
			case KW_UINT32:
			case KW_INT64:
			case KW_UINT64:
				enterOuterAlt(_localctx, 1);
				{
				setState(700);
				integer_type();
				}
				break;
			case KW_BOOLEAN:
				enterOuterAlt(_localctx, 2);
				{
				setState(701);
				boolean_type();
				}
				break;
			case KW_OCTET:
				enterOuterAlt(_localctx, 3);
				{
				setState(702);
				octet_type();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Base_type_specContext extends ParserRuleContext {
		public Floating_pt_typeContext floating_pt_type() {
			return getRuleContext(Floating_pt_typeContext.class,0);
		}
		public Integer_typeContext integer_type() {
			return getRuleContext(Integer_typeContext.class,0);
		}
		public Char_typeContext char_type() {
			return getRuleContext(Char_typeContext.class,0);
		}
		public Wide_char_typeContext wide_char_type() {
			return getRuleContext(Wide_char_typeContext.class,0);
		}
		public Boolean_typeContext boolean_type() {
			return getRuleContext(Boolean_typeContext.class,0);
		}
		public Octet_typeContext octet_type() {
			return getRuleContext(Octet_typeContext.class,0);
		}
		public Any_typeContext any_type() {
			return getRuleContext(Any_typeContext.class,0);
		}
		public Object_typeContext object_type() {
			return getRuleContext(Object_typeContext.class,0);
		}
		public Value_base_typeContext value_base_type() {
			return getRuleContext(Value_base_typeContext.class,0);
		}
		public Base_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_base_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBase_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBase_type_spec(this);
		}
	}

	public final Base_type_specContext base_type_spec() throws RecognitionException {
		Base_type_specContext _localctx = new Base_type_specContext(_ctx, getState());
		enterRule(_localctx, 92, RULE_base_type_spec);
		try {
			setState(714);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,41,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(705);
				floating_pt_type();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(706);
				integer_type();
				}
				break;
			case 3:
				enterOuterAlt(_localctx, 3);
				{
				setState(707);
				char_type();
				}
				break;
			case 4:
				enterOuterAlt(_localctx, 4);
				{
				setState(708);
				wide_char_type();
				}
				break;
			case 5:
				enterOuterAlt(_localctx, 5);
				{
				setState(709);
				boolean_type();
				}
				break;
			case 6:
				enterOuterAlt(_localctx, 6);
				{
				setState(710);
				octet_type();
				}
				break;
			case 7:
				enterOuterAlt(_localctx, 7);
				{
				setState(711);
				any_type();
				}
				break;
			case 8:
				enterOuterAlt(_localctx, 8);
				{
				setState(712);
				object_type();
				}
				break;
			case 9:
				enterOuterAlt(_localctx, 9);
				{
				setState(713);
				value_base_type();
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Template_type_specContext extends ParserRuleContext {
		public Sequence_typeContext sequence_type() {
			return getRuleContext(Sequence_typeContext.class,0);
		}
		public Set_typeContext set_type() {
			return getRuleContext(Set_typeContext.class,0);
		}
		public Map_typeContext map_type() {
			return getRuleContext(Map_typeContext.class,0);
		}
		public String_typeContext string_type() {
			return getRuleContext(String_typeContext.class,0);
		}
		public Wide_string_typeContext wide_string_type() {
			return getRuleContext(Wide_string_typeContext.class,0);
		}
		public Fixed_pt_typeContext fixed_pt_type() {
			return getRuleContext(Fixed_pt_typeContext.class,0);
		}
		public Template_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_template_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterTemplate_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitTemplate_type_spec(this);
		}
	}

	public final Template_type_specContext template_type_spec() throws RecognitionException {
		Template_type_specContext _localctx = new Template_type_specContext(_ctx, getState());
		enterRule(_localctx, 94, RULE_template_type_spec);
		try {
			setState(722);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_SEQUENCE:
				enterOuterAlt(_localctx, 1);
				{
				setState(716);
				sequence_type();
				}
				break;
			case KW_SET:
				enterOuterAlt(_localctx, 2);
				{
				setState(717);
				set_type();
				}
				break;
			case KW_MAP:
				enterOuterAlt(_localctx, 3);
				{
				setState(718);
				map_type();
				}
				break;
			case KW_STRING:
				enterOuterAlt(_localctx, 4);
				{
				setState(719);
				string_type();
				}
				break;
			case KW_WSTRING:
				enterOuterAlt(_localctx, 5);
				{
				setState(720);
				wide_string_type();
				}
				break;
			case KW_FIXED:
				enterOuterAlt(_localctx, 6);
				{
				setState(721);
				fixed_pt_type();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Constr_type_specContext extends ParserRuleContext {
		public Struct_typeContext struct_type() {
			return getRuleContext(Struct_typeContext.class,0);
		}
		public Union_typeContext union_type() {
			return getRuleContext(Union_typeContext.class,0);
		}
		public Enum_typeContext enum_type() {
			return getRuleContext(Enum_typeContext.class,0);
		}
		public Bitset_typeContext bitset_type() {
			return getRuleContext(Bitset_typeContext.class,0);
		}
		public Bitmask_typeContext bitmask_type() {
			return getRuleContext(Bitmask_typeContext.class,0);
		}
		public Constr_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_constr_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterConstr_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitConstr_type_spec(this);
		}
	}

	public final Constr_type_specContext constr_type_spec() throws RecognitionException {
		Constr_type_specContext _localctx = new Constr_type_specContext(_ctx, getState());
		enterRule(_localctx, 96, RULE_constr_type_spec);
		try {
			setState(729);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_STRUCT:
				enterOuterAlt(_localctx, 1);
				{
				setState(724);
				struct_type();
				}
				break;
			case KW_UNION:
				enterOuterAlt(_localctx, 2);
				{
				setState(725);
				union_type();
				}
				break;
			case KW_ENUM:
				enterOuterAlt(_localctx, 3);
				{
				setState(726);
				enum_type();
				}
				break;
			case KW_BITSET:
				enterOuterAlt(_localctx, 4);
				{
				setState(727);
				bitset_type();
				}
				break;
			case KW_BITMASK:
				enterOuterAlt(_localctx, 5);
				{
				setState(728);
				bitmask_type();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Simple_declaratorsContext extends ParserRuleContext {
		public List<IdentifierContext> identifier() {
			return getRuleContexts(IdentifierContext.class);
		}
		public IdentifierContext identifier(int i) {
			return getRuleContext(IdentifierContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Simple_declaratorsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_simple_declarators; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSimple_declarators(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSimple_declarators(this);
		}
	}

	public final Simple_declaratorsContext simple_declarators() throws RecognitionException {
		Simple_declaratorsContext _localctx = new Simple_declaratorsContext(_ctx, getState());
		enterRule(_localctx, 98, RULE_simple_declarators);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(731);
			identifier();
			setState(736);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(732);
				match(COMMA);
				setState(733);
				identifier();
				}
				}
				setState(738);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class DeclaratorsContext extends ParserRuleContext {
		public List<DeclaratorContext> declarator() {
			return getRuleContexts(DeclaratorContext.class);
		}
		public DeclaratorContext declarator(int i) {
			return getRuleContext(DeclaratorContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public DeclaratorsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_declarators; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterDeclarators(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitDeclarators(this);
		}
	}

	public final DeclaratorsContext declarators() throws RecognitionException {
		DeclaratorsContext _localctx = new DeclaratorsContext(_ctx, getState());
		enterRule(_localctx, 100, RULE_declarators);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(739);
			declarator();
			setState(744);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(740);
				match(COMMA);
				setState(741);
				declarator();
				}
				}
				setState(746);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class DeclaratorContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Simple_declaratorContext simple_declarator() {
			return getRuleContext(Simple_declaratorContext.class,0);
		}
		public Complex_declaratorContext complex_declarator() {
			return getRuleContext(Complex_declaratorContext.class,0);
		}
		public DeclaratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_declarator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterDeclarator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitDeclarator(this);
		}
	}

	public final DeclaratorContext declarator() throws RecognitionException {
		DeclaratorContext _localctx = new DeclaratorContext(_ctx, getState());
		enterRule(_localctx, 102, RULE_declarator);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(747);
			annapps();
			setState(750);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,46,_ctx) ) {
			case 1:
				{
				setState(748);
				simple_declarator();
				}
				break;
			case 2:
				{
				setState(749);
				complex_declarator();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Simple_declaratorContext extends ParserRuleContext {
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public Simple_declaratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_simple_declarator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSimple_declarator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSimple_declarator(this);
		}
	}

	public final Simple_declaratorContext simple_declarator() throws RecognitionException {
		Simple_declaratorContext _localctx = new Simple_declaratorContext(_ctx, getState());
		enterRule(_localctx, 104, RULE_simple_declarator);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(752);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Complex_declaratorContext extends ParserRuleContext {
		public Array_declaratorContext array_declarator() {
			return getRuleContext(Array_declaratorContext.class,0);
		}
		public Complex_declaratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_complex_declarator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComplex_declarator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComplex_declarator(this);
		}
	}

	public final Complex_declaratorContext complex_declarator() throws RecognitionException {
		Complex_declaratorContext _localctx = new Complex_declaratorContext(_ctx, getState());
		enterRule(_localctx, 106, RULE_complex_declarator);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(754);
			array_declarator();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Floating_pt_typeContext extends ParserRuleContext {
		public TerminalNode KW_FLOAT() { return getToken(IDLParser.KW_FLOAT, 0); }
		public TerminalNode KW_DOUBLE() { return getToken(IDLParser.KW_DOUBLE, 0); }
		public TerminalNode KW_LONG() { return getToken(IDLParser.KW_LONG, 0); }
		public Floating_pt_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_floating_pt_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterFloating_pt_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitFloating_pt_type(this);
		}
	}

	public final Floating_pt_typeContext floating_pt_type() throws RecognitionException {
		Floating_pt_typeContext _localctx = new Floating_pt_typeContext(_ctx, getState());
		enterRule(_localctx, 108, RULE_floating_pt_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(760);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_FLOAT:
				{
				setState(756);
				match(KW_FLOAT);
				}
				break;
			case KW_DOUBLE:
				{
				setState(757);
				match(KW_DOUBLE);
				}
				break;
			case KW_LONG:
				{
				setState(758);
				match(KW_LONG);
				setState(759);
				match(KW_DOUBLE);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Integer_typeContext extends ParserRuleContext {
		public Signed_intContext signed_int() {
			return getRuleContext(Signed_intContext.class,0);
		}
		public Unsigned_intContext unsigned_int() {
			return getRuleContext(Unsigned_intContext.class,0);
		}
		public Integer_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_integer_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInteger_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInteger_type(this);
		}
	}

	public final Integer_typeContext integer_type() throws RecognitionException {
		Integer_typeContext _localctx = new Integer_typeContext(_ctx, getState());
		enterRule(_localctx, 110, RULE_integer_type);
		try {
			setState(764);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_SHORT:
			case KW_LONG:
			case KW_INT8:
			case KW_INT16:
			case KW_INT32:
			case KW_INT64:
				enterOuterAlt(_localctx, 1);
				{
				setState(762);
				signed_int();
				}
				break;
			case KW_UNSIGNED:
			case KW_UINT8:
			case KW_UINT16:
			case KW_UINT32:
			case KW_UINT64:
				enterOuterAlt(_localctx, 2);
				{
				setState(763);
				unsigned_int();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Signed_intContext extends ParserRuleContext {
		public Signed_short_intContext signed_short_int() {
			return getRuleContext(Signed_short_intContext.class,0);
		}
		public Signed_long_intContext signed_long_int() {
			return getRuleContext(Signed_long_intContext.class,0);
		}
		public Signed_longlong_intContext signed_longlong_int() {
			return getRuleContext(Signed_longlong_intContext.class,0);
		}
		public Signed_tiny_intContext signed_tiny_int() {
			return getRuleContext(Signed_tiny_intContext.class,0);
		}
		public Signed_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_signed_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSigned_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSigned_int(this);
		}
	}

	public final Signed_intContext signed_int() throws RecognitionException {
		Signed_intContext _localctx = new Signed_intContext(_ctx, getState());
		enterRule(_localctx, 112, RULE_signed_int);
		try {
			setState(770);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,49,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(766);
				signed_short_int();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(767);
				signed_long_int();
				}
				break;
			case 3:
				enterOuterAlt(_localctx, 3);
				{
				setState(768);
				signed_longlong_int();
				}
				break;
			case 4:
				enterOuterAlt(_localctx, 4);
				{
				setState(769);
				signed_tiny_int();
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Signed_tiny_intContext extends ParserRuleContext {
		public TerminalNode KW_INT8() { return getToken(IDLParser.KW_INT8, 0); }
		public Signed_tiny_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_signed_tiny_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSigned_tiny_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSigned_tiny_int(this);
		}
	}

	public final Signed_tiny_intContext signed_tiny_int() throws RecognitionException {
		Signed_tiny_intContext _localctx = new Signed_tiny_intContext(_ctx, getState());
		enterRule(_localctx, 114, RULE_signed_tiny_int);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(772);
			match(KW_INT8);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Signed_short_intContext extends ParserRuleContext {
		public TerminalNode KW_SHORT() { return getToken(IDLParser.KW_SHORT, 0); }
		public TerminalNode KW_INT16() { return getToken(IDLParser.KW_INT16, 0); }
		public Signed_short_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_signed_short_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSigned_short_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSigned_short_int(this);
		}
	}

	public final Signed_short_intContext signed_short_int() throws RecognitionException {
		Signed_short_intContext _localctx = new Signed_short_intContext(_ctx, getState());
		enterRule(_localctx, 116, RULE_signed_short_int);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(774);
			_la = _input.LA(1);
			if ( !(_la==KW_SHORT || _la==KW_INT16) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Signed_long_intContext extends ParserRuleContext {
		public TerminalNode KW_LONG() { return getToken(IDLParser.KW_LONG, 0); }
		public TerminalNode KW_INT32() { return getToken(IDLParser.KW_INT32, 0); }
		public Signed_long_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_signed_long_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSigned_long_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSigned_long_int(this);
		}
	}

	public final Signed_long_intContext signed_long_int() throws RecognitionException {
		Signed_long_intContext _localctx = new Signed_long_intContext(_ctx, getState());
		enterRule(_localctx, 118, RULE_signed_long_int);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(776);
			_la = _input.LA(1);
			if ( !(_la==KW_LONG || _la==KW_INT32) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Signed_longlong_intContext extends ParserRuleContext {
		public List<TerminalNode> KW_LONG() { return getTokens(IDLParser.KW_LONG); }
		public TerminalNode KW_LONG(int i) {
			return getToken(IDLParser.KW_LONG, i);
		}
		public TerminalNode KW_INT64() { return getToken(IDLParser.KW_INT64, 0); }
		public Signed_longlong_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_signed_longlong_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSigned_longlong_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSigned_longlong_int(this);
		}
	}

	public final Signed_longlong_intContext signed_longlong_int() throws RecognitionException {
		Signed_longlong_intContext _localctx = new Signed_longlong_intContext(_ctx, getState());
		enterRule(_localctx, 120, RULE_signed_longlong_int);
		try {
			setState(781);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_LONG:
				enterOuterAlt(_localctx, 1);
				{
				setState(778);
				match(KW_LONG);
				setState(779);
				match(KW_LONG);
				}
				break;
			case KW_INT64:
				enterOuterAlt(_localctx, 2);
				{
				setState(780);
				match(KW_INT64);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Unsigned_intContext extends ParserRuleContext {
		public Unsigned_short_intContext unsigned_short_int() {
			return getRuleContext(Unsigned_short_intContext.class,0);
		}
		public Unsigned_long_intContext unsigned_long_int() {
			return getRuleContext(Unsigned_long_intContext.class,0);
		}
		public Unsigned_longlong_intContext unsigned_longlong_int() {
			return getRuleContext(Unsigned_longlong_intContext.class,0);
		}
		public Unsigned_tiny_intContext unsigned_tiny_int() {
			return getRuleContext(Unsigned_tiny_intContext.class,0);
		}
		public Unsigned_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unsigned_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnsigned_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnsigned_int(this);
		}
	}

	public final Unsigned_intContext unsigned_int() throws RecognitionException {
		Unsigned_intContext _localctx = new Unsigned_intContext(_ctx, getState());
		enterRule(_localctx, 122, RULE_unsigned_int);
		try {
			setState(787);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,51,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(783);
				unsigned_short_int();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(784);
				unsigned_long_int();
				}
				break;
			case 3:
				enterOuterAlt(_localctx, 3);
				{
				setState(785);
				unsigned_longlong_int();
				}
				break;
			case 4:
				enterOuterAlt(_localctx, 4);
				{
				setState(786);
				unsigned_tiny_int();
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Unsigned_tiny_intContext extends ParserRuleContext {
		public TerminalNode KW_UINT8() { return getToken(IDLParser.KW_UINT8, 0); }
		public Unsigned_tiny_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unsigned_tiny_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnsigned_tiny_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnsigned_tiny_int(this);
		}
	}

	public final Unsigned_tiny_intContext unsigned_tiny_int() throws RecognitionException {
		Unsigned_tiny_intContext _localctx = new Unsigned_tiny_intContext(_ctx, getState());
		enterRule(_localctx, 124, RULE_unsigned_tiny_int);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(789);
			match(KW_UINT8);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Unsigned_short_intContext extends ParserRuleContext {
		public TerminalNode KW_UNSIGNED() { return getToken(IDLParser.KW_UNSIGNED, 0); }
		public TerminalNode KW_SHORT() { return getToken(IDLParser.KW_SHORT, 0); }
		public TerminalNode KW_UINT16() { return getToken(IDLParser.KW_UINT16, 0); }
		public Unsigned_short_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unsigned_short_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnsigned_short_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnsigned_short_int(this);
		}
	}

	public final Unsigned_short_intContext unsigned_short_int() throws RecognitionException {
		Unsigned_short_intContext _localctx = new Unsigned_short_intContext(_ctx, getState());
		enterRule(_localctx, 126, RULE_unsigned_short_int);
		try {
			setState(794);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_UNSIGNED:
				enterOuterAlt(_localctx, 1);
				{
				setState(791);
				match(KW_UNSIGNED);
				setState(792);
				match(KW_SHORT);
				}
				break;
			case KW_UINT16:
				enterOuterAlt(_localctx, 2);
				{
				setState(793);
				match(KW_UINT16);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Unsigned_long_intContext extends ParserRuleContext {
		public TerminalNode KW_UNSIGNED() { return getToken(IDLParser.KW_UNSIGNED, 0); }
		public TerminalNode KW_LONG() { return getToken(IDLParser.KW_LONG, 0); }
		public TerminalNode KW_UINT32() { return getToken(IDLParser.KW_UINT32, 0); }
		public Unsigned_long_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unsigned_long_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnsigned_long_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnsigned_long_int(this);
		}
	}

	public final Unsigned_long_intContext unsigned_long_int() throws RecognitionException {
		Unsigned_long_intContext _localctx = new Unsigned_long_intContext(_ctx, getState());
		enterRule(_localctx, 128, RULE_unsigned_long_int);
		try {
			setState(799);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_UNSIGNED:
				enterOuterAlt(_localctx, 1);
				{
				setState(796);
				match(KW_UNSIGNED);
				setState(797);
				match(KW_LONG);
				}
				break;
			case KW_UINT32:
				enterOuterAlt(_localctx, 2);
				{
				setState(798);
				match(KW_UINT32);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Unsigned_longlong_intContext extends ParserRuleContext {
		public TerminalNode KW_UNSIGNED() { return getToken(IDLParser.KW_UNSIGNED, 0); }
		public List<TerminalNode> KW_LONG() { return getTokens(IDLParser.KW_LONG); }
		public TerminalNode KW_LONG(int i) {
			return getToken(IDLParser.KW_LONG, i);
		}
		public TerminalNode KW_UINT64() { return getToken(IDLParser.KW_UINT64, 0); }
		public Unsigned_longlong_intContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unsigned_longlong_int; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnsigned_longlong_int(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnsigned_longlong_int(this);
		}
	}

	public final Unsigned_longlong_intContext unsigned_longlong_int() throws RecognitionException {
		Unsigned_longlong_intContext _localctx = new Unsigned_longlong_intContext(_ctx, getState());
		enterRule(_localctx, 130, RULE_unsigned_longlong_int);
		try {
			setState(805);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_UNSIGNED:
				enterOuterAlt(_localctx, 1);
				{
				setState(801);
				match(KW_UNSIGNED);
				setState(802);
				match(KW_LONG);
				setState(803);
				match(KW_LONG);
				}
				break;
			case KW_UINT64:
				enterOuterAlt(_localctx, 2);
				{
				setState(804);
				match(KW_UINT64);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Char_typeContext extends ParserRuleContext {
		public TerminalNode KW_CHAR() { return getToken(IDLParser.KW_CHAR, 0); }
		public Char_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_char_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterChar_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitChar_type(this);
		}
	}

	public final Char_typeContext char_type() throws RecognitionException {
		Char_typeContext _localctx = new Char_typeContext(_ctx, getState());
		enterRule(_localctx, 132, RULE_char_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(807);
			match(KW_CHAR);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Wide_char_typeContext extends ParserRuleContext {
		public TerminalNode KW_WCHAR() { return getToken(IDLParser.KW_WCHAR, 0); }
		public Wide_char_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_wide_char_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterWide_char_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitWide_char_type(this);
		}
	}

	public final Wide_char_typeContext wide_char_type() throws RecognitionException {
		Wide_char_typeContext _localctx = new Wide_char_typeContext(_ctx, getState());
		enterRule(_localctx, 134, RULE_wide_char_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(809);
			match(KW_WCHAR);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Boolean_typeContext extends ParserRuleContext {
		public TerminalNode KW_BOOLEAN() { return getToken(IDLParser.KW_BOOLEAN, 0); }
		public Boolean_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_boolean_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBoolean_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBoolean_type(this);
		}
	}

	public final Boolean_typeContext boolean_type() throws RecognitionException {
		Boolean_typeContext _localctx = new Boolean_typeContext(_ctx, getState());
		enterRule(_localctx, 136, RULE_boolean_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(811);
			match(KW_BOOLEAN);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Octet_typeContext extends ParserRuleContext {
		public TerminalNode KW_OCTET() { return getToken(IDLParser.KW_OCTET, 0); }
		public Octet_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_octet_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterOctet_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitOctet_type(this);
		}
	}

	public final Octet_typeContext octet_type() throws RecognitionException {
		Octet_typeContext _localctx = new Octet_typeContext(_ctx, getState());
		enterRule(_localctx, 138, RULE_octet_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(813);
			match(KW_OCTET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Any_typeContext extends ParserRuleContext {
		public TerminalNode KW_ANY() { return getToken(IDLParser.KW_ANY, 0); }
		public Any_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_any_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAny_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAny_type(this);
		}
	}

	public final Any_typeContext any_type() throws RecognitionException {
		Any_typeContext _localctx = new Any_typeContext(_ctx, getState());
		enterRule(_localctx, 140, RULE_any_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(815);
			match(KW_ANY);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Object_typeContext extends ParserRuleContext {
		public TerminalNode KW_OBJECT() { return getToken(IDLParser.KW_OBJECT, 0); }
		public Object_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_object_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterObject_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitObject_type(this);
		}
	}

	public final Object_typeContext object_type() throws RecognitionException {
		Object_typeContext _localctx = new Object_typeContext(_ctx, getState());
		enterRule(_localctx, 142, RULE_object_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(817);
			match(KW_OBJECT);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_declContext extends ParserRuleContext {
		public Annotation_defContext annotation_def() {
			return getRuleContext(Annotation_defContext.class,0);
		}
		public Annotation_forward_dclContext annotation_forward_dcl() {
			return getRuleContext(Annotation_forward_dclContext.class,0);
		}
		public Annotation_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_decl(this);
		}
	}

	public final Annotation_declContext annotation_decl() throws RecognitionException {
		Annotation_declContext _localctx = new Annotation_declContext(_ctx, getState());
		enterRule(_localctx, 144, RULE_annotation_decl);
		try {
			setState(821);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,55,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(819);
				annotation_def();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(820);
				annotation_forward_dcl();
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_defContext extends ParserRuleContext {
		public Annotation_headerContext annotation_header() {
			return getRuleContext(Annotation_headerContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public Annotation_bodyContext annotation_body() {
			return getRuleContext(Annotation_bodyContext.class,0);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public Annotation_defContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_def; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_def(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_def(this);
		}
	}

	public final Annotation_defContext annotation_def() throws RecognitionException {
		Annotation_defContext _localctx = new Annotation_defContext(_ctx, getState());
		enterRule(_localctx, 146, RULE_annotation_def);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(823);
			annotation_header();
			setState(824);
			match(LEFT_BRACE);
			setState(825);
			annotation_body();
			setState(826);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_headerContext extends ParserRuleContext {
		public TerminalNode KW_AT_ANNOTATION() { return getToken(IDLParser.KW_AT_ANNOTATION, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Annotation_inheritance_specContext annotation_inheritance_spec() {
			return getRuleContext(Annotation_inheritance_specContext.class,0);
		}
		public Annotation_headerContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_header; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_header(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_header(this);
		}
	}

	public final Annotation_headerContext annotation_header() throws RecognitionException {
		Annotation_headerContext _localctx = new Annotation_headerContext(_ctx, getState());
		enterRule(_localctx, 148, RULE_annotation_header);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(828);
			match(KW_AT_ANNOTATION);
			setState(829);
			identifier();
			setState(831);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COLON) {
				{
				setState(830);
				annotation_inheritance_spec();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_inheritance_specContext extends ParserRuleContext {
		public TerminalNode COLON() { return getToken(IDLParser.COLON, 0); }
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Annotation_inheritance_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_inheritance_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_inheritance_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_inheritance_spec(this);
		}
	}

	public final Annotation_inheritance_specContext annotation_inheritance_spec() throws RecognitionException {
		Annotation_inheritance_specContext _localctx = new Annotation_inheritance_specContext(_ctx, getState());
		enterRule(_localctx, 150, RULE_annotation_inheritance_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(833);
			match(COLON);
			setState(834);
			scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_bodyContext extends ParserRuleContext {
		public List<Annotation_memberContext> annotation_member() {
			return getRuleContexts(Annotation_memberContext.class);
		}
		public Annotation_memberContext annotation_member(int i) {
			return getRuleContext(Annotation_memberContext.class,i);
		}
		public List<Enum_typeContext> enum_type() {
			return getRuleContexts(Enum_typeContext.class);
		}
		public Enum_typeContext enum_type(int i) {
			return getRuleContext(Enum_typeContext.class,i);
		}
		public List<TerminalNode> SEMICOLON() { return getTokens(IDLParser.SEMICOLON); }
		public TerminalNode SEMICOLON(int i) {
			return getToken(IDLParser.SEMICOLON, i);
		}
		public List<Const_declContext> const_decl() {
			return getRuleContexts(Const_declContext.class);
		}
		public Const_declContext const_decl(int i) {
			return getRuleContext(Const_declContext.class,i);
		}
		public List<TerminalNode> KW_TYPEDEF() { return getTokens(IDLParser.KW_TYPEDEF); }
		public TerminalNode KW_TYPEDEF(int i) {
			return getToken(IDLParser.KW_TYPEDEF, i);
		}
		public List<Type_declaratorContext> type_declarator() {
			return getRuleContexts(Type_declaratorContext.class);
		}
		public Type_declaratorContext type_declarator(int i) {
			return getRuleContext(Type_declaratorContext.class,i);
		}
		public Annotation_bodyContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_body; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_body(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_body(this);
		}
	}

	public final Annotation_bodyContext annotation_body() throws RecognitionException {
		Annotation_bodyContext _localctx = new Annotation_bodyContext(_ctx, getState());
		enterRule(_localctx, 152, RULE_annotation_body);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(849);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -2161652465936629760L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 843325556154433L) != 0)) {
				{
				setState(847);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case DOUBLE_COLON:
				case AT:
				case KW_STRING:
				case KW_OCTET:
				case KW_WCHAR:
				case KW_SHORT:
				case KW_LONG:
				case KW_WSTRING:
				case KW_UNSIGNED:
				case KW_FIXED:
				case KW_CHAR:
				case KW_FLOAT:
				case KW_BOOLEAN:
				case KW_DOUBLE:
				case KW_INT8:
				case KW_UINT8:
				case KW_INT16:
				case KW_UINT16:
				case KW_INT32:
				case KW_UINT32:
				case KW_INT64:
				case KW_UINT64:
				case ID:
					{
					setState(836);
					annotation_member();
					}
					break;
				case KW_ENUM:
					{
					setState(837);
					enum_type();
					setState(838);
					match(SEMICOLON);
					}
					break;
				case KW_CONST:
					{
					setState(840);
					const_decl();
					setState(841);
					match(SEMICOLON);
					}
					break;
				case KW_TYPEDEF:
					{
					setState(843);
					match(KW_TYPEDEF);
					setState(844);
					type_declarator();
					setState(845);
					match(SEMICOLON);
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(851);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_memberContext extends ParserRuleContext {
		public Const_typeContext const_type() {
			return getRuleContext(Const_typeContext.class,0);
		}
		public Simple_declaratorContext simple_declarator() {
			return getRuleContext(Simple_declaratorContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public TerminalNode KW_DEFAULT() { return getToken(IDLParser.KW_DEFAULT, 0); }
		public Const_expContext const_exp() {
			return getRuleContext(Const_expContext.class,0);
		}
		public Annotation_memberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_member; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_member(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_member(this);
		}
	}

	public final Annotation_memberContext annotation_member() throws RecognitionException {
		Annotation_memberContext _localctx = new Annotation_memberContext(_ctx, getState());
		enterRule(_localctx, 154, RULE_annotation_member);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(852);
			const_type();
			setState(853);
			simple_declarator();
			setState(856);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_DEFAULT) {
				{
				setState(854);
				match(KW_DEFAULT);
				setState(855);
				const_exp();
				}
			}

			setState(858);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_forward_dclContext extends ParserRuleContext {
		public TerminalNode KW_AT_ANNOTATION() { return getToken(IDLParser.KW_AT_ANNOTATION, 0); }
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Annotation_forward_dclContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_forward_dcl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_forward_dcl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_forward_dcl(this);
		}
	}

	public final Annotation_forward_dclContext annotation_forward_dcl() throws RecognitionException {
		Annotation_forward_dclContext _localctx = new Annotation_forward_dclContext(_ctx, getState());
		enterRule(_localctx, 156, RULE_annotation_forward_dcl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(860);
			match(KW_AT_ANNOTATION);
			setState(861);
			scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Bitset_typeContext extends ParserRuleContext {
		public TerminalNode KW_BITSET() { return getToken(IDLParser.KW_BITSET, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public BitfieldContext bitfield() {
			return getRuleContext(BitfieldContext.class,0);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public TerminalNode COLON() { return getToken(IDLParser.COLON, 0); }
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Bitset_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bitset_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBitset_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBitset_type(this);
		}
	}

	public final Bitset_typeContext bitset_type() throws RecognitionException {
		Bitset_typeContext _localctx = new Bitset_typeContext(_ctx, getState());
		enterRule(_localctx, 158, RULE_bitset_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(863);
			match(KW_BITSET);
			setState(864);
			identifier();
			setState(867);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COLON) {
				{
				setState(865);
				match(COLON);
				setState(866);
				scoped_name();
				}
			}

			setState(869);
			match(LEFT_BRACE);
			setState(870);
			bitfield();
			setState(871);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class BitfieldContext extends ParserRuleContext {
		public List<Bitfield_specContext> bitfield_spec() {
			return getRuleContexts(Bitfield_specContext.class);
		}
		public Bitfield_specContext bitfield_spec(int i) {
			return getRuleContext(Bitfield_specContext.class,i);
		}
		public List<TerminalNode> SEMICOLON() { return getTokens(IDLParser.SEMICOLON); }
		public TerminalNode SEMICOLON(int i) {
			return getToken(IDLParser.SEMICOLON, i);
		}
		public List<Simple_declaratorsContext> simple_declarators() {
			return getRuleContexts(Simple_declaratorsContext.class);
		}
		public Simple_declaratorsContext simple_declarators(int i) {
			return getRuleContext(Simple_declaratorsContext.class,i);
		}
		public BitfieldContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bitfield; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBitfield(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBitfield(this);
		}
	}

	public final BitfieldContext bitfield() throws RecognitionException {
		BitfieldContext _localctx = new BitfieldContext(_ctx, getState());
		enterRule(_localctx, 160, RULE_bitfield);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(879); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(873);
				bitfield_spec();
				setState(875);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==AT || _la==ID) {
					{
					setState(874);
					simple_declarators();
					}
				}

				setState(877);
				match(SEMICOLON);
				}
				}
				setState(881); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==AT || _la==KW_BITFIELD );
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Bitfield_specContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public TerminalNode KW_BITFIELD() { return getToken(IDLParser.KW_BITFIELD, 0); }
		public TerminalNode LEFT_ANG_BRACKET() { return getToken(IDLParser.LEFT_ANG_BRACKET, 0); }
		public Positive_int_constContext positive_int_const() {
			return getRuleContext(Positive_int_constContext.class,0);
		}
		public TerminalNode RIGHT_ANG_BRACKET() { return getToken(IDLParser.RIGHT_ANG_BRACKET, 0); }
		public TerminalNode COMMA() { return getToken(IDLParser.COMMA, 0); }
		public Bitfield_type_specContext bitfield_type_spec() {
			return getRuleContext(Bitfield_type_specContext.class,0);
		}
		public Bitfield_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bitfield_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBitfield_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBitfield_spec(this);
		}
	}

	public final Bitfield_specContext bitfield_spec() throws RecognitionException {
		Bitfield_specContext _localctx = new Bitfield_specContext(_ctx, getState());
		enterRule(_localctx, 162, RULE_bitfield_spec);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(883);
			annapps();
			setState(884);
			match(KW_BITFIELD);
			setState(885);
			match(LEFT_ANG_BRACKET);
			setState(886);
			positive_int_const();
			setState(889);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COMMA) {
				{
				setState(887);
				match(COMMA);
				setState(888);
				bitfield_type_spec();
				}
			}

			setState(891);
			match(RIGHT_ANG_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Bitmask_typeContext extends ParserRuleContext {
		public TerminalNode KW_BITMASK() { return getToken(IDLParser.KW_BITMASK, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public Bit_valuesContext bit_values() {
			return getRuleContext(Bit_valuesContext.class,0);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public Bitmask_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bitmask_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBitmask_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBitmask_type(this);
		}
	}

	public final Bitmask_typeContext bitmask_type() throws RecognitionException {
		Bitmask_typeContext _localctx = new Bitmask_typeContext(_ctx, getState());
		enterRule(_localctx, 164, RULE_bitmask_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(893);
			match(KW_BITMASK);
			setState(894);
			identifier();
			setState(895);
			match(LEFT_BRACE);
			setState(896);
			bit_values();
			setState(897);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Bit_valuesContext extends ParserRuleContext {
		public List<IdentifierContext> identifier() {
			return getRuleContexts(IdentifierContext.class);
		}
		public IdentifierContext identifier(int i) {
			return getRuleContext(IdentifierContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Bit_valuesContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_bit_values; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterBit_values(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitBit_values(this);
		}
	}

	public final Bit_valuesContext bit_values() throws RecognitionException {
		Bit_valuesContext _localctx = new Bit_valuesContext(_ctx, getState());
		enterRule(_localctx, 166, RULE_bit_values);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(899);
			identifier();
			setState(904);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(900);
				match(COMMA);
				setState(901);
				identifier();
				}
				}
				setState(906);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Struct_typeContext extends ParserRuleContext {
		public TerminalNode KW_STRUCT() { return getToken(IDLParser.KW_STRUCT, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public Member_listContext member_list() {
			return getRuleContext(Member_listContext.class,0);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public TerminalNode DOUBLE_COLON() { return getToken(IDLParser.DOUBLE_COLON, 0); }
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Struct_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_struct_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterStruct_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitStruct_type(this);
		}
	}

	public final Struct_typeContext struct_type() throws RecognitionException {
		Struct_typeContext _localctx = new Struct_typeContext(_ctx, getState());
		enterRule(_localctx, 168, RULE_struct_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(907);
			match(KW_STRUCT);
			setState(908);
			identifier();
			setState(911);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==DOUBLE_COLON) {
				{
				setState(909);
				match(DOUBLE_COLON);
				setState(910);
				scoped_name();
				}
			}

			setState(913);
			match(LEFT_BRACE);
			setState(914);
			member_list();
			setState(915);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Member_listContext extends ParserRuleContext {
		public List<MemberContext> member() {
			return getRuleContexts(MemberContext.class);
		}
		public MemberContext member(int i) {
			return getRuleContext(MemberContext.class,i);
		}
		public Member_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_member_list; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterMember_list(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitMember_list(this);
		}
	}

	public final Member_listContext member_list() throws RecognitionException {
		Member_listContext _localctx = new Member_listContext(_ctx, getState());
		enterRule(_localctx, 170, RULE_member_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(920);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -2160953176541364224L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844253269256321L) != 0)) {
				{
				{
				setState(917);
				member();
				}
				}
				setState(922);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class MemberContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Type_specContext type_spec() {
			return getRuleContext(Type_specContext.class,0);
		}
		public DeclaratorsContext declarators() {
			return getRuleContext(DeclaratorsContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public MemberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_member; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterMember(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitMember(this);
		}
	}

	public final MemberContext member() throws RecognitionException {
		MemberContext _localctx = new MemberContext(_ctx, getState());
		enterRule(_localctx, 172, RULE_member);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(923);
			annapps();
			setState(924);
			type_spec();
			setState(925);
			declarators();
			setState(926);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Union_typeContext extends ParserRuleContext {
		public TerminalNode KW_UNION() { return getToken(IDLParser.KW_UNION, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode KW_SWITCH() { return getToken(IDLParser.KW_SWITCH, 0); }
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Switch_type_specContext switch_type_spec() {
			return getRuleContext(Switch_type_specContext.class,0);
		}
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public Switch_bodyContext switch_body() {
			return getRuleContext(Switch_bodyContext.class,0);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public Union_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_union_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUnion_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUnion_type(this);
		}
	}

	public final Union_typeContext union_type() throws RecognitionException {
		Union_typeContext _localctx = new Union_typeContext(_ctx, getState());
		enterRule(_localctx, 174, RULE_union_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(928);
			match(KW_UNION);
			setState(929);
			identifier();
			setState(930);
			match(KW_SWITCH);
			setState(931);
			match(LEFT_BRACKET);
			setState(932);
			annapps();
			setState(933);
			switch_type_spec();
			setState(934);
			match(RIGHT_BRACKET);
			setState(935);
			match(LEFT_BRACE);
			setState(936);
			switch_body();
			setState(937);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Switch_type_specContext extends ParserRuleContext {
		public Integer_typeContext integer_type() {
			return getRuleContext(Integer_typeContext.class,0);
		}
		public Char_typeContext char_type() {
			return getRuleContext(Char_typeContext.class,0);
		}
		public Wide_char_typeContext wide_char_type() {
			return getRuleContext(Wide_char_typeContext.class,0);
		}
		public Octet_typeContext octet_type() {
			return getRuleContext(Octet_typeContext.class,0);
		}
		public Boolean_typeContext boolean_type() {
			return getRuleContext(Boolean_typeContext.class,0);
		}
		public Enum_typeContext enum_type() {
			return getRuleContext(Enum_typeContext.class,0);
		}
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Switch_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_switch_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSwitch_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSwitch_type_spec(this);
		}
	}

	public final Switch_type_specContext switch_type_spec() throws RecognitionException {
		Switch_type_specContext _localctx = new Switch_type_specContext(_ctx, getState());
		enterRule(_localctx, 176, RULE_switch_type_spec);
		try {
			setState(946);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_SHORT:
			case KW_LONG:
			case KW_UNSIGNED:
			case KW_INT8:
			case KW_UINT8:
			case KW_INT16:
			case KW_UINT16:
			case KW_INT32:
			case KW_UINT32:
			case KW_INT64:
			case KW_UINT64:
				enterOuterAlt(_localctx, 1);
				{
				setState(939);
				integer_type();
				}
				break;
			case KW_CHAR:
				enterOuterAlt(_localctx, 2);
				{
				setState(940);
				char_type();
				}
				break;
			case KW_WCHAR:
				enterOuterAlt(_localctx, 3);
				{
				setState(941);
				wide_char_type();
				}
				break;
			case KW_OCTET:
				enterOuterAlt(_localctx, 4);
				{
				setState(942);
				octet_type();
				}
				break;
			case KW_BOOLEAN:
				enterOuterAlt(_localctx, 5);
				{
				setState(943);
				boolean_type();
				}
				break;
			case KW_ENUM:
				enterOuterAlt(_localctx, 6);
				{
				setState(944);
				enum_type();
				}
				break;
			case DOUBLE_COLON:
			case ID:
				enterOuterAlt(_localctx, 7);
				{
				setState(945);
				scoped_name();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Switch_bodyContext extends ParserRuleContext {
		public List<Case_stmtContext> case_stmt() {
			return getRuleContexts(Case_stmtContext.class);
		}
		public Case_stmtContext case_stmt(int i) {
			return getRuleContext(Case_stmtContext.class,i);
		}
		public Switch_bodyContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_switch_body; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSwitch_body(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSwitch_body(this);
		}
	}

	public final Switch_bodyContext switch_body() throws RecognitionException {
		Switch_bodyContext _localctx = new Switch_bodyContext(_ctx, getState());
		enterRule(_localctx, 178, RULE_switch_body);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(949); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(948);
				case_stmt();
				}
				}
				setState(951); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( ((((_la - 35)) & ~0x3f) == 0 && ((1L << (_la - 35)) & 281474993487873L) != 0) );
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Case_stmtContext extends ParserRuleContext {
		public Element_specContext element_spec() {
			return getRuleContext(Element_specContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public List<Case_labelContext> case_label() {
			return getRuleContexts(Case_labelContext.class);
		}
		public Case_labelContext case_label(int i) {
			return getRuleContext(Case_labelContext.class,i);
		}
		public Case_stmtContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_case_stmt; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterCase_stmt(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitCase_stmt(this);
		}
	}

	public final Case_stmtContext case_stmt() throws RecognitionException {
		Case_stmtContext _localctx = new Case_stmtContext(_ctx, getState());
		enterRule(_localctx, 180, RULE_case_stmt);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(954); 
			_errHandler.sync(this);
			_alt = 1;
			do {
				switch (_alt) {
				case 1:
					{
					{
					setState(953);
					case_label();
					}
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				setState(956); 
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,69,_ctx);
			} while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER );
			setState(958);
			element_spec();
			setState(959);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Case_labelContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public TerminalNode KW_CASE() { return getToken(IDLParser.KW_CASE, 0); }
		public Const_expContext const_exp() {
			return getRuleContext(Const_expContext.class,0);
		}
		public TerminalNode COLON() { return getToken(IDLParser.COLON, 0); }
		public TerminalNode KW_DEFAULT() { return getToken(IDLParser.KW_DEFAULT, 0); }
		public Case_labelContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_case_label; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterCase_label(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitCase_label(this);
		}
	}

	public final Case_labelContext case_label() throws RecognitionException {
		Case_labelContext _localctx = new Case_labelContext(_ctx, getState());
		enterRule(_localctx, 182, RULE_case_label);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(961);
			annapps();
			setState(968);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_CASE:
				{
				setState(962);
				match(KW_CASE);
				setState(963);
				const_exp();
				setState(964);
				match(COLON);
				}
				break;
			case KW_DEFAULT:
				{
				setState(966);
				match(KW_DEFAULT);
				setState(967);
				match(COLON);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Element_specContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Type_specContext type_spec() {
			return getRuleContext(Type_specContext.class,0);
		}
		public DeclaratorContext declarator() {
			return getRuleContext(DeclaratorContext.class,0);
		}
		public Element_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_element_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterElement_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitElement_spec(this);
		}
	}

	public final Element_specContext element_spec() throws RecognitionException {
		Element_specContext _localctx = new Element_specContext(_ctx, getState());
		enterRule(_localctx, 184, RULE_element_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(970);
			annapps();
			setState(971);
			type_spec();
			setState(972);
			declarator();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Enum_typeContext extends ParserRuleContext {
		public TerminalNode KW_ENUM() { return getToken(IDLParser.KW_ENUM, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public List<EnumeratorContext> enumerator() {
			return getRuleContexts(EnumeratorContext.class);
		}
		public EnumeratorContext enumerator(int i) {
			return getRuleContext(EnumeratorContext.class,i);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Enum_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_enum_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEnum_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEnum_type(this);
		}
	}

	public final Enum_typeContext enum_type() throws RecognitionException {
		Enum_typeContext _localctx = new Enum_typeContext(_ctx, getState());
		enterRule(_localctx, 186, RULE_enum_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(974);
			match(KW_ENUM);
			setState(975);
			identifier();
			setState(976);
			match(LEFT_BRACE);
			setState(977);
			enumerator();
			setState(982);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(978);
				match(COMMA);
				setState(979);
				enumerator();
				}
				}
				setState(984);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(985);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class EnumeratorContext extends ParserRuleContext {
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public EnumeratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_enumerator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEnumerator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEnumerator(this);
		}
	}

	public final EnumeratorContext enumerator() throws RecognitionException {
		EnumeratorContext _localctx = new EnumeratorContext(_ctx, getState());
		enterRule(_localctx, 188, RULE_enumerator);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(987);
			identifier();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Sequence_typeContext extends ParserRuleContext {
		public TerminalNode KW_SEQUENCE() { return getToken(IDLParser.KW_SEQUENCE, 0); }
		public TerminalNode LEFT_ANG_BRACKET() { return getToken(IDLParser.LEFT_ANG_BRACKET, 0); }
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Simple_type_specContext simple_type_spec() {
			return getRuleContext(Simple_type_specContext.class,0);
		}
		public TerminalNode RIGHT_ANG_BRACKET() { return getToken(IDLParser.RIGHT_ANG_BRACKET, 0); }
		public TerminalNode COMMA() { return getToken(IDLParser.COMMA, 0); }
		public Positive_int_constContext positive_int_const() {
			return getRuleContext(Positive_int_constContext.class,0);
		}
		public Sequence_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sequence_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSequence_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSequence_type(this);
		}
	}

	public final Sequence_typeContext sequence_type() throws RecognitionException {
		Sequence_typeContext _localctx = new Sequence_typeContext(_ctx, getState());
		enterRule(_localctx, 190, RULE_sequence_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(989);
			match(KW_SEQUENCE);
			setState(990);
			match(LEFT_ANG_BRACKET);
			setState(991);
			annapps();
			setState(992);
			simple_type_spec();
			setState(995);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COMMA) {
				{
				setState(993);
				match(COMMA);
				setState(994);
				positive_int_const();
				}
			}

			setState(997);
			match(RIGHT_ANG_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Set_typeContext extends ParserRuleContext {
		public TerminalNode KW_SET() { return getToken(IDLParser.KW_SET, 0); }
		public TerminalNode LEFT_ANG_BRACKET() { return getToken(IDLParser.LEFT_ANG_BRACKET, 0); }
		public Simple_type_specContext simple_type_spec() {
			return getRuleContext(Simple_type_specContext.class,0);
		}
		public TerminalNode RIGHT_ANG_BRACKET() { return getToken(IDLParser.RIGHT_ANG_BRACKET, 0); }
		public TerminalNode COMMA() { return getToken(IDLParser.COMMA, 0); }
		public Positive_int_constContext positive_int_const() {
			return getRuleContext(Positive_int_constContext.class,0);
		}
		public Set_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_set_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSet_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSet_type(this);
		}
	}

	public final Set_typeContext set_type() throws RecognitionException {
		Set_typeContext _localctx = new Set_typeContext(_ctx, getState());
		enterRule(_localctx, 192, RULE_set_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(999);
			match(KW_SET);
			setState(1000);
			match(LEFT_ANG_BRACKET);
			setState(1001);
			simple_type_spec();
			setState(1004);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COMMA) {
				{
				setState(1002);
				match(COMMA);
				setState(1003);
				positive_int_const();
				}
			}

			setState(1006);
			match(RIGHT_ANG_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Map_typeContext extends ParserRuleContext {
		public TerminalNode KW_MAP() { return getToken(IDLParser.KW_MAP, 0); }
		public TerminalNode LEFT_ANG_BRACKET() { return getToken(IDLParser.LEFT_ANG_BRACKET, 0); }
		public List<Simple_type_specContext> simple_type_spec() {
			return getRuleContexts(Simple_type_specContext.class);
		}
		public Simple_type_specContext simple_type_spec(int i) {
			return getRuleContext(Simple_type_specContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public TerminalNode RIGHT_ANG_BRACKET() { return getToken(IDLParser.RIGHT_ANG_BRACKET, 0); }
		public Positive_int_constContext positive_int_const() {
			return getRuleContext(Positive_int_constContext.class,0);
		}
		public Map_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_map_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterMap_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitMap_type(this);
		}
	}

	public final Map_typeContext map_type() throws RecognitionException {
		Map_typeContext _localctx = new Map_typeContext(_ctx, getState());
		enterRule(_localctx, 194, RULE_map_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1008);
			match(KW_MAP);
			setState(1009);
			match(LEFT_ANG_BRACKET);
			setState(1010);
			simple_type_spec();
			setState(1011);
			match(COMMA);
			setState(1012);
			simple_type_spec();
			setState(1015);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COMMA) {
				{
				setState(1013);
				match(COMMA);
				setState(1014);
				positive_int_const();
				}
			}

			setState(1017);
			match(RIGHT_ANG_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class String_typeContext extends ParserRuleContext {
		public TerminalNode KW_STRING() { return getToken(IDLParser.KW_STRING, 0); }
		public TerminalNode LEFT_ANG_BRACKET() { return getToken(IDLParser.LEFT_ANG_BRACKET, 0); }
		public Positive_int_constContext positive_int_const() {
			return getRuleContext(Positive_int_constContext.class,0);
		}
		public TerminalNode RIGHT_ANG_BRACKET() { return getToken(IDLParser.RIGHT_ANG_BRACKET, 0); }
		public String_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_string_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterString_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitString_type(this);
		}
	}

	public final String_typeContext string_type() throws RecognitionException {
		String_typeContext _localctx = new String_typeContext(_ctx, getState());
		enterRule(_localctx, 196, RULE_string_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1019);
			match(KW_STRING);
			setState(1024);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==LEFT_ANG_BRACKET) {
				{
				setState(1020);
				match(LEFT_ANG_BRACKET);
				setState(1021);
				positive_int_const();
				setState(1022);
				match(RIGHT_ANG_BRACKET);
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Wide_string_typeContext extends ParserRuleContext {
		public TerminalNode KW_WSTRING() { return getToken(IDLParser.KW_WSTRING, 0); }
		public TerminalNode LEFT_ANG_BRACKET() { return getToken(IDLParser.LEFT_ANG_BRACKET, 0); }
		public Positive_int_constContext positive_int_const() {
			return getRuleContext(Positive_int_constContext.class,0);
		}
		public TerminalNode RIGHT_ANG_BRACKET() { return getToken(IDLParser.RIGHT_ANG_BRACKET, 0); }
		public Wide_string_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_wide_string_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterWide_string_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitWide_string_type(this);
		}
	}

	public final Wide_string_typeContext wide_string_type() throws RecognitionException {
		Wide_string_typeContext _localctx = new Wide_string_typeContext(_ctx, getState());
		enterRule(_localctx, 198, RULE_wide_string_type);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1026);
			match(KW_WSTRING);
			setState(1031);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==LEFT_ANG_BRACKET) {
				{
				setState(1027);
				match(LEFT_ANG_BRACKET);
				setState(1028);
				positive_int_const();
				setState(1029);
				match(RIGHT_ANG_BRACKET);
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Array_declaratorContext extends ParserRuleContext {
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public List<Fixed_array_sizeContext> fixed_array_size() {
			return getRuleContexts(Fixed_array_sizeContext.class);
		}
		public Fixed_array_sizeContext fixed_array_size(int i) {
			return getRuleContext(Fixed_array_sizeContext.class,i);
		}
		public Array_declaratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_array_declarator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterArray_declarator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitArray_declarator(this);
		}
	}

	public final Array_declaratorContext array_declarator() throws RecognitionException {
		Array_declaratorContext _localctx = new Array_declaratorContext(_ctx, getState());
		enterRule(_localctx, 200, RULE_array_declarator);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1033);
			match(ID);
			setState(1035); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(1034);
				fixed_array_size();
				}
				}
				setState(1037); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==LEFT_SQUARE_BRACKET );
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Fixed_array_sizeContext extends ParserRuleContext {
		public TerminalNode LEFT_SQUARE_BRACKET() { return getToken(IDLParser.LEFT_SQUARE_BRACKET, 0); }
		public Positive_int_constContext positive_int_const() {
			return getRuleContext(Positive_int_constContext.class,0);
		}
		public TerminalNode RIGHT_SQUARE_BRACKET() { return getToken(IDLParser.RIGHT_SQUARE_BRACKET, 0); }
		public Fixed_array_sizeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_fixed_array_size; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterFixed_array_size(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitFixed_array_size(this);
		}
	}

	public final Fixed_array_sizeContext fixed_array_size() throws RecognitionException {
		Fixed_array_sizeContext _localctx = new Fixed_array_sizeContext(_ctx, getState());
		enterRule(_localctx, 202, RULE_fixed_array_size);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1039);
			match(LEFT_SQUARE_BRACKET);
			setState(1040);
			positive_int_const();
			setState(1041);
			match(RIGHT_SQUARE_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attr_declContext extends ParserRuleContext {
		public Readonly_attr_specContext readonly_attr_spec() {
			return getRuleContext(Readonly_attr_specContext.class,0);
		}
		public Attr_specContext attr_spec() {
			return getRuleContext(Attr_specContext.class,0);
		}
		public Attr_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attr_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAttr_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAttr_decl(this);
		}
	}

	public final Attr_declContext attr_decl() throws RecognitionException {
		Attr_declContext _localctx = new Attr_declContext(_ctx, getState());
		enterRule(_localctx, 204, RULE_attr_decl);
		try {
			setState(1045);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_READONLY:
				enterOuterAlt(_localctx, 1);
				{
				setState(1043);
				readonly_attr_spec();
				}
				break;
			case KW_ATTRIBUTE:
				enterOuterAlt(_localctx, 2);
				{
				setState(1044);
				attr_spec();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Except_declContext extends ParserRuleContext {
		public TerminalNode KW_EXCEPTION() { return getToken(IDLParser.KW_EXCEPTION, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<MemberContext> member() {
			return getRuleContexts(MemberContext.class);
		}
		public MemberContext member(int i) {
			return getRuleContext(MemberContext.class,i);
		}
		public Except_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_except_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterExcept_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitExcept_decl(this);
		}
	}

	public final Except_declContext except_decl() throws RecognitionException {
		Except_declContext _localctx = new Except_declContext(_ctx, getState());
		enterRule(_localctx, 206, RULE_except_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1047);
			match(KW_EXCEPTION);
			setState(1048);
			identifier();
			setState(1049);
			match(LEFT_BRACE);
			setState(1053);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -2160953176541364224L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844253269256321L) != 0)) {
				{
				{
				setState(1050);
				member();
				}
				}
				setState(1055);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1056);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Op_declContext extends ParserRuleContext {
		public Op_type_specContext op_type_spec() {
			return getRuleContext(Op_type_specContext.class,0);
		}
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Parameter_declsContext parameter_decls() {
			return getRuleContext(Parameter_declsContext.class,0);
		}
		public Op_attributeContext op_attribute() {
			return getRuleContext(Op_attributeContext.class,0);
		}
		public Raises_exprContext raises_expr() {
			return getRuleContext(Raises_exprContext.class,0);
		}
		public Context_exprContext context_expr() {
			return getRuleContext(Context_exprContext.class,0);
		}
		public Op_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_op_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterOp_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitOp_decl(this);
		}
	}

	public final Op_declContext op_decl() throws RecognitionException {
		Op_declContext _localctx = new Op_declContext(_ctx, getState());
		enterRule(_localctx, 208, RULE_op_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1059);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_ONEWAY) {
				{
				setState(1058);
				op_attribute();
				}
			}

			setState(1061);
			op_type_spec();
			setState(1062);
			identifier();
			setState(1063);
			parameter_decls();
			setState(1065);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_RAISES) {
				{
				setState(1064);
				raises_expr();
				}
			}

			setState(1068);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_CONTEXT) {
				{
				setState(1067);
				context_expr();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Op_attributeContext extends ParserRuleContext {
		public TerminalNode KW_ONEWAY() { return getToken(IDLParser.KW_ONEWAY, 0); }
		public Op_attributeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_op_attribute; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterOp_attribute(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitOp_attribute(this);
		}
	}

	public final Op_attributeContext op_attribute() throws RecognitionException {
		Op_attributeContext _localctx = new Op_attributeContext(_ctx, getState());
		enterRule(_localctx, 210, RULE_op_attribute);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1070);
			match(KW_ONEWAY);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Op_type_specContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Param_type_specContext param_type_spec() {
			return getRuleContext(Param_type_specContext.class,0);
		}
		public TerminalNode KW_VOID() { return getToken(IDLParser.KW_VOID, 0); }
		public Op_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_op_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterOp_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitOp_type_spec(this);
		}
	}

	public final Op_type_specContext op_type_spec() throws RecognitionException {
		Op_type_specContext _localctx = new Op_type_specContext(_ctx, getState());
		enterRule(_localctx, 212, RULE_op_type_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1072);
			annapps();
			setState(1075);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case DOUBLE_COLON:
			case KW_STRING:
			case KW_OCTET:
			case KW_WCHAR:
			case KW_SHORT:
			case KW_LONG:
			case KW_WSTRING:
			case KW_VALUEBASE:
			case KW_OBJECT:
			case KW_UNSIGNED:
			case KW_ANY:
			case KW_CHAR:
			case KW_FLOAT:
			case KW_BOOLEAN:
			case KW_DOUBLE:
			case KW_INT8:
			case KW_UINT8:
			case KW_INT16:
			case KW_UINT16:
			case KW_INT32:
			case KW_UINT32:
			case KW_INT64:
			case KW_UINT64:
			case ID:
				{
				setState(1073);
				param_type_spec();
				}
				break;
			case KW_VOID:
				{
				setState(1074);
				match(KW_VOID);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Parameter_declsContext extends ParserRuleContext {
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public List<Param_declContext> param_decl() {
			return getRuleContexts(Param_declContext.class);
		}
		public Param_declContext param_decl(int i) {
			return getRuleContext(Param_declContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Parameter_declsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_parameter_decls; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterParameter_decls(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitParameter_decls(this);
		}
	}

	public final Parameter_declsContext parameter_decls() throws RecognitionException {
		Parameter_declsContext _localctx = new Parameter_declsContext(_ctx, getState());
		enterRule(_localctx, 214, RULE_parameter_decls);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1077);
			match(LEFT_BRACKET);
			setState(1086);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (((((_la - 35)) & ~0x3f) == 0 && ((1L << (_la - 35)) & 9007199263129605L) != 0)) {
				{
				setState(1078);
				param_decl();
				setState(1083);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==COMMA) {
					{
					{
					setState(1079);
					match(COMMA);
					setState(1080);
					param_decl();
					}
					}
					setState(1085);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
			}

			setState(1088);
			match(RIGHT_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Param_declContext extends ParserRuleContext {
		public List<AnnappsContext> annapps() {
			return getRuleContexts(AnnappsContext.class);
		}
		public AnnappsContext annapps(int i) {
			return getRuleContext(AnnappsContext.class,i);
		}
		public Param_attributeContext param_attribute() {
			return getRuleContext(Param_attributeContext.class,0);
		}
		public Param_type_specContext param_type_spec() {
			return getRuleContext(Param_type_specContext.class,0);
		}
		public Simple_declaratorContext simple_declarator() {
			return getRuleContext(Simple_declaratorContext.class,0);
		}
		public Param_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_param_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterParam_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitParam_decl(this);
		}
	}

	public final Param_declContext param_decl() throws RecognitionException {
		Param_declContext _localctx = new Param_declContext(_ctx, getState());
		enterRule(_localctx, 216, RULE_param_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1090);
			annapps();
			setState(1091);
			param_attribute();
			setState(1092);
			annapps();
			setState(1093);
			param_type_spec();
			setState(1094);
			annapps();
			setState(1095);
			simple_declarator();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Param_attributeContext extends ParserRuleContext {
		public TerminalNode KW_IN() { return getToken(IDLParser.KW_IN, 0); }
		public TerminalNode KW_OUT() { return getToken(IDLParser.KW_OUT, 0); }
		public TerminalNode KW_INOUT() { return getToken(IDLParser.KW_INOUT, 0); }
		public Param_attributeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_param_attribute; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterParam_attribute(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitParam_attribute(this);
		}
	}

	public final Param_attributeContext param_attribute() throws RecognitionException {
		Param_attributeContext _localctx = new Param_attributeContext(_ctx, getState());
		enterRule(_localctx, 218, RULE_param_attribute);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1097);
			_la = _input.LA(1);
			if ( !(((((_la - 37)) & ~0x3f) == 0 && ((1L << (_la - 37)) & 2251799815782401L) != 0)) ) {
			_errHandler.recoverInline(this);
			}
			else {
				if ( _input.LA(1)==Token.EOF ) matchedEOF = true;
				_errHandler.reportMatch(this);
				consume();
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Raises_exprContext extends ParserRuleContext {
		public TerminalNode KW_RAISES() { return getToken(IDLParser.KW_RAISES, 0); }
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public List<A_scoped_nameContext> a_scoped_name() {
			return getRuleContexts(A_scoped_nameContext.class);
		}
		public A_scoped_nameContext a_scoped_name(int i) {
			return getRuleContext(A_scoped_nameContext.class,i);
		}
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Raises_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_raises_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterRaises_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitRaises_expr(this);
		}
	}

	public final Raises_exprContext raises_expr() throws RecognitionException {
		Raises_exprContext _localctx = new Raises_exprContext(_ctx, getState());
		enterRule(_localctx, 220, RULE_raises_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1099);
			match(KW_RAISES);
			setState(1100);
			match(LEFT_BRACKET);
			setState(1101);
			a_scoped_name();
			setState(1106);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(1102);
				match(COMMA);
				setState(1103);
				a_scoped_name();
				}
				}
				setState(1108);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1109);
			match(RIGHT_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Context_exprContext extends ParserRuleContext {
		public TerminalNode KW_CONTEXT() { return getToken(IDLParser.KW_CONTEXT, 0); }
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public List<TerminalNode> STRING_LITERAL() { return getTokens(IDLParser.STRING_LITERAL); }
		public TerminalNode STRING_LITERAL(int i) {
			return getToken(IDLParser.STRING_LITERAL, i);
		}
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Context_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_context_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterContext_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitContext_expr(this);
		}
	}

	public final Context_exprContext context_expr() throws RecognitionException {
		Context_exprContext _localctx = new Context_exprContext(_ctx, getState());
		enterRule(_localctx, 222, RULE_context_expr);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1111);
			match(KW_CONTEXT);
			setState(1112);
			match(LEFT_BRACKET);
			setState(1113);
			match(STRING_LITERAL);
			setState(1118);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(1114);
				match(COMMA);
				setState(1115);
				match(STRING_LITERAL);
				}
				}
				setState(1120);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1121);
			match(RIGHT_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Param_type_specContext extends ParserRuleContext {
		public Base_type_specContext base_type_spec() {
			return getRuleContext(Base_type_specContext.class,0);
		}
		public String_typeContext string_type() {
			return getRuleContext(String_typeContext.class,0);
		}
		public Wide_string_typeContext wide_string_type() {
			return getRuleContext(Wide_string_typeContext.class,0);
		}
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public Param_type_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_param_type_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterParam_type_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitParam_type_spec(this);
		}
	}

	public final Param_type_specContext param_type_spec() throws RecognitionException {
		Param_type_specContext _localctx = new Param_type_specContext(_ctx, getState());
		enterRule(_localctx, 224, RULE_param_type_spec);
		try {
			setState(1127);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_OCTET:
			case KW_WCHAR:
			case KW_SHORT:
			case KW_LONG:
			case KW_VALUEBASE:
			case KW_OBJECT:
			case KW_UNSIGNED:
			case KW_ANY:
			case KW_CHAR:
			case KW_FLOAT:
			case KW_BOOLEAN:
			case KW_DOUBLE:
			case KW_INT8:
			case KW_UINT8:
			case KW_INT16:
			case KW_UINT16:
			case KW_INT32:
			case KW_UINT32:
			case KW_INT64:
			case KW_UINT64:
				enterOuterAlt(_localctx, 1);
				{
				setState(1123);
				base_type_spec();
				}
				break;
			case KW_STRING:
				enterOuterAlt(_localctx, 2);
				{
				setState(1124);
				string_type();
				}
				break;
			case KW_WSTRING:
				enterOuterAlt(_localctx, 3);
				{
				setState(1125);
				wide_string_type();
				}
				break;
			case DOUBLE_COLON:
			case ID:
				enterOuterAlt(_localctx, 4);
				{
				setState(1126);
				scoped_name();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Fixed_pt_typeContext extends ParserRuleContext {
		public TerminalNode KW_FIXED() { return getToken(IDLParser.KW_FIXED, 0); }
		public TerminalNode LEFT_ANG_BRACKET() { return getToken(IDLParser.LEFT_ANG_BRACKET, 0); }
		public List<Positive_int_constContext> positive_int_const() {
			return getRuleContexts(Positive_int_constContext.class);
		}
		public Positive_int_constContext positive_int_const(int i) {
			return getRuleContext(Positive_int_constContext.class,i);
		}
		public TerminalNode COMMA() { return getToken(IDLParser.COMMA, 0); }
		public TerminalNode RIGHT_ANG_BRACKET() { return getToken(IDLParser.RIGHT_ANG_BRACKET, 0); }
		public Fixed_pt_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_fixed_pt_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterFixed_pt_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitFixed_pt_type(this);
		}
	}

	public final Fixed_pt_typeContext fixed_pt_type() throws RecognitionException {
		Fixed_pt_typeContext _localctx = new Fixed_pt_typeContext(_ctx, getState());
		enterRule(_localctx, 226, RULE_fixed_pt_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1129);
			match(KW_FIXED);
			setState(1130);
			match(LEFT_ANG_BRACKET);
			setState(1131);
			positive_int_const();
			setState(1132);
			match(COMMA);
			setState(1133);
			positive_int_const();
			setState(1134);
			match(RIGHT_ANG_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Fixed_pt_const_typeContext extends ParserRuleContext {
		public TerminalNode KW_FIXED() { return getToken(IDLParser.KW_FIXED, 0); }
		public Fixed_pt_const_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_fixed_pt_const_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterFixed_pt_const_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitFixed_pt_const_type(this);
		}
	}

	public final Fixed_pt_const_typeContext fixed_pt_const_type() throws RecognitionException {
		Fixed_pt_const_typeContext _localctx = new Fixed_pt_const_typeContext(_ctx, getState());
		enterRule(_localctx, 228, RULE_fixed_pt_const_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1136);
			match(KW_FIXED);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Value_base_typeContext extends ParserRuleContext {
		public TerminalNode KW_VALUEBASE() { return getToken(IDLParser.KW_VALUEBASE, 0); }
		public Value_base_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_value_base_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterValue_base_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitValue_base_type(this);
		}
	}

	public final Value_base_typeContext value_base_type() throws RecognitionException {
		Value_base_typeContext _localctx = new Value_base_typeContext(_ctx, getState());
		enterRule(_localctx, 230, RULE_value_base_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1138);
			match(KW_VALUEBASE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Constr_forward_declContext extends ParserRuleContext {
		public TerminalNode KW_STRUCT() { return getToken(IDLParser.KW_STRUCT, 0); }
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public TerminalNode KW_UNION() { return getToken(IDLParser.KW_UNION, 0); }
		public Constr_forward_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_constr_forward_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterConstr_forward_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitConstr_forward_decl(this);
		}
	}

	public final Constr_forward_declContext constr_forward_decl() throws RecognitionException {
		Constr_forward_declContext _localctx = new Constr_forward_declContext(_ctx, getState());
		enterRule(_localctx, 232, RULE_constr_forward_decl);
		try {
			setState(1144);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_STRUCT:
				enterOuterAlt(_localctx, 1);
				{
				setState(1140);
				match(KW_STRUCT);
				setState(1141);
				match(ID);
				}
				break;
			case KW_UNION:
				enterOuterAlt(_localctx, 2);
				{
				setState(1142);
				match(KW_UNION);
				setState(1143);
				match(ID);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Import_declContext extends ParserRuleContext {
		public List<AnnappsContext> annapps() {
			return getRuleContexts(AnnappsContext.class);
		}
		public AnnappsContext annapps(int i) {
			return getRuleContext(AnnappsContext.class,i);
		}
		public TerminalNode KW_IMPORT() { return getToken(IDLParser.KW_IMPORT, 0); }
		public Imported_scopeContext imported_scope() {
			return getRuleContext(Imported_scopeContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public Import_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_import_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterImport_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitImport_decl(this);
		}
	}

	public final Import_declContext import_decl() throws RecognitionException {
		Import_declContext _localctx = new Import_declContext(_ctx, getState());
		enterRule(_localctx, 234, RULE_import_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1146);
			annapps();
			setState(1147);
			match(KW_IMPORT);
			setState(1148);
			annapps();
			setState(1149);
			imported_scope();
			setState(1150);
			match(SEMICOLON);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Imported_scopeContext extends ParserRuleContext {
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public TerminalNode STRING_LITERAL() { return getToken(IDLParser.STRING_LITERAL, 0); }
		public Imported_scopeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_imported_scope; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterImported_scope(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitImported_scope(this);
		}
	}

	public final Imported_scopeContext imported_scope() throws RecognitionException {
		Imported_scopeContext _localctx = new Imported_scopeContext(_ctx, getState());
		enterRule(_localctx, 236, RULE_imported_scope);
		try {
			setState(1154);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case DOUBLE_COLON:
			case ID:
				enterOuterAlt(_localctx, 1);
				{
				setState(1152);
				scoped_name();
				}
				break;
			case STRING_LITERAL:
				enterOuterAlt(_localctx, 2);
				{
				setState(1153);
				match(STRING_LITERAL);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Type_id_declContext extends ParserRuleContext {
		public TerminalNode KW_TYPEID() { return getToken(IDLParser.KW_TYPEID, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public TerminalNode STRING_LITERAL() { return getToken(IDLParser.STRING_LITERAL, 0); }
		public Type_id_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type_id_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterType_id_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitType_id_decl(this);
		}
	}

	public final Type_id_declContext type_id_decl() throws RecognitionException {
		Type_id_declContext _localctx = new Type_id_declContext(_ctx, getState());
		enterRule(_localctx, 238, RULE_type_id_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1156);
			match(KW_TYPEID);
			setState(1157);
			a_scoped_name();
			setState(1158);
			match(STRING_LITERAL);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Type_prefix_declContext extends ParserRuleContext {
		public TerminalNode KW_TYPEPREFIX() { return getToken(IDLParser.KW_TYPEPREFIX, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public TerminalNode STRING_LITERAL() { return getToken(IDLParser.STRING_LITERAL, 0); }
		public Type_prefix_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_type_prefix_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterType_prefix_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitType_prefix_decl(this);
		}
	}

	public final Type_prefix_declContext type_prefix_decl() throws RecognitionException {
		Type_prefix_declContext _localctx = new Type_prefix_declContext(_ctx, getState());
		enterRule(_localctx, 240, RULE_type_prefix_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1160);
			match(KW_TYPEPREFIX);
			setState(1161);
			a_scoped_name();
			setState(1162);
			match(STRING_LITERAL);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Readonly_attr_specContext extends ParserRuleContext {
		public TerminalNode KW_READONLY() { return getToken(IDLParser.KW_READONLY, 0); }
		public TerminalNode KW_ATTRIBUTE() { return getToken(IDLParser.KW_ATTRIBUTE, 0); }
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Param_type_specContext param_type_spec() {
			return getRuleContext(Param_type_specContext.class,0);
		}
		public Readonly_attr_declaratorContext readonly_attr_declarator() {
			return getRuleContext(Readonly_attr_declaratorContext.class,0);
		}
		public Readonly_attr_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_readonly_attr_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterReadonly_attr_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitReadonly_attr_spec(this);
		}
	}

	public final Readonly_attr_specContext readonly_attr_spec() throws RecognitionException {
		Readonly_attr_specContext _localctx = new Readonly_attr_specContext(_ctx, getState());
		enterRule(_localctx, 242, RULE_readonly_attr_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1164);
			match(KW_READONLY);
			setState(1165);
			match(KW_ATTRIBUTE);
			setState(1166);
			annapps();
			setState(1167);
			param_type_spec();
			setState(1168);
			readonly_attr_declarator();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Readonly_attr_declaratorContext extends ParserRuleContext {
		public List<AnnappsContext> annapps() {
			return getRuleContexts(AnnappsContext.class);
		}
		public AnnappsContext annapps(int i) {
			return getRuleContext(AnnappsContext.class,i);
		}
		public List<Simple_declaratorContext> simple_declarator() {
			return getRuleContexts(Simple_declaratorContext.class);
		}
		public Simple_declaratorContext simple_declarator(int i) {
			return getRuleContext(Simple_declaratorContext.class,i);
		}
		public Raises_exprContext raises_expr() {
			return getRuleContext(Raises_exprContext.class,0);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Readonly_attr_declaratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_readonly_attr_declarator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterReadonly_attr_declarator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitReadonly_attr_declarator(this);
		}
	}

	public final Readonly_attr_declaratorContext readonly_attr_declarator() throws RecognitionException {
		Readonly_attr_declaratorContext _localctx = new Readonly_attr_declaratorContext(_ctx, getState());
		enterRule(_localctx, 244, RULE_readonly_attr_declarator);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1170);
			annapps();
			setState(1171);
			simple_declarator();
			setState(1182);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_RAISES:
				{
				setState(1172);
				raises_expr();
				}
				break;
			case SEMICOLON:
			case COMMA:
				{
				setState(1179);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==COMMA) {
					{
					{
					setState(1173);
					match(COMMA);
					setState(1174);
					annapps();
					setState(1175);
					simple_declarator();
					}
					}
					setState(1181);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attr_specContext extends ParserRuleContext {
		public TerminalNode KW_ATTRIBUTE() { return getToken(IDLParser.KW_ATTRIBUTE, 0); }
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Param_type_specContext param_type_spec() {
			return getRuleContext(Param_type_specContext.class,0);
		}
		public Attr_declaratorContext attr_declarator() {
			return getRuleContext(Attr_declaratorContext.class,0);
		}
		public Attr_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attr_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAttr_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAttr_spec(this);
		}
	}

	public final Attr_specContext attr_spec() throws RecognitionException {
		Attr_specContext _localctx = new Attr_specContext(_ctx, getState());
		enterRule(_localctx, 246, RULE_attr_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1184);
			match(KW_ATTRIBUTE);
			setState(1185);
			annapps();
			setState(1186);
			param_type_spec();
			setState(1187);
			attr_declarator();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attr_declaratorContext extends ParserRuleContext {
		public List<AnnappsContext> annapps() {
			return getRuleContexts(AnnappsContext.class);
		}
		public AnnappsContext annapps(int i) {
			return getRuleContext(AnnappsContext.class,i);
		}
		public List<Simple_declaratorContext> simple_declarator() {
			return getRuleContexts(Simple_declaratorContext.class);
		}
		public Simple_declaratorContext simple_declarator(int i) {
			return getRuleContext(Simple_declaratorContext.class,i);
		}
		public Attr_raises_exprContext attr_raises_expr() {
			return getRuleContext(Attr_raises_exprContext.class,0);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Attr_declaratorContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attr_declarator; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAttr_declarator(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAttr_declarator(this);
		}
	}

	public final Attr_declaratorContext attr_declarator() throws RecognitionException {
		Attr_declaratorContext _localctx = new Attr_declaratorContext(_ctx, getState());
		enterRule(_localctx, 248, RULE_attr_declarator);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1189);
			annapps();
			setState(1190);
			simple_declarator();
			setState(1201);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_SETRAISES:
			case KW_GETRAISES:
				{
				setState(1191);
				attr_raises_expr();
				}
				break;
			case SEMICOLON:
			case COMMA:
				{
				setState(1198);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==COMMA) {
					{
					{
					setState(1192);
					match(COMMA);
					setState(1193);
					annapps();
					setState(1194);
					simple_declarator();
					}
					}
					setState(1200);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Attr_raises_exprContext extends ParserRuleContext {
		public Get_excep_exprContext get_excep_expr() {
			return getRuleContext(Get_excep_exprContext.class,0);
		}
		public Set_excep_exprContext set_excep_expr() {
			return getRuleContext(Set_excep_exprContext.class,0);
		}
		public Attr_raises_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attr_raises_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAttr_raises_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAttr_raises_expr(this);
		}
	}

	public final Attr_raises_exprContext attr_raises_expr() throws RecognitionException {
		Attr_raises_exprContext _localctx = new Attr_raises_exprContext(_ctx, getState());
		enterRule(_localctx, 250, RULE_attr_raises_expr);
		int _la;
		try {
			setState(1208);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_GETRAISES:
				enterOuterAlt(_localctx, 1);
				{
				setState(1203);
				get_excep_expr();
				setState(1205);
				_errHandler.sync(this);
				_la = _input.LA(1);
				if (_la==KW_SETRAISES) {
					{
					setState(1204);
					set_excep_expr();
					}
				}

				}
				break;
			case KW_SETRAISES:
				enterOuterAlt(_localctx, 2);
				{
				setState(1207);
				set_excep_expr();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Get_excep_exprContext extends ParserRuleContext {
		public TerminalNode KW_GETRAISES() { return getToken(IDLParser.KW_GETRAISES, 0); }
		public Exception_listContext exception_list() {
			return getRuleContext(Exception_listContext.class,0);
		}
		public Get_excep_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_get_excep_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterGet_excep_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitGet_excep_expr(this);
		}
	}

	public final Get_excep_exprContext get_excep_expr() throws RecognitionException {
		Get_excep_exprContext _localctx = new Get_excep_exprContext(_ctx, getState());
		enterRule(_localctx, 252, RULE_get_excep_expr);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1210);
			match(KW_GETRAISES);
			setState(1211);
			exception_list();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Set_excep_exprContext extends ParserRuleContext {
		public TerminalNode KW_SETRAISES() { return getToken(IDLParser.KW_SETRAISES, 0); }
		public Exception_listContext exception_list() {
			return getRuleContext(Exception_listContext.class,0);
		}
		public Set_excep_exprContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_set_excep_expr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSet_excep_expr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSet_excep_expr(this);
		}
	}

	public final Set_excep_exprContext set_excep_expr() throws RecognitionException {
		Set_excep_exprContext _localctx = new Set_excep_exprContext(_ctx, getState());
		enterRule(_localctx, 254, RULE_set_excep_expr);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1213);
			match(KW_SETRAISES);
			setState(1214);
			exception_list();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Exception_listContext extends ParserRuleContext {
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public List<A_scoped_nameContext> a_scoped_name() {
			return getRuleContexts(A_scoped_nameContext.class);
		}
		public A_scoped_nameContext a_scoped_name(int i) {
			return getRuleContext(A_scoped_nameContext.class,i);
		}
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Exception_listContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_exception_list; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterException_list(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitException_list(this);
		}
	}

	public final Exception_listContext exception_list() throws RecognitionException {
		Exception_listContext _localctx = new Exception_listContext(_ctx, getState());
		enterRule(_localctx, 256, RULE_exception_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1216);
			match(LEFT_BRACKET);
			setState(1217);
			a_scoped_name();
			setState(1222);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(1218);
				match(COMMA);
				setState(1219);
				a_scoped_name();
				}
				}
				setState(1224);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1225);
			match(RIGHT_BRACKET);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ComponentContext extends ParserRuleContext {
		public Component_declContext component_decl() {
			return getRuleContext(Component_declContext.class,0);
		}
		public Component_forward_declContext component_forward_decl() {
			return getRuleContext(Component_forward_declContext.class,0);
		}
		public ComponentContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_component; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComponent(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComponent(this);
		}
	}

	public final ComponentContext component() throws RecognitionException {
		ComponentContext _localctx = new ComponentContext(_ctx, getState());
		enterRule(_localctx, 258, RULE_component);
		try {
			setState(1229);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,98,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(1227);
				component_decl();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(1228);
				component_forward_decl();
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Component_forward_declContext extends ParserRuleContext {
		public TerminalNode KW_COMPONENT() { return getToken(IDLParser.KW_COMPONENT, 0); }
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public Component_forward_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_component_forward_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComponent_forward_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComponent_forward_decl(this);
		}
	}

	public final Component_forward_declContext component_forward_decl() throws RecognitionException {
		Component_forward_declContext _localctx = new Component_forward_declContext(_ctx, getState());
		enterRule(_localctx, 260, RULE_component_forward_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1231);
			match(KW_COMPONENT);
			setState(1232);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Component_declContext extends ParserRuleContext {
		public Component_headerContext component_header() {
			return getRuleContext(Component_headerContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public Component_bodyContext component_body() {
			return getRuleContext(Component_bodyContext.class,0);
		}
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public Component_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_component_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComponent_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComponent_decl(this);
		}
	}

	public final Component_declContext component_decl() throws RecognitionException {
		Component_declContext _localctx = new Component_declContext(_ctx, getState());
		enterRule(_localctx, 262, RULE_component_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1234);
			component_header();
			setState(1235);
			match(LEFT_BRACE);
			setState(1236);
			component_body();
			setState(1237);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Component_headerContext extends ParserRuleContext {
		public TerminalNode KW_COMPONENT() { return getToken(IDLParser.KW_COMPONENT, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Component_inheritance_specContext component_inheritance_spec() {
			return getRuleContext(Component_inheritance_specContext.class,0);
		}
		public Supported_interface_specContext supported_interface_spec() {
			return getRuleContext(Supported_interface_specContext.class,0);
		}
		public Component_headerContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_component_header; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComponent_header(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComponent_header(this);
		}
	}

	public final Component_headerContext component_header() throws RecognitionException {
		Component_headerContext _localctx = new Component_headerContext(_ctx, getState());
		enterRule(_localctx, 264, RULE_component_header);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1239);
			match(KW_COMPONENT);
			setState(1240);
			identifier();
			setState(1242);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COLON) {
				{
				setState(1241);
				component_inheritance_spec();
				}
			}

			setState(1245);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_SUPPORTS) {
				{
				setState(1244);
				supported_interface_spec();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Supported_interface_specContext extends ParserRuleContext {
		public TerminalNode KW_SUPPORTS() { return getToken(IDLParser.KW_SUPPORTS, 0); }
		public List<A_scoped_nameContext> a_scoped_name() {
			return getRuleContexts(A_scoped_nameContext.class);
		}
		public A_scoped_nameContext a_scoped_name(int i) {
			return getRuleContext(A_scoped_nameContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Supported_interface_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_supported_interface_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterSupported_interface_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitSupported_interface_spec(this);
		}
	}

	public final Supported_interface_specContext supported_interface_spec() throws RecognitionException {
		Supported_interface_specContext _localctx = new Supported_interface_specContext(_ctx, getState());
		enterRule(_localctx, 266, RULE_supported_interface_spec);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1247);
			match(KW_SUPPORTS);
			setState(1248);
			a_scoped_name();
			setState(1253);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==COMMA) {
				{
				{
				setState(1249);
				match(COMMA);
				setState(1250);
				a_scoped_name();
				}
				}
				setState(1255);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Component_inheritance_specContext extends ParserRuleContext {
		public TerminalNode COLON() { return getToken(IDLParser.COLON, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public Component_inheritance_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_component_inheritance_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComponent_inheritance_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComponent_inheritance_spec(this);
		}
	}

	public final Component_inheritance_specContext component_inheritance_spec() throws RecognitionException {
		Component_inheritance_specContext _localctx = new Component_inheritance_specContext(_ctx, getState());
		enterRule(_localctx, 268, RULE_component_inheritance_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1256);
			match(COLON);
			setState(1257);
			a_scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Component_bodyContext extends ParserRuleContext {
		public List<Component_exportContext> component_export() {
			return getRuleContexts(Component_exportContext.class);
		}
		public Component_exportContext component_export(int i) {
			return getRuleContext(Component_exportContext.class,i);
		}
		public Component_bodyContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_component_body; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComponent_body(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComponent_body(this);
		}
	}

	public final Component_bodyContext component_body() throws RecognitionException {
		Component_bodyContext _localctx = new Component_bodyContext(_ctx, getState());
		enterRule(_localctx, 270, RULE_component_body);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1262);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (((((_la - 35)) & ~0x3f) == 0 && ((1L << (_la - 35)) & 630503947831935305L) != 0)) {
				{
				{
				setState(1259);
				component_export();
				}
				}
				setState(1264);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Component_exportContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Provides_declContext provides_decl() {
			return getRuleContext(Provides_declContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public Uses_declContext uses_decl() {
			return getRuleContext(Uses_declContext.class,0);
		}
		public Emits_declContext emits_decl() {
			return getRuleContext(Emits_declContext.class,0);
		}
		public Publishes_declContext publishes_decl() {
			return getRuleContext(Publishes_declContext.class,0);
		}
		public Consumes_declContext consumes_decl() {
			return getRuleContext(Consumes_declContext.class,0);
		}
		public Attr_declContext attr_decl() {
			return getRuleContext(Attr_declContext.class,0);
		}
		public Component_exportContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_component_export; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterComponent_export(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitComponent_export(this);
		}
	}

	public final Component_exportContext component_export() throws RecognitionException {
		Component_exportContext _localctx = new Component_exportContext(_ctx, getState());
		enterRule(_localctx, 272, RULE_component_export);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1265);
			annapps();
			setState(1284);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case KW_PROVIDES:
				{
				setState(1266);
				provides_decl();
				setState(1267);
				match(SEMICOLON);
				}
				break;
			case KW_USES:
				{
				setState(1269);
				uses_decl();
				setState(1270);
				match(SEMICOLON);
				}
				break;
			case KW_EMITS:
				{
				setState(1272);
				emits_decl();
				setState(1273);
				match(SEMICOLON);
				}
				break;
			case KW_PUBLISHES:
				{
				setState(1275);
				publishes_decl();
				setState(1276);
				match(SEMICOLON);
				}
				break;
			case KW_CONSUMES:
				{
				setState(1278);
				consumes_decl();
				setState(1279);
				match(SEMICOLON);
				}
				break;
			case KW_READONLY:
			case KW_ATTRIBUTE:
				{
				setState(1281);
				attr_decl();
				setState(1282);
				match(SEMICOLON);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Provides_declContext extends ParserRuleContext {
		public TerminalNode KW_PROVIDES() { return getToken(IDLParser.KW_PROVIDES, 0); }
		public Interface_typeContext interface_type() {
			return getRuleContext(Interface_typeContext.class,0);
		}
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public Provides_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_provides_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterProvides_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitProvides_decl(this);
		}
	}

	public final Provides_declContext provides_decl() throws RecognitionException {
		Provides_declContext _localctx = new Provides_declContext(_ctx, getState());
		enterRule(_localctx, 274, RULE_provides_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1286);
			match(KW_PROVIDES);
			setState(1287);
			interface_type();
			setState(1288);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Interface_typeContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public TerminalNode KW_OBJECT() { return getToken(IDLParser.KW_OBJECT, 0); }
		public Interface_typeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_interface_type; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterInterface_type(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitInterface_type(this);
		}
	}

	public final Interface_typeContext interface_type() throws RecognitionException {
		Interface_typeContext _localctx = new Interface_typeContext(_ctx, getState());
		enterRule(_localctx, 276, RULE_interface_type);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1290);
			annapps();
			setState(1293);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case DOUBLE_COLON:
			case ID:
				{
				setState(1291);
				scoped_name();
				}
				break;
			case KW_OBJECT:
				{
				setState(1292);
				match(KW_OBJECT);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Uses_declContext extends ParserRuleContext {
		public TerminalNode KW_USES() { return getToken(IDLParser.KW_USES, 0); }
		public Interface_typeContext interface_type() {
			return getRuleContext(Interface_typeContext.class,0);
		}
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public TerminalNode KW_MULTIPLE() { return getToken(IDLParser.KW_MULTIPLE, 0); }
		public Uses_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_uses_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterUses_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitUses_decl(this);
		}
	}

	public final Uses_declContext uses_decl() throws RecognitionException {
		Uses_declContext _localctx = new Uses_declContext(_ctx, getState());
		enterRule(_localctx, 278, RULE_uses_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1295);
			match(KW_USES);
			setState(1297);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_MULTIPLE) {
				{
				setState(1296);
				match(KW_MULTIPLE);
				}
			}

			setState(1299);
			interface_type();
			setState(1300);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Emits_declContext extends ParserRuleContext {
		public TerminalNode KW_EMITS() { return getToken(IDLParser.KW_EMITS, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public Emits_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_emits_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEmits_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEmits_decl(this);
		}
	}

	public final Emits_declContext emits_decl() throws RecognitionException {
		Emits_declContext _localctx = new Emits_declContext(_ctx, getState());
		enterRule(_localctx, 280, RULE_emits_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1302);
			match(KW_EMITS);
			setState(1303);
			a_scoped_name();
			setState(1304);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Publishes_declContext extends ParserRuleContext {
		public TerminalNode KW_PUBLISHES() { return getToken(IDLParser.KW_PUBLISHES, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public Publishes_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_publishes_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterPublishes_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitPublishes_decl(this);
		}
	}

	public final Publishes_declContext publishes_decl() throws RecognitionException {
		Publishes_declContext _localctx = new Publishes_declContext(_ctx, getState());
		enterRule(_localctx, 282, RULE_publishes_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1306);
			match(KW_PUBLISHES);
			setState(1307);
			a_scoped_name();
			setState(1308);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Consumes_declContext extends ParserRuleContext {
		public TerminalNode KW_CONSUMES() { return getToken(IDLParser.KW_CONSUMES, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public Consumes_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_consumes_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterConsumes_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitConsumes_decl(this);
		}
	}

	public final Consumes_declContext consumes_decl() throws RecognitionException {
		Consumes_declContext _localctx = new Consumes_declContext(_ctx, getState());
		enterRule(_localctx, 284, RULE_consumes_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1310);
			match(KW_CONSUMES);
			setState(1311);
			a_scoped_name();
			setState(1312);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Home_declContext extends ParserRuleContext {
		public Home_headerContext home_header() {
			return getRuleContext(Home_headerContext.class,0);
		}
		public Home_bodyContext home_body() {
			return getRuleContext(Home_bodyContext.class,0);
		}
		public Home_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_home_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterHome_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitHome_decl(this);
		}
	}

	public final Home_declContext home_decl() throws RecognitionException {
		Home_declContext _localctx = new Home_declContext(_ctx, getState());
		enterRule(_localctx, 286, RULE_home_decl);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1314);
			home_header();
			setState(1315);
			home_body();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Home_headerContext extends ParserRuleContext {
		public TerminalNode KW_HOME() { return getToken(IDLParser.KW_HOME, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode KW_MANAGES() { return getToken(IDLParser.KW_MANAGES, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public Home_inheritance_specContext home_inheritance_spec() {
			return getRuleContext(Home_inheritance_specContext.class,0);
		}
		public Supported_interface_specContext supported_interface_spec() {
			return getRuleContext(Supported_interface_specContext.class,0);
		}
		public Primary_key_specContext primary_key_spec() {
			return getRuleContext(Primary_key_specContext.class,0);
		}
		public Home_headerContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_home_header; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterHome_header(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitHome_header(this);
		}
	}

	public final Home_headerContext home_header() throws RecognitionException {
		Home_headerContext _localctx = new Home_headerContext(_ctx, getState());
		enterRule(_localctx, 288, RULE_home_header);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1317);
			match(KW_HOME);
			setState(1318);
			identifier();
			setState(1320);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==COLON) {
				{
				setState(1319);
				home_inheritance_spec();
				}
			}

			setState(1323);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_SUPPORTS) {
				{
				setState(1322);
				supported_interface_spec();
				}
			}

			setState(1325);
			match(KW_MANAGES);
			setState(1326);
			a_scoped_name();
			setState(1328);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_PRIMARYKEY) {
				{
				setState(1327);
				primary_key_spec();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Home_inheritance_specContext extends ParserRuleContext {
		public TerminalNode COLON() { return getToken(IDLParser.COLON, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public Home_inheritance_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_home_inheritance_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterHome_inheritance_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitHome_inheritance_spec(this);
		}
	}

	public final Home_inheritance_specContext home_inheritance_spec() throws RecognitionException {
		Home_inheritance_specContext _localctx = new Home_inheritance_specContext(_ctx, getState());
		enterRule(_localctx, 290, RULE_home_inheritance_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1330);
			match(COLON);
			setState(1331);
			a_scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Primary_key_specContext extends ParserRuleContext {
		public TerminalNode KW_PRIMARYKEY() { return getToken(IDLParser.KW_PRIMARYKEY, 0); }
		public A_scoped_nameContext a_scoped_name() {
			return getRuleContext(A_scoped_nameContext.class,0);
		}
		public Primary_key_specContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_primary_key_spec; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterPrimary_key_spec(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitPrimary_key_spec(this);
		}
	}

	public final Primary_key_specContext primary_key_spec() throws RecognitionException {
		Primary_key_specContext _localctx = new Primary_key_specContext(_ctx, getState());
		enterRule(_localctx, 292, RULE_primary_key_spec);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1333);
			match(KW_PRIMARYKEY);
			setState(1334);
			a_scoped_name();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Home_bodyContext extends ParserRuleContext {
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<Home_exportContext> home_export() {
			return getRuleContexts(Home_exportContext.class);
		}
		public Home_exportContext home_export(int i) {
			return getRuleContext(Home_exportContext.class,i);
		}
		public Home_bodyContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_home_body; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterHome_body(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitHome_body(this);
		}
	}

	public final Home_bodyContext home_body() throws RecognitionException {
		Home_bodyContext _localctx = new Home_bodyContext(_ctx, getState());
		enterRule(_localctx, 294, RULE_home_body);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1336);
			match(LEFT_BRACE);
			setState(1340);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -2135193818125828096L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844152069138649L) != 0)) {
				{
				{
				setState(1337);
				home_export();
				}
				}
				setState(1342);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1343);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Home_exportContext extends ParserRuleContext {
		public Export_Context export_() {
			return getRuleContext(Export_Context.class,0);
		}
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public TerminalNode SEMICOLON() { return getToken(IDLParser.SEMICOLON, 0); }
		public Factory_declContext factory_decl() {
			return getRuleContext(Factory_declContext.class,0);
		}
		public Finder_declContext finder_decl() {
			return getRuleContext(Finder_declContext.class,0);
		}
		public Home_exportContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_home_export; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterHome_export(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitHome_export(this);
		}
	}

	public final Home_exportContext home_export() throws RecognitionException {
		Home_exportContext _localctx = new Home_exportContext(_ctx, getState());
		enterRule(_localctx, 296, RULE_home_export);
		try {
			setState(1353);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,111,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(1345);
				export_();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(1346);
				annapps();
				setState(1349);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case KW_FACTORY:
					{
					setState(1347);
					factory_decl();
					}
					break;
				case KW_FINDER:
					{
					setState(1348);
					finder_decl();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				setState(1351);
				match(SEMICOLON);
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Factory_declContext extends ParserRuleContext {
		public TerminalNode KW_FACTORY() { return getToken(IDLParser.KW_FACTORY, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public Init_param_declsContext init_param_decls() {
			return getRuleContext(Init_param_declsContext.class,0);
		}
		public Raises_exprContext raises_expr() {
			return getRuleContext(Raises_exprContext.class,0);
		}
		public Factory_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_factory_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterFactory_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitFactory_decl(this);
		}
	}

	public final Factory_declContext factory_decl() throws RecognitionException {
		Factory_declContext _localctx = new Factory_declContext(_ctx, getState());
		enterRule(_localctx, 298, RULE_factory_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1355);
			match(KW_FACTORY);
			setState(1356);
			identifier();
			setState(1357);
			match(LEFT_BRACKET);
			setState(1359);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==AT || _la==KW_IN) {
				{
				setState(1358);
				init_param_decls();
				}
			}

			setState(1361);
			match(RIGHT_BRACKET);
			setState(1363);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_RAISES) {
				{
				setState(1362);
				raises_expr();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Finder_declContext extends ParserRuleContext {
		public TerminalNode KW_FINDER() { return getToken(IDLParser.KW_FINDER, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public Init_param_declsContext init_param_decls() {
			return getRuleContext(Init_param_declsContext.class,0);
		}
		public Raises_exprContext raises_expr() {
			return getRuleContext(Raises_exprContext.class,0);
		}
		public Finder_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_finder_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterFinder_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitFinder_decl(this);
		}
	}

	public final Finder_declContext finder_decl() throws RecognitionException {
		Finder_declContext _localctx = new Finder_declContext(_ctx, getState());
		enterRule(_localctx, 300, RULE_finder_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1365);
			match(KW_FINDER);
			setState(1366);
			identifier();
			setState(1367);
			match(LEFT_BRACKET);
			setState(1369);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==AT || _la==KW_IN) {
				{
				setState(1368);
				init_param_decls();
				}
			}

			setState(1371);
			match(RIGHT_BRACKET);
			setState(1373);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_RAISES) {
				{
				setState(1372);
				raises_expr();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class EventContext extends ParserRuleContext {
		public Event_declContext event_decl() {
			return getRuleContext(Event_declContext.class,0);
		}
		public Event_abs_declContext event_abs_decl() {
			return getRuleContext(Event_abs_declContext.class,0);
		}
		public Event_forward_declContext event_forward_decl() {
			return getRuleContext(Event_forward_declContext.class,0);
		}
		public EventContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_event; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEvent(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEvent(this);
		}
	}

	public final EventContext event() throws RecognitionException {
		EventContext _localctx = new EventContext(_ctx, getState());
		enterRule(_localctx, 302, RULE_event);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1378);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,116,_ctx) ) {
			case 1:
				{
				setState(1375);
				event_decl();
				}
				break;
			case 2:
				{
				setState(1376);
				event_abs_decl();
				}
				break;
			case 3:
				{
				setState(1377);
				event_forward_decl();
				}
				break;
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Event_forward_declContext extends ParserRuleContext {
		public TerminalNode KW_EVENTTYPE() { return getToken(IDLParser.KW_EVENTTYPE, 0); }
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public TerminalNode KW_ABSTRACT() { return getToken(IDLParser.KW_ABSTRACT, 0); }
		public Event_forward_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_event_forward_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEvent_forward_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEvent_forward_decl(this);
		}
	}

	public final Event_forward_declContext event_forward_decl() throws RecognitionException {
		Event_forward_declContext _localctx = new Event_forward_declContext(_ctx, getState());
		enterRule(_localctx, 304, RULE_event_forward_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1381);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_ABSTRACT) {
				{
				setState(1380);
				match(KW_ABSTRACT);
				}
			}

			setState(1383);
			match(KW_EVENTTYPE);
			setState(1384);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Event_abs_declContext extends ParserRuleContext {
		public TerminalNode KW_ABSTRACT() { return getToken(IDLParser.KW_ABSTRACT, 0); }
		public TerminalNode KW_EVENTTYPE() { return getToken(IDLParser.KW_EVENTTYPE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Value_inheritance_specContext value_inheritance_spec() {
			return getRuleContext(Value_inheritance_specContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<Export_Context> export_() {
			return getRuleContexts(Export_Context.class);
		}
		public Export_Context export_(int i) {
			return getRuleContext(Export_Context.class,i);
		}
		public Event_abs_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_event_abs_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEvent_abs_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEvent_abs_decl(this);
		}
	}

	public final Event_abs_declContext event_abs_decl() throws RecognitionException {
		Event_abs_declContext _localctx = new Event_abs_declContext(_ctx, getState());
		enterRule(_localctx, 306, RULE_event_abs_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1386);
			match(KW_ABSTRACT);
			setState(1387);
			match(KW_EVENTTYPE);
			setState(1388);
			identifier();
			setState(1389);
			value_inheritance_spec();
			setState(1390);
			match(LEFT_BRACE);
			setState(1394);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -2139697417753198592L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844152069138641L) != 0)) {
				{
				{
				setState(1391);
				export_();
				}
				}
				setState(1396);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1397);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Event_declContext extends ParserRuleContext {
		public Event_headerContext event_header() {
			return getRuleContext(Event_headerContext.class,0);
		}
		public TerminalNode LEFT_BRACE() { return getToken(IDLParser.LEFT_BRACE, 0); }
		public TerminalNode RIGHT_BRACE() { return getToken(IDLParser.RIGHT_BRACE, 0); }
		public List<Value_elementContext> value_element() {
			return getRuleContexts(Value_elementContext.class);
		}
		public Value_elementContext value_element(int i) {
			return getRuleContext(Value_elementContext.class,i);
		}
		public Event_declContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_event_decl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEvent_decl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEvent_decl(this);
		}
	}

	public final Event_declContext event_decl() throws RecognitionException {
		Event_declContext _localctx = new Event_declContext(_ctx, getState());
		enterRule(_localctx, 308, RULE_event_decl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1399);
			event_header();
			setState(1400);
			match(LEFT_BRACE);
			setState(1404);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & -950747116127387648L) != 0) || ((((_la - 64)) & ~0x3f) == 0 && ((1L << (_la - 64)) & 844152069138649L) != 0)) {
				{
				{
				setState(1401);
				value_element();
				}
				}
				setState(1406);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(1407);
			match(RIGHT_BRACE);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Event_headerContext extends ParserRuleContext {
		public TerminalNode KW_EVENTTYPE() { return getToken(IDLParser.KW_EVENTTYPE, 0); }
		public IdentifierContext identifier() {
			return getRuleContext(IdentifierContext.class,0);
		}
		public Value_inheritance_specContext value_inheritance_spec() {
			return getRuleContext(Value_inheritance_specContext.class,0);
		}
		public TerminalNode KW_CUSTOM() { return getToken(IDLParser.KW_CUSTOM, 0); }
		public Event_headerContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_event_header; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterEvent_header(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitEvent_header(this);
		}
	}

	public final Event_headerContext event_header() throws RecognitionException {
		Event_headerContext _localctx = new Event_headerContext(_ctx, getState());
		enterRule(_localctx, 310, RULE_event_header);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1410);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==KW_CUSTOM) {
				{
				setState(1409);
				match(KW_CUSTOM);
				}
			}

			setState(1412);
			match(KW_EVENTTYPE);
			setState(1413);
			identifier();
			setState(1414);
			value_inheritance_spec();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class AnnappsContext extends ParserRuleContext {
		public List<Annotation_applContext> annotation_appl() {
			return getRuleContexts(Annotation_applContext.class);
		}
		public Annotation_applContext annotation_appl(int i) {
			return getRuleContext(Annotation_applContext.class,i);
		}
		public AnnappsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annapps; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnapps(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnapps(this);
		}
	}

	public final AnnappsContext annapps() throws RecognitionException {
		AnnappsContext _localctx = new AnnappsContext(_ctx, getState());
		enterRule(_localctx, 312, RULE_annapps);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(1419);
			_errHandler.sync(this);
			_alt = getInterpreter().adaptivePredict(_input,121,_ctx);
			while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER ) {
				if ( _alt==1 ) {
					{
					{
					setState(1416);
					annotation_appl();
					}
					} 
				}
				setState(1421);
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,121,_ctx);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_applContext extends ParserRuleContext {
		public TerminalNode AT() { return getToken(IDLParser.AT, 0); }
		public Scoped_nameContext scoped_name() {
			return getRuleContext(Scoped_nameContext.class,0);
		}
		public TerminalNode LEFT_BRACKET() { return getToken(IDLParser.LEFT_BRACKET, 0); }
		public Annotation_appl_paramsContext annotation_appl_params() {
			return getRuleContext(Annotation_appl_paramsContext.class,0);
		}
		public TerminalNode RIGHT_BRACKET() { return getToken(IDLParser.RIGHT_BRACKET, 0); }
		public Annotation_applContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_appl; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_appl(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_appl(this);
		}
	}

	public final Annotation_applContext annotation_appl() throws RecognitionException {
		Annotation_applContext _localctx = new Annotation_applContext(_ctx, getState());
		enterRule(_localctx, 314, RULE_annotation_appl);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1422);
			match(AT);
			setState(1423);
			scoped_name();
			setState(1428);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==LEFT_BRACKET) {
				{
				setState(1424);
				match(LEFT_BRACKET);
				setState(1425);
				annotation_appl_params();
				setState(1426);
				match(RIGHT_BRACKET);
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_appl_paramsContext extends ParserRuleContext {
		public Const_expContext const_exp() {
			return getRuleContext(Const_expContext.class,0);
		}
		public List<Annotation_appl_paramContext> annotation_appl_param() {
			return getRuleContexts(Annotation_appl_paramContext.class);
		}
		public Annotation_appl_paramContext annotation_appl_param(int i) {
			return getRuleContext(Annotation_appl_paramContext.class,i);
		}
		public List<TerminalNode> COMMA() { return getTokens(IDLParser.COMMA); }
		public TerminalNode COMMA(int i) {
			return getToken(IDLParser.COMMA, i);
		}
		public Annotation_appl_paramsContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_appl_params; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_appl_params(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_appl_params(this);
		}
	}

	public final Annotation_appl_paramsContext annotation_appl_params() throws RecognitionException {
		Annotation_appl_paramsContext _localctx = new Annotation_appl_paramsContext(_ctx, getState());
		enterRule(_localctx, 316, RULE_annotation_appl_params);
		int _la;
		try {
			setState(1439);
			_errHandler.sync(this);
			switch ( getInterpreter().adaptivePredict(_input,124,_ctx) ) {
			case 1:
				enterOuterAlt(_localctx, 1);
				{
				setState(1430);
				const_exp();
				}
				break;
			case 2:
				enterOuterAlt(_localctx, 2);
				{
				setState(1431);
				annotation_appl_param();
				setState(1436);
				_errHandler.sync(this);
				_la = _input.LA(1);
				while (_la==COMMA) {
					{
					{
					setState(1432);
					match(COMMA);
					setState(1433);
					annotation_appl_param();
					}
					}
					setState(1438);
					_errHandler.sync(this);
					_la = _input.LA(1);
				}
				}
				break;
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class Annotation_appl_paramContext extends ParserRuleContext {
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public TerminalNode EQUAL() { return getToken(IDLParser.EQUAL, 0); }
		public Const_expContext const_exp() {
			return getRuleContext(Const_expContext.class,0);
		}
		public Annotation_appl_paramContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_annotation_appl_param; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterAnnotation_appl_param(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitAnnotation_appl_param(this);
		}
	}

	public final Annotation_appl_paramContext annotation_appl_param() throws RecognitionException {
		Annotation_appl_paramContext _localctx = new Annotation_appl_paramContext(_ctx, getState());
		enterRule(_localctx, 318, RULE_annotation_appl_param);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1441);
			match(ID);
			setState(1442);
			match(EQUAL);
			setState(1443);
			const_exp();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class IdentifierContext extends ParserRuleContext {
		public AnnappsContext annapps() {
			return getRuleContext(AnnappsContext.class,0);
		}
		public TerminalNode ID() { return getToken(IDLParser.ID, 0); }
		public IdentifierContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_identifier; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).enterIdentifier(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof IDLListener ) ((IDLListener)listener).exitIdentifier(this);
		}
	}

	public final IdentifierContext identifier() throws RecognitionException {
		IdentifierContext _localctx = new IdentifierContext(_ctx, getState());
		enterRule(_localctx, 320, RULE_identifier);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(1445);
			annapps();
			setState(1446);
			match(ID);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static final String _serializedATN =
		"\u0004\u0001t\u05a9\u0002\u0000\u0007\u0000\u0002\u0001\u0007\u0001\u0002"+
		"\u0002\u0007\u0002\u0002\u0003\u0007\u0003\u0002\u0004\u0007\u0004\u0002"+
		"\u0005\u0007\u0005\u0002\u0006\u0007\u0006\u0002\u0007\u0007\u0007\u0002"+
		"\b\u0007\b\u0002\t\u0007\t\u0002\n\u0007\n\u0002\u000b\u0007\u000b\u0002"+
		"\f\u0007\f\u0002\r\u0007\r\u0002\u000e\u0007\u000e\u0002\u000f\u0007\u000f"+
		"\u0002\u0010\u0007\u0010\u0002\u0011\u0007\u0011\u0002\u0012\u0007\u0012"+
		"\u0002\u0013\u0007\u0013\u0002\u0014\u0007\u0014\u0002\u0015\u0007\u0015"+
		"\u0002\u0016\u0007\u0016\u0002\u0017\u0007\u0017\u0002\u0018\u0007\u0018"+
		"\u0002\u0019\u0007\u0019\u0002\u001a\u0007\u001a\u0002\u001b\u0007\u001b"+
		"\u0002\u001c\u0007\u001c\u0002\u001d\u0007\u001d\u0002\u001e\u0007\u001e"+
		"\u0002\u001f\u0007\u001f\u0002 \u0007 \u0002!\u0007!\u0002\"\u0007\"\u0002"+
		"#\u0007#\u0002$\u0007$\u0002%\u0007%\u0002&\u0007&\u0002\'\u0007\'\u0002"+
		"(\u0007(\u0002)\u0007)\u0002*\u0007*\u0002+\u0007+\u0002,\u0007,\u0002"+
		"-\u0007-\u0002.\u0007.\u0002/\u0007/\u00020\u00070\u00021\u00071\u0002"+
		"2\u00072\u00023\u00073\u00024\u00074\u00025\u00075\u00026\u00076\u0002"+
		"7\u00077\u00028\u00078\u00029\u00079\u0002:\u0007:\u0002;\u0007;\u0002"+
		"<\u0007<\u0002=\u0007=\u0002>\u0007>\u0002?\u0007?\u0002@\u0007@\u0002"+
		"A\u0007A\u0002B\u0007B\u0002C\u0007C\u0002D\u0007D\u0002E\u0007E\u0002"+
		"F\u0007F\u0002G\u0007G\u0002H\u0007H\u0002I\u0007I\u0002J\u0007J\u0002"+
		"K\u0007K\u0002L\u0007L\u0002M\u0007M\u0002N\u0007N\u0002O\u0007O\u0002"+
		"P\u0007P\u0002Q\u0007Q\u0002R\u0007R\u0002S\u0007S\u0002T\u0007T\u0002"+
		"U\u0007U\u0002V\u0007V\u0002W\u0007W\u0002X\u0007X\u0002Y\u0007Y\u0002"+
		"Z\u0007Z\u0002[\u0007[\u0002\\\u0007\\\u0002]\u0007]\u0002^\u0007^\u0002"+
		"_\u0007_\u0002`\u0007`\u0002a\u0007a\u0002b\u0007b\u0002c\u0007c\u0002"+
		"d\u0007d\u0002e\u0007e\u0002f\u0007f\u0002g\u0007g\u0002h\u0007h\u0002"+
		"i\u0007i\u0002j\u0007j\u0002k\u0007k\u0002l\u0007l\u0002m\u0007m\u0002"+
		"n\u0007n\u0002o\u0007o\u0002p\u0007p\u0002q\u0007q\u0002r\u0007r\u0002"+
		"s\u0007s\u0002t\u0007t\u0002u\u0007u\u0002v\u0007v\u0002w\u0007w\u0002"+
		"x\u0007x\u0002y\u0007y\u0002z\u0007z\u0002{\u0007{\u0002|\u0007|\u0002"+
		"}\u0007}\u0002~\u0007~\u0002\u007f\u0007\u007f\u0002\u0080\u0007\u0080"+
		"\u0002\u0081\u0007\u0081\u0002\u0082\u0007\u0082\u0002\u0083\u0007\u0083"+
		"\u0002\u0084\u0007\u0084\u0002\u0085\u0007\u0085\u0002\u0086\u0007\u0086"+
		"\u0002\u0087\u0007\u0087\u0002\u0088\u0007\u0088\u0002\u0089\u0007\u0089"+
		"\u0002\u008a\u0007\u008a\u0002\u008b\u0007\u008b\u0002\u008c\u0007\u008c"+
		"\u0002\u008d\u0007\u008d\u0002\u008e\u0007\u008e\u0002\u008f\u0007\u008f"+
		"\u0002\u0090\u0007\u0090\u0002\u0091\u0007\u0091\u0002\u0092\u0007\u0092"+
		"\u0002\u0093\u0007\u0093\u0002\u0094\u0007\u0094\u0002\u0095\u0007\u0095"+
		"\u0002\u0096\u0007\u0096\u0002\u0097\u0007\u0097\u0002\u0098\u0007\u0098"+
		"\u0002\u0099\u0007\u0099\u0002\u009a\u0007\u009a\u0002\u009b\u0007\u009b"+
		"\u0002\u009c\u0007\u009c\u0002\u009d\u0007\u009d\u0002\u009e\u0007\u009e"+
		"\u0002\u009f\u0007\u009f\u0002\u00a0\u0007\u00a0\u0001\u0000\u0005\u0000"+
		"\u0144\b\u0000\n\u0000\f\u0000\u0147\t\u0000\u0001\u0000\u0004\u0000\u014a"+
		"\b\u0000\u000b\u0000\f\u0000\u014b\u0001\u0000\u0001\u0000\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0003\u0001\u0175\b\u0001\u0001\u0002\u0001\u0002\u0001\u0002\u0001\u0002"+
		"\u0004\u0002\u017b\b\u0002\u000b\u0002\f\u0002\u017c\u0001\u0002\u0001"+
		"\u0002\u0001\u0003\u0001\u0003\u0001\u0003\u0003\u0003\u0184\b\u0003\u0001"+
		"\u0004\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0005\u0003"+
		"\u0005\u018c\b\u0005\u0001\u0005\u0001\u0005\u0001\u0005\u0001\u0006\u0003"+
		"\u0006\u0192\b\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0003\u0006\u0197"+
		"\b\u0006\u0001\u0007\u0005\u0007\u019a\b\u0007\n\u0007\f\u0007\u019d\t"+
		"\u0007\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b"+
		"\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001"+
		"\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0003\b\u01b5\b\b\u0001\t\u0001"+
		"\t\u0001\t\u0001\t\u0005\t\u01bb\b\t\n\t\f\t\u01be\t\t\u0001\n\u0001\n"+
		"\u0001\u000b\u0001\u000b\u0001\u000b\u0001\f\u0003\f\u01c6\b\f\u0001\f"+
		"\u0001\f\u0001\f\u0005\f\u01cb\b\f\n\f\f\f\u01ce\t\f\u0001\r\u0001\r\u0001"+
		"\r\u0001\r\u0001\r\u0003\r\u01d5\b\r\u0001\u000e\u0003\u000e\u01d8\b\u000e"+
		"\u0001\u000e\u0001\u000e\u0001\u000e\u0001\u000f\u0001\u000f\u0001\u000f"+
		"\u0001\u000f\u0001\u0010\u0001\u0010\u0001\u0010\u0001\u0010\u0001\u0010"+
		"\u0001\u0010\u0005\u0010\u01e7\b\u0010\n\u0010\f\u0010\u01ea\t\u0010\u0001"+
		"\u0010\u0001\u0010\u0001\u0011\u0001\u0011\u0001\u0011\u0005\u0011\u01f1"+
		"\b\u0011\n\u0011\f\u0011\u01f4\t\u0011\u0001\u0011\u0001\u0011\u0001\u0012"+
		"\u0003\u0012\u01f9\b\u0012\u0001\u0012\u0001\u0012\u0001\u0012\u0001\u0012"+
		"\u0001\u0013\u0001\u0013\u0003\u0013\u0201\b\u0013\u0001\u0013\u0001\u0013"+
		"\u0001\u0013\u0005\u0013\u0206\b\u0013\n\u0013\f\u0013\u0209\t\u0013\u0003"+
		"\u0013\u020b\b\u0013\u0001\u0013\u0001\u0013\u0001\u0013\u0001\u0013\u0005"+
		"\u0013\u0211\b\u0013\n\u0013\f\u0013\u0214\t\u0013\u0003\u0013\u0216\b"+
		"\u0013\u0001\u0014\u0001\u0014\u0001\u0015\u0001\u0015\u0001\u0015\u0003"+
		"\u0015\u021d\b\u0015\u0001\u0016\u0001\u0016\u0001\u0016\u0001\u0016\u0001"+
		"\u0016\u0003\u0016\u0224\b\u0016\u0001\u0016\u0001\u0016\u0001\u0016\u0001"+
		"\u0016\u0001\u0017\u0001\u0017\u0001\u0017\u0001\u0017\u0001\u0017\u0003"+
		"\u0017\u022f\b\u0017\u0001\u0017\u0001\u0017\u0003\u0017\u0233\b\u0017"+
		"\u0001\u0017\u0001\u0017\u0001\u0018\u0001\u0018\u0001\u0018\u0005\u0018"+
		"\u023a\b\u0018\n\u0018\f\u0018\u023d\t\u0018\u0001\u0019\u0001\u0019\u0001"+
		"\u0019\u0001\u0019\u0001\u0019\u0001\u0019\u0001\u0019\u0001\u001a\u0001"+
		"\u001a\u0001\u001b\u0001\u001b\u0001\u001b\u0001\u001b\u0001\u001b\u0001"+
		"\u001b\u0001\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0001"+
		"\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0001\u001c\u0003"+
		"\u001c\u0259\b\u001c\u0001\u001d\u0001\u001d\u0001\u001e\u0001\u001e\u0001"+
		"\u001e\u0005\u001e\u0260\b\u001e\n\u001e\f\u001e\u0263\t\u001e\u0001\u001f"+
		"\u0001\u001f\u0001\u001f\u0005\u001f\u0268\b\u001f\n\u001f\f\u001f\u026b"+
		"\t\u001f\u0001 \u0001 \u0001 \u0005 \u0270\b \n \f \u0273\t \u0001!\u0001"+
		"!\u0001!\u0005!\u0278\b!\n!\f!\u027b\t!\u0001\"\u0001\"\u0001\"\u0005"+
		"\"\u0280\b\"\n\"\f\"\u0283\t\"\u0001#\u0001#\u0001#\u0005#\u0288\b#\n"+
		"#\f#\u028b\t#\u0001$\u0001$\u0001$\u0001$\u0003$\u0291\b$\u0001%\u0001"+
		"%\u0001&\u0001&\u0001&\u0001&\u0001&\u0001&\u0003&\u029b\b&\u0001\'\u0001"+
		"\'\u0001(\u0001(\u0001)\u0001)\u0001)\u0001)\u0001)\u0001)\u0001)\u0001"+
		")\u0001)\u0001)\u0001)\u0001)\u0001)\u0001)\u0003)\u02af\b)\u0001*\u0001"+
		"*\u0001*\u0001+\u0001+\u0003+\u02b6\b+\u0001,\u0001,\u0001,\u0003,\u02bb"+
		"\b,\u0001-\u0001-\u0001-\u0003-\u02c0\b-\u0001.\u0001.\u0001.\u0001.\u0001"+
		".\u0001.\u0001.\u0001.\u0001.\u0003.\u02cb\b.\u0001/\u0001/\u0001/\u0001"+
		"/\u0001/\u0001/\u0003/\u02d3\b/\u00010\u00010\u00010\u00010\u00010\u0003"+
		"0\u02da\b0\u00011\u00011\u00011\u00051\u02df\b1\n1\f1\u02e2\t1\u00012"+
		"\u00012\u00012\u00052\u02e7\b2\n2\f2\u02ea\t2\u00013\u00013\u00013\u0003"+
		"3\u02ef\b3\u00014\u00014\u00015\u00015\u00016\u00016\u00016\u00016\u0003"+
		"6\u02f9\b6\u00017\u00017\u00037\u02fd\b7\u00018\u00018\u00018\u00018\u0003"+
		"8\u0303\b8\u00019\u00019\u0001:\u0001:\u0001;\u0001;\u0001<\u0001<\u0001"+
		"<\u0003<\u030e\b<\u0001=\u0001=\u0001=\u0001=\u0003=\u0314\b=\u0001>\u0001"+
		">\u0001?\u0001?\u0001?\u0003?\u031b\b?\u0001@\u0001@\u0001@\u0003@\u0320"+
		"\b@\u0001A\u0001A\u0001A\u0001A\u0003A\u0326\bA\u0001B\u0001B\u0001C\u0001"+
		"C\u0001D\u0001D\u0001E\u0001E\u0001F\u0001F\u0001G\u0001G\u0001H\u0001"+
		"H\u0003H\u0336\bH\u0001I\u0001I\u0001I\u0001I\u0001I\u0001J\u0001J\u0001"+
		"J\u0003J\u0340\bJ\u0001K\u0001K\u0001K\u0001L\u0001L\u0001L\u0001L\u0001"+
		"L\u0001L\u0001L\u0001L\u0001L\u0001L\u0001L\u0005L\u0350\bL\nL\fL\u0353"+
		"\tL\u0001M\u0001M\u0001M\u0001M\u0003M\u0359\bM\u0001M\u0001M\u0001N\u0001"+
		"N\u0001N\u0001O\u0001O\u0001O\u0001O\u0003O\u0364\bO\u0001O\u0001O\u0001"+
		"O\u0001O\u0001P\u0001P\u0003P\u036c\bP\u0001P\u0001P\u0004P\u0370\bP\u000b"+
		"P\fP\u0371\u0001Q\u0001Q\u0001Q\u0001Q\u0001Q\u0001Q\u0003Q\u037a\bQ\u0001"+
		"Q\u0001Q\u0001R\u0001R\u0001R\u0001R\u0001R\u0001R\u0001S\u0001S\u0001"+
		"S\u0005S\u0387\bS\nS\fS\u038a\tS\u0001T\u0001T\u0001T\u0001T\u0003T\u0390"+
		"\bT\u0001T\u0001T\u0001T\u0001T\u0001U\u0005U\u0397\bU\nU\fU\u039a\tU"+
		"\u0001V\u0001V\u0001V\u0001V\u0001V\u0001W\u0001W\u0001W\u0001W\u0001"+
		"W\u0001W\u0001W\u0001W\u0001W\u0001W\u0001W\u0001X\u0001X\u0001X\u0001"+
		"X\u0001X\u0001X\u0001X\u0003X\u03b3\bX\u0001Y\u0004Y\u03b6\bY\u000bY\f"+
		"Y\u03b7\u0001Z\u0004Z\u03bb\bZ\u000bZ\fZ\u03bc\u0001Z\u0001Z\u0001Z\u0001"+
		"[\u0001[\u0001[\u0001[\u0001[\u0001[\u0001[\u0003[\u03c9\b[\u0001\\\u0001"+
		"\\\u0001\\\u0001\\\u0001]\u0001]\u0001]\u0001]\u0001]\u0001]\u0005]\u03d5"+
		"\b]\n]\f]\u03d8\t]\u0001]\u0001]\u0001^\u0001^\u0001_\u0001_\u0001_\u0001"+
		"_\u0001_\u0001_\u0003_\u03e4\b_\u0001_\u0001_\u0001`\u0001`\u0001`\u0001"+
		"`\u0001`\u0003`\u03ed\b`\u0001`\u0001`\u0001a\u0001a\u0001a\u0001a\u0001"+
		"a\u0001a\u0001a\u0003a\u03f8\ba\u0001a\u0001a\u0001b\u0001b\u0001b\u0001"+
		"b\u0001b\u0003b\u0401\bb\u0001c\u0001c\u0001c\u0001c\u0001c\u0003c\u0408"+
		"\bc\u0001d\u0001d\u0004d\u040c\bd\u000bd\fd\u040d\u0001e\u0001e\u0001"+
		"e\u0001e\u0001f\u0001f\u0003f\u0416\bf\u0001g\u0001g\u0001g\u0001g\u0005"+
		"g\u041c\bg\ng\fg\u041f\tg\u0001g\u0001g\u0001h\u0003h\u0424\bh\u0001h"+
		"\u0001h\u0001h\u0001h\u0003h\u042a\bh\u0001h\u0003h\u042d\bh\u0001i\u0001"+
		"i\u0001j\u0001j\u0001j\u0003j\u0434\bj\u0001k\u0001k\u0001k\u0001k\u0005"+
		"k\u043a\bk\nk\fk\u043d\tk\u0003k\u043f\bk\u0001k\u0001k\u0001l\u0001l"+
		"\u0001l\u0001l\u0001l\u0001l\u0001l\u0001m\u0001m\u0001n\u0001n\u0001"+
		"n\u0001n\u0001n\u0005n\u0451\bn\nn\fn\u0454\tn\u0001n\u0001n\u0001o\u0001"+
		"o\u0001o\u0001o\u0001o\u0005o\u045d\bo\no\fo\u0460\to\u0001o\u0001o\u0001"+
		"p\u0001p\u0001p\u0001p\u0003p\u0468\bp\u0001q\u0001q\u0001q\u0001q\u0001"+
		"q\u0001q\u0001q\u0001r\u0001r\u0001s\u0001s\u0001t\u0001t\u0001t\u0001"+
		"t\u0003t\u0479\bt\u0001u\u0001u\u0001u\u0001u\u0001u\u0001u\u0001v\u0001"+
		"v\u0003v\u0483\bv\u0001w\u0001w\u0001w\u0001w\u0001x\u0001x\u0001x\u0001"+
		"x\u0001y\u0001y\u0001y\u0001y\u0001y\u0001y\u0001z\u0001z\u0001z\u0001"+
		"z\u0001z\u0001z\u0001z\u0005z\u049a\bz\nz\fz\u049d\tz\u0003z\u049f\bz"+
		"\u0001{\u0001{\u0001{\u0001{\u0001{\u0001|\u0001|\u0001|\u0001|\u0001"+
		"|\u0001|\u0001|\u0005|\u04ad\b|\n|\f|\u04b0\t|\u0003|\u04b2\b|\u0001}"+
		"\u0001}\u0003}\u04b6\b}\u0001}\u0003}\u04b9\b}\u0001~\u0001~\u0001~\u0001"+
		"\u007f\u0001\u007f\u0001\u007f\u0001\u0080\u0001\u0080\u0001\u0080\u0001"+
		"\u0080\u0005\u0080\u04c5\b\u0080\n\u0080\f\u0080\u04c8\t\u0080\u0001\u0080"+
		"\u0001\u0080\u0001\u0081\u0001\u0081\u0003\u0081\u04ce\b\u0081\u0001\u0082"+
		"\u0001\u0082\u0001\u0082\u0001\u0083\u0001\u0083\u0001\u0083\u0001\u0083"+
		"\u0001\u0083\u0001\u0084\u0001\u0084\u0001\u0084\u0003\u0084\u04db\b\u0084"+
		"\u0001\u0084\u0003\u0084\u04de\b\u0084\u0001\u0085\u0001\u0085\u0001\u0085"+
		"\u0001\u0085\u0005\u0085\u04e4\b\u0085\n\u0085\f\u0085\u04e7\t\u0085\u0001"+
		"\u0086\u0001\u0086\u0001\u0086\u0001\u0087\u0005\u0087\u04ed\b\u0087\n"+
		"\u0087\f\u0087\u04f0\t\u0087\u0001\u0088\u0001\u0088\u0001\u0088\u0001"+
		"\u0088\u0001\u0088\u0001\u0088\u0001\u0088\u0001\u0088\u0001\u0088\u0001"+
		"\u0088\u0001\u0088\u0001\u0088\u0001\u0088\u0001\u0088\u0001\u0088\u0001"+
		"\u0088\u0001\u0088\u0001\u0088\u0001\u0088\u0003\u0088\u0505\b\u0088\u0001"+
		"\u0089\u0001\u0089\u0001\u0089\u0001\u0089\u0001\u008a\u0001\u008a\u0001"+
		"\u008a\u0003\u008a\u050e\b\u008a\u0001\u008b\u0001\u008b\u0003\u008b\u0512"+
		"\b\u008b\u0001\u008b\u0001\u008b\u0001\u008b\u0001\u008c\u0001\u008c\u0001"+
		"\u008c\u0001\u008c\u0001\u008d\u0001\u008d\u0001\u008d\u0001\u008d\u0001"+
		"\u008e\u0001\u008e\u0001\u008e\u0001\u008e\u0001\u008f\u0001\u008f\u0001"+
		"\u008f\u0001\u0090\u0001\u0090\u0001\u0090\u0003\u0090\u0529\b\u0090\u0001"+
		"\u0090\u0003\u0090\u052c\b\u0090\u0001\u0090\u0001\u0090\u0001\u0090\u0003"+
		"\u0090\u0531\b\u0090\u0001\u0091\u0001\u0091\u0001\u0091\u0001\u0092\u0001"+
		"\u0092\u0001\u0092\u0001\u0093\u0001\u0093\u0005\u0093\u053b\b\u0093\n"+
		"\u0093\f\u0093\u053e\t\u0093\u0001\u0093\u0001\u0093\u0001\u0094\u0001"+
		"\u0094\u0001\u0094\u0001\u0094\u0003\u0094\u0546\b\u0094\u0001\u0094\u0001"+
		"\u0094\u0003\u0094\u054a\b\u0094\u0001\u0095\u0001\u0095\u0001\u0095\u0001"+
		"\u0095\u0003\u0095\u0550\b\u0095\u0001\u0095\u0001\u0095\u0003\u0095\u0554"+
		"\b\u0095\u0001\u0096\u0001\u0096\u0001\u0096\u0001\u0096\u0003\u0096\u055a"+
		"\b\u0096\u0001\u0096\u0001\u0096\u0003\u0096\u055e\b\u0096\u0001\u0097"+
		"\u0001\u0097\u0001\u0097\u0003\u0097\u0563\b\u0097\u0001\u0098\u0003\u0098"+
		"\u0566\b\u0098\u0001\u0098\u0001\u0098\u0001\u0098\u0001\u0099\u0001\u0099"+
		"\u0001\u0099\u0001\u0099\u0001\u0099\u0001\u0099\u0005\u0099\u0571\b\u0099"+
		"\n\u0099\f\u0099\u0574\t\u0099\u0001\u0099\u0001\u0099\u0001\u009a\u0001"+
		"\u009a\u0001\u009a\u0005\u009a\u057b\b\u009a\n\u009a\f\u009a\u057e\t\u009a"+
		"\u0001\u009a\u0001\u009a\u0001\u009b\u0003\u009b\u0583\b\u009b\u0001\u009b"+
		"\u0001\u009b\u0001\u009b\u0001\u009b\u0001\u009c\u0005\u009c\u058a\b\u009c"+
		"\n\u009c\f\u009c\u058d\t\u009c\u0001\u009d\u0001\u009d\u0001\u009d\u0001"+
		"\u009d\u0001\u009d\u0001\u009d\u0003\u009d\u0595\b\u009d\u0001\u009e\u0001"+
		"\u009e\u0001\u009e\u0001\u009e\u0005\u009e\u059b\b\u009e\n\u009e\f\u009e"+
		"\u059e\t\u009e\u0003\u009e\u05a0\b\u009e\u0001\u009f\u0001\u009f\u0001"+
		"\u009f\u0001\u009f\u0001\u00a0\u0001\u00a0\u0001\u00a0\u0001\u00a0\u0000"+
		"\u0000\u00a1\u0000\u0002\u0004\u0006\b\n\f\u000e\u0010\u0012\u0014\u0016"+
		"\u0018\u001a\u001c\u001e \"$&(*,.02468:<>@BDFHJLNPRTVXZ\\^`bdfhjlnprt"+
		"vxz|~\u0080\u0082\u0084\u0086\u0088\u008a\u008c\u008e\u0090\u0092\u0094"+
		"\u0096\u0098\u009a\u009c\u009e\u00a0\u00a2\u00a4\u00a6\u00a8\u00aa\u00ac"+
		"\u00ae\u00b0\u00b2\u00b4\u00b6\u00b8\u00ba\u00bc\u00be\u00c0\u00c2\u00c4"+
		"\u00c6\u00c8\u00ca\u00cc\u00ce\u00d0\u00d2\u00d4\u00d6\u00d8\u00da\u00dc"+
		"\u00de\u00e0\u00e2\u00e4\u00e6\u00e8\u00ea\u00ec\u00ee\u00f0\u00f2\u00f4"+
		"\u00f6\u00f8\u00fa\u00fc\u00fe\u0100\u0102\u0104\u0106\u0108\u010a\u010c"+
		"\u010e\u0110\u0112\u0114\u0116\u0118\u011a\u011c\u011e\u0120\u0122\u0124"+
		"\u0126\u0128\u012a\u012c\u012e\u0130\u0132\u0134\u0136\u0138\u013a\u013c"+
		"\u013e\u0140\u0000\t\u0002\u0000WW__\u0001\u0000!\"\u0001\u0000\u0019"+
		"\u001a\u0003\u0000\u0015\u0015\u0018\u0018\u001f\u001f\u0002\u0000\u0014"+
		"\u0014\u0019\u001a\u0001\u0000\u0001\n\u0002\u0000==jj\u0002\u0000>>l"+
		"l\u0003\u0000%%::XX\u05c8\u0000\u0145\u0001\u0000\u0000\u0000\u0002\u014f"+
		"\u0001\u0000\u0000\u0000\u0004\u0176\u0001\u0000\u0000\u0000\u0006\u0180"+
		"\u0001\u0000\u0000\u0000\b\u0185\u0001\u0000\u0000\u0000\n\u018b\u0001"+
		"\u0000\u0000\u0000\f\u0191\u0001\u0000\u0000\u0000\u000e\u019b\u0001\u0000"+
		"\u0000\u0000\u0010\u019e\u0001\u0000\u0000\u0000\u0012\u01b6\u0001\u0000"+
		"\u0000\u0000\u0014\u01bf\u0001\u0000\u0000\u0000\u0016\u01c1\u0001\u0000"+
		"\u0000\u0000\u0018\u01c5\u0001\u0000\u0000\u0000\u001a\u01cf\u0001\u0000"+
		"\u0000\u0000\u001c\u01d7\u0001\u0000\u0000\u0000\u001e\u01dc\u0001\u0000"+
		"\u0000\u0000 \u01e0\u0001\u0000\u0000\u0000\"\u01ed\u0001\u0000\u0000"+
		"\u0000$\u01f8\u0001\u0000\u0000\u0000&\u020a\u0001\u0000\u0000\u0000("+
		"\u0217\u0001\u0000\u0000\u0000*\u021c\u0001\u0000\u0000\u0000,\u021e\u0001"+
		"\u0000\u0000\u0000.\u0229\u0001\u0000\u0000\u00000\u0236\u0001\u0000\u0000"+
		"\u00002\u023e\u0001\u0000\u0000\u00004\u0245\u0001\u0000\u0000\u00006"+
		"\u0247\u0001\u0000\u0000\u00008\u024d\u0001\u0000\u0000\u0000:\u025a\u0001"+
		"\u0000\u0000\u0000<\u025c\u0001\u0000\u0000\u0000>\u0264\u0001\u0000\u0000"+
		"\u0000@\u026c\u0001\u0000\u0000\u0000B\u0274\u0001\u0000\u0000\u0000D"+
		"\u027c\u0001\u0000\u0000\u0000F\u0284\u0001\u0000\u0000\u0000H\u0290\u0001"+
		"\u0000\u0000\u0000J\u0292\u0001\u0000\u0000\u0000L\u029a\u0001\u0000\u0000"+
		"\u0000N\u029c\u0001\u0000\u0000\u0000P\u029e\u0001\u0000\u0000\u0000R"+
		"\u02ae\u0001\u0000\u0000\u0000T\u02b0\u0001\u0000\u0000\u0000V\u02b5\u0001"+
		"\u0000\u0000\u0000X\u02ba\u0001\u0000\u0000\u0000Z\u02bf\u0001\u0000\u0000"+
		"\u0000\\\u02ca\u0001\u0000\u0000\u0000^\u02d2\u0001\u0000\u0000\u0000"+
		"`\u02d9\u0001\u0000\u0000\u0000b\u02db\u0001\u0000\u0000\u0000d\u02e3"+
		"\u0001\u0000\u0000\u0000f\u02eb\u0001\u0000\u0000\u0000h\u02f0\u0001\u0000"+
		"\u0000\u0000j\u02f2\u0001\u0000\u0000\u0000l\u02f8\u0001\u0000\u0000\u0000"+
		"n\u02fc\u0001\u0000\u0000\u0000p\u0302\u0001\u0000\u0000\u0000r\u0304"+
		"\u0001\u0000\u0000\u0000t\u0306\u0001\u0000\u0000\u0000v\u0308\u0001\u0000"+
		"\u0000\u0000x\u030d\u0001\u0000\u0000\u0000z\u0313\u0001\u0000\u0000\u0000"+
		"|\u0315\u0001\u0000\u0000\u0000~\u031a\u0001\u0000\u0000\u0000\u0080\u031f"+
		"\u0001\u0000\u0000\u0000\u0082\u0325\u0001\u0000\u0000\u0000\u0084\u0327"+
		"\u0001\u0000\u0000\u0000\u0086\u0329\u0001\u0000\u0000\u0000\u0088\u032b"+
		"\u0001\u0000\u0000\u0000\u008a\u032d\u0001\u0000\u0000\u0000\u008c\u032f"+
		"\u0001\u0000\u0000\u0000\u008e\u0331\u0001\u0000\u0000\u0000\u0090\u0335"+
		"\u0001\u0000\u0000\u0000\u0092\u0337\u0001\u0000\u0000\u0000\u0094\u033c"+
		"\u0001\u0000\u0000\u0000\u0096\u0341\u0001\u0000\u0000\u0000\u0098\u0351"+
		"\u0001\u0000\u0000\u0000\u009a\u0354\u0001\u0000\u0000\u0000\u009c\u035c"+
		"\u0001\u0000\u0000\u0000\u009e\u035f\u0001\u0000\u0000\u0000\u00a0\u036f"+
		"\u0001\u0000\u0000\u0000\u00a2\u0373\u0001\u0000\u0000\u0000\u00a4\u037d"+
		"\u0001\u0000\u0000\u0000\u00a6\u0383\u0001\u0000\u0000\u0000\u00a8\u038b"+
		"\u0001\u0000\u0000\u0000\u00aa\u0398\u0001\u0000\u0000\u0000\u00ac\u039b"+
		"\u0001\u0000\u0000\u0000\u00ae\u03a0\u0001\u0000\u0000\u0000\u00b0\u03b2"+
		"\u0001\u0000\u0000\u0000\u00b2\u03b5\u0001\u0000\u0000\u0000\u00b4\u03ba"+
		"\u0001\u0000\u0000\u0000\u00b6\u03c1\u0001\u0000\u0000\u0000\u00b8\u03ca"+
		"\u0001\u0000\u0000\u0000\u00ba\u03ce\u0001\u0000\u0000\u0000\u00bc\u03db"+
		"\u0001\u0000\u0000\u0000\u00be\u03dd\u0001\u0000\u0000\u0000\u00c0\u03e7"+
		"\u0001\u0000\u0000\u0000\u00c2\u03f0\u0001\u0000\u0000\u0000\u00c4\u03fb"+
		"\u0001\u0000\u0000\u0000\u00c6\u0402\u0001\u0000\u0000\u0000\u00c8\u0409"+
		"\u0001\u0000\u0000\u0000\u00ca\u040f\u0001\u0000\u0000\u0000\u00cc\u0415"+
		"\u0001\u0000\u0000\u0000\u00ce\u0417\u0001\u0000\u0000\u0000\u00d0\u0423"+
		"\u0001\u0000\u0000\u0000\u00d2\u042e\u0001\u0000\u0000\u0000\u00d4\u0430"+
		"\u0001\u0000\u0000\u0000\u00d6\u0435\u0001\u0000\u0000\u0000\u00d8\u0442"+
		"\u0001\u0000\u0000\u0000\u00da\u0449\u0001\u0000\u0000\u0000\u00dc\u044b"+
		"\u0001\u0000\u0000\u0000\u00de\u0457\u0001\u0000\u0000\u0000\u00e0\u0467"+
		"\u0001\u0000\u0000\u0000\u00e2\u0469\u0001\u0000\u0000\u0000\u00e4\u0470"+
		"\u0001\u0000\u0000\u0000\u00e6\u0472\u0001\u0000\u0000\u0000\u00e8\u0478"+
		"\u0001\u0000\u0000\u0000\u00ea\u047a\u0001\u0000\u0000\u0000\u00ec\u0482"+
		"\u0001\u0000\u0000\u0000\u00ee\u0484\u0001\u0000\u0000\u0000\u00f0\u0488"+
		"\u0001\u0000\u0000\u0000\u00f2\u048c\u0001\u0000\u0000\u0000\u00f4\u0492"+
		"\u0001\u0000\u0000\u0000\u00f6\u04a0\u0001\u0000\u0000\u0000\u00f8\u04a5"+
		"\u0001\u0000\u0000\u0000\u00fa\u04b8\u0001\u0000\u0000\u0000\u00fc\u04ba"+
		"\u0001\u0000\u0000\u0000\u00fe\u04bd\u0001\u0000\u0000\u0000\u0100\u04c0"+
		"\u0001\u0000\u0000\u0000\u0102\u04cd\u0001\u0000\u0000\u0000\u0104\u04cf"+
		"\u0001\u0000\u0000\u0000\u0106\u04d2\u0001\u0000\u0000\u0000\u0108\u04d7"+
		"\u0001\u0000\u0000\u0000\u010a\u04df\u0001\u0000\u0000\u0000\u010c\u04e8"+
		"\u0001\u0000\u0000\u0000\u010e\u04ee\u0001\u0000\u0000\u0000\u0110\u04f1"+
		"\u0001\u0000\u0000\u0000\u0112\u0506\u0001\u0000\u0000\u0000\u0114\u050a"+
		"\u0001\u0000\u0000\u0000\u0116\u050f\u0001\u0000\u0000\u0000\u0118\u0516"+
		"\u0001\u0000\u0000\u0000\u011a\u051a\u0001\u0000\u0000\u0000\u011c\u051e"+
		"\u0001\u0000\u0000\u0000\u011e\u0522\u0001\u0000\u0000\u0000\u0120\u0525"+
		"\u0001\u0000\u0000\u0000\u0122\u0532\u0001\u0000\u0000\u0000\u0124\u0535"+
		"\u0001\u0000\u0000\u0000\u0126\u0538\u0001\u0000\u0000\u0000\u0128\u0549"+
		"\u0001\u0000\u0000\u0000\u012a\u054b\u0001\u0000\u0000\u0000\u012c\u0555"+
		"\u0001\u0000\u0000\u0000\u012e\u0562\u0001\u0000\u0000\u0000\u0130\u0565"+
		"\u0001\u0000\u0000\u0000\u0132\u056a\u0001\u0000\u0000\u0000\u0134\u0577"+
		"\u0001\u0000\u0000\u0000\u0136\u0582\u0001\u0000\u0000\u0000\u0138\u058b"+
		"\u0001\u0000\u0000\u0000\u013a\u058e\u0001\u0000\u0000\u0000\u013c\u059f"+
		"\u0001\u0000\u0000\u0000\u013e\u05a1\u0001\u0000\u0000\u0000\u0140\u05a5"+
		"\u0001\u0000\u0000\u0000\u0142\u0144\u0003\u00eau\u0000\u0143\u0142\u0001"+
		"\u0000\u0000\u0000\u0144\u0147\u0001\u0000\u0000\u0000\u0145\u0143\u0001"+
		"\u0000\u0000\u0000\u0145\u0146\u0001\u0000\u0000\u0000\u0146\u0149\u0001"+
		"\u0000\u0000\u0000\u0147\u0145\u0001\u0000\u0000\u0000\u0148\u014a\u0003"+
		"\u0002\u0001\u0000\u0149\u0148\u0001\u0000\u0000\u0000\u014a\u014b\u0001"+
		"\u0000\u0000\u0000\u014b\u0149\u0001\u0000\u0000\u0000\u014b\u014c\u0001"+
		"\u0000\u0000\u0000\u014c\u014d\u0001\u0000\u0000\u0000\u014d\u014e\u0005"+
		"\u0000\u0000\u0001\u014e\u0001\u0001\u0000\u0000\u0000\u014f\u0174\u0003"+
		"\u0138\u009c\u0000\u0150\u0151\u0003R)\u0000\u0151\u0152\u0005\u000b\u0000"+
		"\u0000\u0152\u0175\u0001\u0000\u0000\u0000\u0153\u0154\u00036\u001b\u0000"+
		"\u0154\u0155\u0005\u000b\u0000\u0000\u0155\u0175\u0001\u0000\u0000\u0000"+
		"\u0156\u0157\u0003\u00ceg\u0000\u0157\u0158\u0005\u000b\u0000\u0000\u0158"+
		"\u0175\u0001\u0000\u0000\u0000\u0159\u015a\u0003\u0006\u0003\u0000\u015a"+
		"\u015b\u0005\u000b\u0000\u0000\u015b\u0175\u0001\u0000\u0000\u0000\u015c"+
		"\u015d\u0003\u0004\u0002\u0000\u015d\u015e\u0005\u000b\u0000\u0000\u015e"+
		"\u0175\u0001\u0000\u0000\u0000\u015f\u0160\u0003\u001a\r\u0000\u0160\u0161"+
		"\u0005\u000b\u0000\u0000\u0161\u0175\u0001\u0000\u0000\u0000\u0162\u0163"+
		"\u0003\u00eew\u0000\u0163\u0164\u0005\u000b\u0000\u0000\u0164\u0175\u0001"+
		"\u0000\u0000\u0000\u0165\u0166\u0003\u00f0x\u0000\u0166\u0167\u0005\u000b"+
		"\u0000\u0000\u0167\u0175\u0001\u0000\u0000\u0000\u0168\u0169\u0003\u012e"+
		"\u0097\u0000\u0169\u016a\u0005\u000b\u0000\u0000\u016a\u0175\u0001\u0000"+
		"\u0000\u0000\u016b\u016c\u0003\u0102\u0081\u0000\u016c\u016d\u0005\u000b"+
		"\u0000\u0000\u016d\u0175\u0001\u0000\u0000\u0000\u016e\u016f\u0003\u011e"+
		"\u008f\u0000\u016f\u0170\u0005\u000b\u0000\u0000\u0170\u0175\u0001\u0000"+
		"\u0000\u0000\u0171\u0172\u0003\u0090H\u0000\u0172\u0173\u0005\u000b\u0000"+
		"\u0000\u0173\u0175\u0001\u0000\u0000\u0000\u0174\u0150\u0001\u0000\u0000"+
		"\u0000\u0174\u0153\u0001\u0000\u0000\u0000\u0174\u0156\u0001\u0000\u0000"+
		"\u0000\u0174\u0159\u0001\u0000\u0000\u0000\u0174\u015c\u0001\u0000\u0000"+
		"\u0000\u0174\u015f\u0001\u0000\u0000\u0000\u0174\u0162\u0001\u0000\u0000"+
		"\u0000\u0174\u0165\u0001\u0000\u0000\u0000\u0174\u0168\u0001\u0000\u0000"+
		"\u0000\u0174\u016b\u0001\u0000\u0000\u0000\u0174\u016e\u0001\u0000\u0000"+
		"\u0000\u0174\u0171\u0001\u0000\u0000\u0000\u0175\u0003\u0001\u0000\u0000"+
		"\u0000\u0176\u0177\u0005J\u0000\u0000\u0177\u0178\u0003\u0140\u00a0\u0000"+
		"\u0178\u017a\u0005\u000e\u0000\u0000\u0179\u017b\u0003\u0002\u0001\u0000"+
		"\u017a\u0179\u0001\u0000\u0000\u0000\u017b\u017c\u0001\u0000\u0000\u0000"+
		"\u017c\u017a\u0001\u0000\u0000\u0000\u017c\u017d\u0001\u0000\u0000\u0000"+
		"\u017d\u017e\u0001\u0000\u0000\u0000\u017e\u017f\u0005\u000f\u0000\u0000"+
		"\u017f\u0005\u0001\u0000\u0000\u0000\u0180\u0183\u0003\u0138\u009c\u0000"+
		"\u0181\u0184\u0003\b\u0004\u0000\u0182\u0184\u0003\n\u0005\u0000\u0183"+
		"\u0181\u0001\u0000\u0000\u0000\u0183\u0182\u0001\u0000\u0000\u0000\u0184"+
		"\u0007\u0001\u0000\u0000\u0000\u0185\u0186\u0003\f\u0006\u0000\u0186\u0187"+
		"\u0005\u000e\u0000\u0000\u0187\u0188\u0003\u000e\u0007\u0000\u0188\u0189"+
		"\u0005\u000f\u0000\u0000\u0189\t\u0001\u0000\u0000\u0000\u018a\u018c\u0007"+
		"\u0000\u0000\u0000\u018b\u018a\u0001\u0000\u0000\u0000\u018b\u018c\u0001"+
		"\u0000\u0000\u0000\u018c\u018d\u0001\u0000\u0000\u0000\u018d\u018e\u0005"+
		"a\u0000\u0000\u018e\u018f\u0003\u0140\u00a0\u0000\u018f\u000b\u0001\u0000"+
		"\u0000\u0000\u0190\u0192\u0007\u0000\u0000\u0000\u0191\u0190\u0001\u0000"+
		"\u0000\u0000\u0191\u0192\u0001\u0000\u0000\u0000\u0192\u0193\u0001\u0000"+
		"\u0000\u0000\u0193\u0194\u0005a\u0000\u0000\u0194\u0196\u0003\u0140\u00a0"+
		"\u0000\u0195\u0197\u0003\u0012\t\u0000\u0196\u0195\u0001\u0000\u0000\u0000"+
		"\u0196\u0197\u0001\u0000\u0000\u0000\u0197\r\u0001\u0000\u0000\u0000\u0198"+
		"\u019a\u0003\u0010\b\u0000\u0199\u0198\u0001\u0000\u0000\u0000\u019a\u019d"+
		"\u0001\u0000\u0000\u0000\u019b\u0199\u0001\u0000\u0000\u0000\u019b\u019c"+
		"\u0001\u0000\u0000\u0000\u019c\u000f\u0001\u0000\u0000\u0000\u019d\u019b"+
		"\u0001\u0000\u0000\u0000\u019e\u01b4\u0003\u0138\u009c\u0000\u019f\u01a0"+
		"\u0003R)\u0000\u01a0\u01a1\u0005\u000b\u0000\u0000\u01a1\u01b5\u0001\u0000"+
		"\u0000\u0000\u01a2\u01a3\u00036\u001b\u0000\u01a3\u01a4\u0005\u000b\u0000"+
		"\u0000\u01a4\u01b5\u0001\u0000\u0000\u0000\u01a5\u01a6\u0003\u00ceg\u0000"+
		"\u01a6\u01a7\u0005\u000b\u0000\u0000\u01a7\u01b5\u0001\u0000\u0000\u0000"+
		"\u01a8\u01a9\u0003\u00ccf\u0000\u01a9\u01aa\u0005\u000b\u0000\u0000\u01aa"+
		"\u01b5\u0001\u0000\u0000\u0000\u01ab\u01ac\u0003\u00d0h\u0000\u01ac\u01ad"+
		"\u0005\u000b\u0000\u0000\u01ad\u01b5\u0001\u0000\u0000\u0000\u01ae\u01af"+
		"\u0003\u00eew\u0000\u01af\u01b0\u0005\u000b\u0000\u0000\u01b0\u01b5\u0001"+
		"\u0000\u0000\u0000\u01b1\u01b2\u0003\u00f0x\u0000\u01b2\u01b3\u0005\u000b"+
		"\u0000\u0000\u01b3\u01b5\u0001\u0000\u0000\u0000\u01b4\u019f\u0001\u0000"+
		"\u0000\u0000\u01b4\u01a2\u0001\u0000\u0000\u0000\u01b4\u01a5\u0001\u0000"+
		"\u0000\u0000\u01b4\u01a8\u0001\u0000\u0000\u0000\u01b4\u01ab\u0001\u0000"+
		"\u0000\u0000\u01b4\u01ae\u0001\u0000\u0000\u0000\u01b4\u01b1\u0001\u0000"+
		"\u0000\u0000\u01b5\u0011\u0001\u0000\u0000\u0000\u01b6\u01b7\u0005\f\u0000"+
		"\u0000\u01b7\u01bc\u0003\u0014\n\u0000\u01b8\u01b9\u0005\r\u0000\u0000"+
		"\u01b9\u01bb\u0003\u0014\n\u0000\u01ba\u01b8\u0001\u0000\u0000\u0000\u01bb"+
		"\u01be\u0001\u0000\u0000\u0000\u01bc\u01ba\u0001\u0000\u0000\u0000\u01bc"+
		"\u01bd\u0001\u0000\u0000\u0000\u01bd\u0013\u0001\u0000\u0000\u0000\u01be"+
		"\u01bc\u0001\u0000\u0000\u0000\u01bf\u01c0\u0003\u0016\u000b\u0000\u01c0"+
		"\u0015\u0001\u0000\u0000\u0000\u01c1\u01c2\u0003\u0138\u009c\u0000\u01c2"+
		"\u01c3\u0003\u0018\f\u0000\u01c3\u0017\u0001\u0000\u0000\u0000\u01c4\u01c6"+
		"\u0005 \u0000\u0000\u01c5\u01c4\u0001\u0000\u0000\u0000\u01c5\u01c6\u0001"+
		"\u0000\u0000\u0000\u01c6\u01c7\u0001\u0000\u0000\u0000\u01c7\u01cc\u0005"+
		"q\u0000\u0000\u01c8\u01c9\u0005 \u0000\u0000\u01c9\u01cb\u0005q\u0000"+
		"\u0000\u01ca\u01c8\u0001\u0000\u0000\u0000\u01cb\u01ce\u0001\u0000\u0000"+
		"\u0000\u01cc\u01ca\u0001\u0000\u0000\u0000\u01cc\u01cd\u0001\u0000\u0000"+
		"\u0000\u01cd\u0019\u0001\u0000\u0000\u0000\u01ce\u01cc\u0001\u0000\u0000"+
		"\u0000\u01cf\u01d4\u0003\u0138\u009c\u0000\u01d0\u01d5\u0003\"\u0011\u0000"+
		"\u01d1\u01d5\u0003 \u0010\u0000\u01d2\u01d5\u0003\u001e\u000f\u0000\u01d3"+
		"\u01d5\u0003\u001c\u000e\u0000\u01d4\u01d0\u0001\u0000\u0000\u0000\u01d4"+
		"\u01d1\u0001\u0000\u0000\u0000\u01d4\u01d2\u0001\u0000\u0000\u0000\u01d4"+
		"\u01d3\u0001\u0000\u0000\u0000\u01d5\u001b\u0001\u0000\u0000\u0000\u01d6"+
		"\u01d8\u0005W\u0000\u0000\u01d7\u01d6\u0001\u0000\u0000\u0000\u01d7\u01d8"+
		"\u0001\u0000\u0000\u0000\u01d8\u01d9\u0001\u0000\u0000\u0000\u01d9\u01da"+
		"\u0005H\u0000\u0000\u01da\u01db\u0003\u0140\u00a0\u0000\u01db\u001d\u0001"+
		"\u0000\u0000\u0000\u01dc\u01dd\u0005H\u0000\u0000\u01dd\u01de\u0003\u0140"+
		"\u00a0\u0000\u01de\u01df\u0003V+\u0000\u01df\u001f\u0001\u0000\u0000\u0000"+
		"\u01e0\u01e1\u0005W\u0000\u0000\u01e1\u01e2\u0005H\u0000\u0000\u01e2\u01e3"+
		"\u0003\u0140\u00a0\u0000\u01e3\u01e4\u0003&\u0013\u0000\u01e4\u01e8\u0005"+
		"\u000e\u0000\u0000\u01e5\u01e7\u0003\u0010\b\u0000\u01e6\u01e5\u0001\u0000"+
		"\u0000\u0000\u01e7\u01ea\u0001\u0000\u0000\u0000\u01e8\u01e6\u0001\u0000"+
		"\u0000\u0000\u01e8\u01e9\u0001\u0000\u0000\u0000\u01e9\u01eb\u0001\u0000"+
		"\u0000\u0000\u01ea\u01e8\u0001\u0000\u0000\u0000\u01eb\u01ec\u0005\u000f"+
		"\u0000\u0000\u01ec!\u0001\u0000\u0000\u0000\u01ed\u01ee\u0003$\u0012\u0000"+
		"\u01ee\u01f2\u0005\u000e\u0000\u0000\u01ef\u01f1\u0003*\u0015\u0000\u01f0"+
		"\u01ef\u0001\u0000\u0000\u0000\u01f1\u01f4\u0001\u0000\u0000\u0000\u01f2"+
		"\u01f0\u0001\u0000\u0000\u0000\u01f2\u01f3\u0001\u0000\u0000\u0000\u01f3"+
		"\u01f5\u0001\u0000\u0000\u0000\u01f4\u01f2\u0001\u0000\u0000\u0000\u01f5"+
		"\u01f6\u0005\u000f\u0000\u0000\u01f6#\u0001\u0000\u0000\u0000\u01f7\u01f9"+
		"\u0005-\u0000\u0000\u01f8\u01f7\u0001\u0000\u0000\u0000\u01f8\u01f9\u0001"+
		"\u0000\u0000\u0000\u01f9\u01fa\u0001\u0000\u0000\u0000\u01fa\u01fb\u0005"+
		"H\u0000\u0000\u01fb\u01fc\u0003\u0140\u00a0\u0000\u01fc\u01fd\u0003&\u0013"+
		"\u0000\u01fd%\u0001\u0000\u0000\u0000\u01fe\u0200\u0005\f\u0000\u0000"+
		"\u01ff\u0201\u0005L\u0000\u0000\u0200\u01ff\u0001\u0000\u0000\u0000\u0200"+
		"\u0201\u0001\u0000\u0000\u0000\u0201\u0202\u0001\u0000\u0000\u0000\u0202"+
		"\u0207\u0003(\u0014\u0000\u0203\u0204\u0005\r\u0000\u0000\u0204\u0206"+
		"\u0003(\u0014\u0000\u0205\u0203\u0001\u0000\u0000\u0000\u0206\u0209\u0001"+
		"\u0000\u0000\u0000\u0207\u0205\u0001\u0000\u0000\u0000\u0207\u0208\u0001"+
		"\u0000\u0000\u0000\u0208\u020b\u0001\u0000\u0000\u0000\u0209\u0207\u0001"+
		"\u0000\u0000\u0000\u020a\u01fe\u0001\u0000\u0000\u0000\u020a\u020b\u0001"+
		"\u0000\u0000\u0000\u020b\u0215\u0001\u0000\u0000\u0000\u020c\u020d\u0005"+
		"I\u0000\u0000\u020d\u0212\u0003\u0014\n\u0000\u020e\u020f\u0005\r\u0000"+
		"\u0000\u020f\u0211\u0003\u0014\n\u0000\u0210\u020e\u0001\u0000\u0000\u0000"+
		"\u0211\u0214\u0001\u0000\u0000\u0000\u0212\u0210\u0001\u0000\u0000\u0000"+
		"\u0212\u0213\u0001\u0000\u0000\u0000\u0213\u0216\u0001\u0000\u0000\u0000"+
		"\u0214\u0212\u0001\u0000\u0000\u0000\u0215\u020c\u0001\u0000\u0000\u0000"+
		"\u0215\u0216\u0001\u0000\u0000\u0000\u0216\'\u0001\u0000\u0000\u0000\u0217"+
		"\u0218\u0003\u0016\u000b\u0000\u0218)\u0001\u0000\u0000\u0000\u0219\u021d"+
		"\u0003\u0010\b\u0000\u021a\u021d\u0003,\u0016\u0000\u021b\u021d\u0003"+
		".\u0017\u0000\u021c\u0219\u0001\u0000\u0000\u0000\u021c\u021a\u0001\u0000"+
		"\u0000\u0000\u021c\u021b\u0001\u0000\u0000\u0000\u021d+\u0001\u0000\u0000"+
		"\u0000\u021e\u0223\u0003\u0138\u009c\u0000\u021f\u0220\u0005<\u0000\u0000"+
		"\u0220\u0224\u0003\u0138\u009c\u0000\u0221\u0222\u00057\u0000\u0000\u0222"+
		"\u0224\u0003\u0138\u009c\u0000\u0223\u021f\u0001\u0000\u0000\u0000\u0223"+
		"\u0221\u0001\u0000\u0000\u0000\u0224\u0225\u0001\u0000\u0000\u0000\u0225"+
		"\u0226\u0003V+\u0000\u0226\u0227\u0003d2\u0000\u0227\u0228\u0005\u000b"+
		"\u0000\u0000\u0228-\u0001\u0000\u0000\u0000\u0229\u022a\u0003\u0138\u009c"+
		"\u0000\u022a\u022b\u0005C\u0000\u0000\u022b\u022c\u0003\u0140\u00a0\u0000"+
		"\u022c\u022e\u0005\u0010\u0000\u0000\u022d\u022f\u00030\u0018\u0000\u022e"+
		"\u022d\u0001\u0000\u0000\u0000\u022e\u022f\u0001\u0000\u0000\u0000\u022f"+
		"\u0230\u0001\u0000\u0000\u0000\u0230\u0232\u0005\u0011\u0000\u0000\u0231"+
		"\u0233\u0003\u00dcn\u0000\u0232\u0231\u0001\u0000\u0000\u0000\u0232\u0233"+
		"\u0001\u0000\u0000\u0000\u0233\u0234\u0001\u0000\u0000\u0000\u0234\u0235"+
		"\u0005\u000b\u0000\u0000\u0235/\u0001\u0000\u0000\u0000\u0236\u023b\u0003"+
		"2\u0019\u0000\u0237\u0238\u0005\r\u0000\u0000\u0238\u023a\u00032\u0019"+
		"\u0000\u0239\u0237\u0001\u0000\u0000\u0000\u023a\u023d\u0001\u0000\u0000"+
		"\u0000\u023b\u0239\u0001\u0000\u0000\u0000\u023b\u023c\u0001\u0000\u0000"+
		"\u0000\u023c1\u0001\u0000\u0000\u0000\u023d\u023b\u0001\u0000\u0000\u0000"+
		"\u023e\u023f\u0003\u0138\u009c\u0000\u023f\u0240\u00034\u001a\u0000\u0240"+
		"\u0241\u0003\u0138\u009c\u0000\u0241\u0242\u0003\u00e0p\u0000\u0242\u0243"+
		"\u0003\u0138\u009c\u0000\u0243\u0244\u0003h4\u0000\u02443\u0001\u0000"+
		"\u0000\u0000\u0245\u0246\u0005:\u0000\u0000\u02465\u0001\u0000\u0000\u0000"+
		"\u0247\u0248\u0005F\u0000\u0000\u0248\u0249\u00038\u001c\u0000\u0249\u024a"+
		"\u0003\u0140\u00a0\u0000\u024a\u024b\u0005\u001e\u0000\u0000\u024b\u024c"+
		"\u0003:\u001d\u0000\u024c7\u0001\u0000\u0000\u0000\u024d\u0258\u0003\u0138"+
		"\u009c\u0000\u024e\u0259\u0003n7\u0000\u024f\u0259\u0003\u0084B\u0000"+
		"\u0250\u0259\u0003\u0086C\u0000\u0251\u0259\u0003\u0088D\u0000\u0252\u0259"+
		"\u0003l6\u0000\u0253\u0259\u0003\u00c4b\u0000\u0254\u0259\u0003\u00c6"+
		"c\u0000\u0255\u0259\u0003\u00e4r\u0000\u0256\u0259\u0003\u0018\f\u0000"+
		"\u0257\u0259\u0003\u008aE\u0000\u0258\u024e\u0001\u0000\u0000\u0000\u0258"+
		"\u024f\u0001\u0000\u0000\u0000\u0258\u0250\u0001\u0000\u0000\u0000\u0258"+
		"\u0251\u0001\u0000\u0000\u0000\u0258\u0252\u0001\u0000\u0000\u0000\u0258"+
		"\u0253\u0001\u0000\u0000\u0000\u0258\u0254\u0001\u0000\u0000\u0000\u0258"+
		"\u0255\u0001\u0000\u0000\u0000\u0258\u0256\u0001\u0000\u0000\u0000\u0258"+
		"\u0257\u0001\u0000\u0000\u0000\u02599\u0001\u0000\u0000\u0000\u025a\u025b"+
		"\u0003<\u001e\u0000\u025b;\u0001\u0000\u0000\u0000\u025c\u0261\u0003>"+
		"\u001f\u0000\u025d\u025e\u0005\u001d\u0000\u0000\u025e\u0260\u0003>\u001f"+
		"\u0000\u025f\u025d\u0001\u0000\u0000\u0000\u0260\u0263\u0001\u0000\u0000"+
		"\u0000\u0261\u025f\u0001\u0000\u0000\u0000\u0261\u0262\u0001\u0000\u0000"+
		"\u0000\u0262=\u0001\u0000\u0000\u0000\u0263\u0261\u0001\u0000\u0000\u0000"+
		"\u0264\u0269\u0003@ \u0000\u0265\u0266\u0005\u001b\u0000\u0000\u0266\u0268"+
		"\u0003@ \u0000\u0267\u0265\u0001\u0000\u0000\u0000\u0268\u026b\u0001\u0000"+
		"\u0000\u0000\u0269\u0267\u0001\u0000\u0000\u0000\u0269\u026a\u0001\u0000"+
		"\u0000\u0000\u026a?\u0001\u0000\u0000\u0000\u026b\u0269\u0001\u0000\u0000"+
		"\u0000\u026c\u0271\u0003B!\u0000\u026d\u026e\u0005\u001c\u0000\u0000\u026e"+
		"\u0270\u0003B!\u0000\u026f\u026d\u0001\u0000\u0000\u0000\u0270\u0273\u0001"+
		"\u0000\u0000\u0000\u0271\u026f\u0001\u0000\u0000\u0000\u0271\u0272\u0001"+
		"\u0000\u0000\u0000\u0272A\u0001\u0000\u0000\u0000\u0273\u0271\u0001\u0000"+
		"\u0000\u0000\u0274\u0279\u0003D\"\u0000\u0275\u0276\u0007\u0001\u0000"+
		"\u0000\u0276\u0278\u0003D\"\u0000\u0277\u0275\u0001\u0000\u0000\u0000"+
		"\u0278\u027b\u0001\u0000\u0000\u0000\u0279\u0277\u0001\u0000\u0000\u0000"+
		"\u0279\u027a\u0001\u0000\u0000\u0000\u027aC\u0001\u0000\u0000\u0000\u027b"+
		"\u0279\u0001\u0000\u0000\u0000\u027c\u0281\u0003F#\u0000\u027d\u027e\u0007"+
		"\u0002\u0000\u0000\u027e\u0280\u0003F#\u0000\u027f\u027d\u0001\u0000\u0000"+
		"\u0000\u0280\u0283\u0001\u0000\u0000\u0000\u0281\u027f\u0001\u0000\u0000"+
		"\u0000\u0281\u0282\u0001\u0000\u0000\u0000\u0282E\u0001\u0000\u0000\u0000"+
		"\u0283\u0281\u0001\u0000\u0000\u0000\u0284\u0289\u0003H$\u0000\u0285\u0286"+
		"\u0007\u0003\u0000\u0000\u0286\u0288\u0003H$\u0000\u0287\u0285\u0001\u0000"+
		"\u0000\u0000\u0288\u028b\u0001\u0000\u0000\u0000\u0289\u0287\u0001\u0000"+
		"\u0000\u0000\u0289\u028a\u0001\u0000\u0000\u0000\u028aG\u0001\u0000\u0000"+
		"\u0000\u028b\u0289\u0001\u0000\u0000\u0000\u028c\u028d\u0003J%\u0000\u028d"+
		"\u028e\u0003L&\u0000\u028e\u0291\u0001\u0000\u0000\u0000\u028f\u0291\u0003"+
		"L&\u0000\u0290\u028c\u0001\u0000\u0000\u0000\u0290\u028f\u0001\u0000\u0000"+
		"\u0000\u0291I\u0001\u0000\u0000\u0000\u0292\u0293\u0007\u0004\u0000\u0000"+
		"\u0293K\u0001\u0000\u0000\u0000\u0294\u029b\u0003\u0018\f\u0000\u0295"+
		"\u029b\u0003N\'\u0000\u0296\u0297\u0005\u0010\u0000\u0000\u0297\u0298"+
		"\u0003:\u001d\u0000\u0298\u0299\u0005\u0011\u0000\u0000\u0299\u029b\u0001"+
		"\u0000\u0000\u0000\u029a\u0294\u0001\u0000\u0000\u0000\u029a\u0295\u0001"+
		"\u0000\u0000\u0000\u029a\u0296\u0001\u0000\u0000\u0000\u029bM\u0001\u0000"+
		"\u0000\u0000\u029c\u029d\u0007\u0005\u0000\u0000\u029dO\u0001\u0000\u0000"+
		"\u0000\u029e\u029f\u0003:\u001d\u0000\u029fQ\u0001\u0000\u0000\u0000\u02a0"+
		"\u02a1\u0005*\u0000\u0000\u02a1\u02a2\u0003\u0138\u009c\u0000\u02a2\u02a3"+
		"\u0003T*\u0000\u02a3\u02af\u0001\u0000\u0000\u0000\u02a4\u02af\u0003\u00a8"+
		"T\u0000\u02a5\u02af\u0003\u00aeW\u0000\u02a6\u02af\u0003\u00ba]\u0000"+
		"\u02a7\u02af\u0003\u009eO\u0000\u02a8\u02af\u0003\u00a4R\u0000\u02a9\u02aa"+
		"\u00052\u0000\u0000\u02aa\u02ab\u0003\u0138\u009c\u0000\u02ab\u02ac\u0003"+
		"h4\u0000\u02ac\u02af\u0001\u0000\u0000\u0000\u02ad\u02af\u0003\u00e8t"+
		"\u0000\u02ae\u02a0\u0001\u0000\u0000\u0000\u02ae\u02a4\u0001\u0000\u0000"+
		"\u0000\u02ae\u02a5\u0001\u0000\u0000\u0000\u02ae\u02a6\u0001\u0000\u0000"+
		"\u0000\u02ae\u02a7\u0001\u0000\u0000\u0000\u02ae\u02a8\u0001\u0000\u0000"+
		"\u0000\u02ae\u02a9\u0001\u0000\u0000\u0000\u02ae\u02ad\u0001\u0000\u0000"+
		"\u0000\u02afS\u0001\u0000\u0000\u0000\u02b0\u02b1\u0003V+\u0000\u02b1"+
		"\u02b2\u0003d2\u0000\u02b2U\u0001\u0000\u0000\u0000\u02b3\u02b6\u0003"+
		"X,\u0000\u02b4\u02b6\u0003`0\u0000\u02b5\u02b3\u0001\u0000\u0000\u0000"+
		"\u02b5\u02b4\u0001\u0000\u0000\u0000\u02b6W\u0001\u0000\u0000\u0000\u02b7"+
		"\u02bb\u0003\\.\u0000\u02b8\u02bb\u0003^/\u0000\u02b9\u02bb\u0003\u0018"+
		"\f\u0000\u02ba\u02b7\u0001\u0000\u0000\u0000\u02ba\u02b8\u0001\u0000\u0000"+
		"\u0000\u02ba\u02b9\u0001\u0000\u0000\u0000\u02bbY\u0001\u0000\u0000\u0000"+
		"\u02bc\u02c0\u0003n7\u0000\u02bd\u02c0\u0003\u0088D\u0000\u02be\u02c0"+
		"\u0003\u008aE\u0000\u02bf\u02bc\u0001\u0000\u0000\u0000\u02bf\u02bd\u0001"+
		"\u0000\u0000\u0000\u02bf\u02be\u0001\u0000\u0000\u0000\u02c0[\u0001\u0000"+
		"\u0000\u0000\u02c1\u02cb\u0003l6\u0000\u02c2\u02cb\u0003n7\u0000\u02c3"+
		"\u02cb\u0003\u0084B\u0000\u02c4\u02cb\u0003\u0086C\u0000\u02c5\u02cb\u0003"+
		"\u0088D\u0000\u02c6\u02cb\u0003\u008aE\u0000\u02c7\u02cb\u0003\u008cF"+
		"\u0000\u02c8\u02cb\u0003\u008eG\u0000\u02c9\u02cb\u0003\u00e6s\u0000\u02ca"+
		"\u02c1\u0001\u0000\u0000\u0000\u02ca\u02c2\u0001\u0000\u0000\u0000\u02ca"+
		"\u02c3\u0001\u0000\u0000\u0000\u02ca\u02c4\u0001\u0000\u0000\u0000\u02ca"+
		"\u02c5\u0001\u0000\u0000\u0000\u02ca\u02c6\u0001\u0000\u0000\u0000\u02ca"+
		"\u02c7\u0001\u0000\u0000\u0000\u02ca\u02c8\u0001\u0000\u0000\u0000\u02ca"+
		"\u02c9\u0001\u0000\u0000\u0000\u02cb]\u0001\u0000\u0000\u0000\u02cc\u02d3"+
		"\u0003\u00be_\u0000\u02cd\u02d3\u0003\u00c0`\u0000\u02ce\u02d3\u0003\u00c2"+
		"a\u0000\u02cf\u02d3\u0003\u00c4b\u0000\u02d0\u02d3\u0003\u00c6c\u0000"+
		"\u02d1\u02d3\u0003\u00e2q\u0000\u02d2\u02cc\u0001\u0000\u0000\u0000\u02d2"+
		"\u02cd\u0001\u0000\u0000\u0000\u02d2\u02ce\u0001\u0000\u0000\u0000\u02d2"+
		"\u02cf\u0001\u0000\u0000\u0000\u02d2\u02d0\u0001\u0000\u0000\u0000\u02d2"+
		"\u02d1\u0001\u0000\u0000\u0000\u02d3_\u0001\u0000\u0000\u0000\u02d4\u02da"+
		"\u0003\u00a8T\u0000\u02d5\u02da\u0003\u00aeW\u0000\u02d6\u02da\u0003\u00ba"+
		"]\u0000\u02d7\u02da\u0003\u009eO\u0000\u02d8\u02da\u0003\u00a4R\u0000"+
		"\u02d9\u02d4\u0001\u0000\u0000\u0000\u02d9\u02d5\u0001\u0000\u0000\u0000"+
		"\u02d9\u02d6\u0001\u0000\u0000\u0000\u02d9\u02d7\u0001\u0000\u0000\u0000"+
		"\u02d9\u02d8\u0001\u0000\u0000\u0000\u02daa\u0001\u0000\u0000\u0000\u02db"+
		"\u02e0\u0003\u0140\u00a0\u0000\u02dc\u02dd\u0005\r\u0000\u0000\u02dd\u02df"+
		"\u0003\u0140\u00a0\u0000\u02de\u02dc\u0001\u0000\u0000\u0000\u02df\u02e2"+
		"\u0001\u0000\u0000\u0000\u02e0\u02de\u0001\u0000\u0000\u0000\u02e0\u02e1"+
		"\u0001\u0000\u0000\u0000\u02e1c\u0001\u0000\u0000\u0000\u02e2\u02e0\u0001"+
		"\u0000\u0000\u0000\u02e3\u02e8\u0003f3\u0000\u02e4\u02e5\u0005\r\u0000"+
		"\u0000\u02e5\u02e7\u0003f3\u0000\u02e6\u02e4\u0001\u0000\u0000\u0000\u02e7"+
		"\u02ea\u0001\u0000\u0000\u0000\u02e8\u02e6\u0001\u0000\u0000\u0000\u02e8"+
		"\u02e9\u0001\u0000\u0000\u0000\u02e9e\u0001\u0000\u0000\u0000\u02ea\u02e8"+
		"\u0001\u0000\u0000\u0000\u02eb\u02ee\u0003\u0138\u009c\u0000\u02ec\u02ef"+
		"\u0003h4\u0000\u02ed\u02ef\u0003j5\u0000\u02ee\u02ec\u0001\u0000\u0000"+
		"\u0000\u02ee\u02ed\u0001\u0000\u0000\u0000\u02efg\u0001\u0000\u0000\u0000"+
		"\u02f0\u02f1\u0005q\u0000\u0000\u02f1i\u0001\u0000\u0000\u0000\u02f2\u02f3"+
		"\u0003\u00c8d\u0000\u02f3k\u0001\u0000\u0000\u0000\u02f4\u02f9\u0005T"+
		"\u0000\u0000\u02f5\u02f9\u0005[\u0000\u0000\u02f6\u02f7\u0005>\u0000\u0000"+
		"\u02f7\u02f9\u0005[\u0000\u0000\u02f8\u02f4\u0001\u0000\u0000\u0000\u02f8"+
		"\u02f5\u0001\u0000\u0000\u0000\u02f8\u02f6\u0001\u0000\u0000\u0000\u02f9"+
		"m\u0001\u0000\u0000\u0000\u02fa\u02fd\u0003p8\u0000\u02fb\u02fd\u0003"+
		"z=\u0000\u02fc\u02fa\u0001\u0000\u0000\u0000\u02fc\u02fb\u0001\u0000\u0000"+
		"\u0000\u02fdo\u0001\u0000\u0000\u0000\u02fe\u0303\u0003t:\u0000\u02ff"+
		"\u0303\u0003v;\u0000\u0300\u0303\u0003x<\u0000\u0301\u0303\u0003r9\u0000"+
		"\u0302\u02fe\u0001\u0000\u0000\u0000\u0302\u02ff\u0001\u0000\u0000\u0000"+
		"\u0302\u0300\u0001\u0000\u0000\u0000\u0302\u0301\u0001\u0000\u0000\u0000"+
		"\u0303q\u0001\u0000\u0000\u0000\u0304\u0305\u0005h\u0000\u0000\u0305s"+
		"\u0001\u0000\u0000\u0000\u0306\u0307\u0007\u0006\u0000\u0000\u0307u\u0001"+
		"\u0000\u0000\u0000\u0308\u0309\u0007\u0007\u0000\u0000\u0309w\u0001\u0000"+
		"\u0000\u0000\u030a\u030b\u0005>\u0000\u0000\u030b\u030e\u0005>\u0000\u0000"+
		"\u030c\u030e\u0005n\u0000\u0000\u030d\u030a\u0001\u0000\u0000\u0000\u030d"+
		"\u030c\u0001\u0000\u0000\u0000\u030ey\u0001\u0000\u0000\u0000\u030f\u0314"+
		"\u0003~?\u0000\u0310\u0314\u0003\u0080@\u0000\u0311\u0314\u0003\u0082"+
		"A\u0000\u0312\u0314\u0003|>\u0000\u0313\u030f\u0001\u0000\u0000\u0000"+
		"\u0313\u0310\u0001\u0000\u0000\u0000\u0313\u0311\u0001\u0000\u0000\u0000"+
		"\u0313\u0312\u0001\u0000\u0000\u0000\u0314{\u0001\u0000\u0000\u0000\u0315"+
		"\u0316\u0005i\u0000\u0000\u0316}\u0001\u0000\u0000\u0000\u0317\u0318\u0005"+
		"M\u0000\u0000\u0318\u031b\u0005=\u0000\u0000\u0319\u031b\u0005k\u0000"+
		"\u0000\u031a\u0317\u0001\u0000\u0000\u0000\u031a\u0319\u0001\u0000\u0000"+
		"\u0000\u031b\u007f\u0001\u0000\u0000\u0000\u031c\u031d\u0005M\u0000\u0000"+
		"\u031d\u0320\u0005>\u0000\u0000\u031e\u0320\u0005m\u0000\u0000\u031f\u031c"+
		"\u0001\u0000\u0000\u0000\u031f\u031e\u0001\u0000\u0000\u0000\u0320\u0081"+
		"\u0001\u0000\u0000\u0000\u0321\u0322\u0005M\u0000\u0000\u0322\u0323\u0005"+
		">\u0000\u0000\u0323\u0326\u0005>\u0000\u0000\u0324\u0326\u0005o\u0000"+
		"\u0000\u0325\u0321\u0001\u0000\u0000\u0000\u0325\u0324\u0001\u0000\u0000"+
		"\u0000\u0326\u0083\u0001\u0000\u0000\u0000\u0327\u0328\u0005R\u0000\u0000"+
		"\u0328\u0085\u0001\u0000\u0000\u0000\u0329\u032a\u00059\u0000\u0000\u032a"+
		"\u0087\u0001\u0000\u0000\u0000\u032b\u032c\u0005U\u0000\u0000\u032c\u0089"+
		"\u0001\u0000\u0000\u0000\u032d\u032e\u0005.\u0000\u0000\u032e\u008b\u0001"+
		"\u0000\u0000\u0000\u032f\u0330\u0005Q\u0000\u0000\u0330\u008d\u0001\u0000"+
		"\u0000\u0000\u0331\u0332\u0005K\u0000\u0000\u0332\u008f\u0001\u0000\u0000"+
		"\u0000\u0333\u0336\u0003\u0092I\u0000\u0334\u0336\u0003\u009cN\u0000\u0335"+
		"\u0333\u0001\u0000\u0000\u0000\u0335\u0334\u0001\u0000\u0000\u0000\u0336"+
		"\u0091\u0001\u0000\u0000\u0000\u0337\u0338\u0003\u0094J\u0000\u0338\u0339"+
		"\u0005\u000e\u0000\u0000\u0339\u033a\u0003\u0098L\u0000\u033a\u033b\u0005"+
		"\u000f\u0000\u0000\u033b\u0093\u0001\u0000\u0000\u0000\u033c\u033d\u0005"+
		"p\u0000\u0000\u033d\u033f\u0003\u0140\u00a0\u0000\u033e\u0340\u0003\u0096"+
		"K\u0000\u033f\u033e\u0001\u0000\u0000\u0000\u033f\u0340\u0001\u0000\u0000"+
		"\u0000\u0340\u0095\u0001\u0000\u0000\u0000\u0341\u0342\u0005\f\u0000\u0000"+
		"\u0342\u0343\u0003\u0018\f\u0000\u0343\u0097\u0001\u0000\u0000\u0000\u0344"+
		"\u0350\u0003\u009aM\u0000\u0345\u0346\u0003\u00ba]\u0000\u0346\u0347\u0005"+
		"\u000b\u0000\u0000\u0347\u0350\u0001\u0000\u0000\u0000\u0348\u0349\u0003"+
		"6\u001b\u0000\u0349\u034a\u0005\u000b\u0000\u0000\u034a\u0350\u0001\u0000"+
		"\u0000\u0000\u034b\u034c\u0005*\u0000\u0000\u034c\u034d\u0003T*\u0000"+
		"\u034d\u034e\u0005\u000b\u0000\u0000\u034e\u0350\u0001\u0000\u0000\u0000"+
		"\u034f\u0344\u0001\u0000\u0000\u0000\u034f\u0345\u0001\u0000\u0000\u0000"+
		"\u034f\u0348\u0001\u0000\u0000\u0000\u034f\u034b\u0001\u0000\u0000\u0000"+
		"\u0350\u0353\u0001\u0000\u0000\u0000\u0351\u034f\u0001\u0000\u0000\u0000"+
		"\u0351\u0352\u0001\u0000\u0000\u0000\u0352\u0099\u0001\u0000\u0000\u0000"+
		"\u0353\u0351\u0001\u0000\u0000\u0000\u0354\u0355\u00038\u001c\u0000\u0355"+
		"\u0358\u0003h4\u0000\u0356\u0357\u0005;\u0000\u0000\u0357\u0359\u0003"+
		":\u001d\u0000\u0358\u0356\u0001\u0000\u0000\u0000\u0358\u0359\u0001\u0000"+
		"\u0000\u0000\u0359\u035a\u0001\u0000\u0000\u0000\u035a\u035b\u0005\u000b"+
		"\u0000\u0000\u035b\u009b\u0001\u0000\u0000\u0000\u035c\u035d\u0005p\u0000"+
		"\u0000\u035d\u035e\u0003\u0018\f\u0000\u035e\u009d\u0001\u0000\u0000\u0000"+
		"\u035f\u0360\u0005f\u0000\u0000\u0360\u0363\u0003\u0140\u00a0\u0000\u0361"+
		"\u0362\u0005\f\u0000\u0000\u0362\u0364\u0003\u0018\f\u0000\u0363\u0361"+
		"\u0001\u0000\u0000\u0000\u0363\u0364\u0001\u0000\u0000\u0000\u0364\u0365"+
		"\u0001\u0000\u0000\u0000\u0365\u0366\u0005\u000e\u0000\u0000\u0366\u0367"+
		"\u0003\u00a0P\u0000\u0367\u0368\u0005\u000f\u0000\u0000\u0368\u009f\u0001"+
		"\u0000\u0000\u0000\u0369\u036b\u0003\u00a2Q\u0000\u036a\u036c\u0003b1"+
		"\u0000\u036b\u036a\u0001\u0000\u0000\u0000\u036b\u036c\u0001\u0000\u0000"+
		"\u0000\u036c\u036d\u0001\u0000\u0000\u0000\u036d\u036e\u0005\u000b\u0000"+
		"\u0000\u036e\u0370\u0001\u0000\u0000\u0000\u036f\u0369\u0001\u0000\u0000"+
		"\u0000\u0370\u0371\u0001\u0000\u0000\u0000\u0371\u036f\u0001\u0000\u0000"+
		"\u0000\u0371\u0372\u0001\u0000\u0000\u0000\u0372\u00a1\u0001\u0000\u0000"+
		"\u0000\u0373\u0374\u0003\u0138\u009c\u0000\u0374\u0375\u0005e\u0000\u0000"+
		"\u0375\u0376\u0005\u0016\u0000\u0000\u0376\u0379\u0003P(\u0000\u0377\u0378"+
		"\u0005\r\u0000\u0000\u0378\u037a\u0003Z-\u0000\u0379\u0377\u0001\u0000"+
		"\u0000\u0000\u0379\u037a\u0001\u0000\u0000\u0000\u037a\u037b\u0001\u0000"+
		"\u0000\u0000\u037b\u037c\u0005\u0017\u0000\u0000\u037c\u00a3\u0001\u0000"+
		"\u0000\u0000\u037d\u037e\u0005g\u0000\u0000\u037e\u037f\u0003\u0140\u00a0"+
		"\u0000\u037f\u0380\u0005\u000e\u0000\u0000\u0380\u0381\u0003\u00a6S\u0000"+
		"\u0381\u0382\u0005\u000f\u0000\u0000\u0382\u00a5\u0001\u0000\u0000\u0000"+
		"\u0383\u0388\u0003\u0140\u00a0\u0000\u0384\u0385\u0005\r\u0000\u0000\u0385"+
		"\u0387\u0003\u0140\u00a0\u0000\u0386\u0384\u0001\u0000\u0000\u0000\u0387"+
		"\u038a\u0001\u0000\u0000\u0000\u0388\u0386\u0001\u0000\u0000\u0000\u0388"+
		"\u0389\u0001\u0000\u0000\u0000\u0389\u00a7\u0001\u0000\u0000\u0000\u038a"+
		"\u0388\u0001\u0000\u0000\u0000\u038b\u038c\u00051\u0000\u0000\u038c\u038f"+
		"\u0003\u0140\u00a0\u0000\u038d\u038e\u0005 \u0000\u0000\u038e\u0390\u0003"+
		"\u0018\f\u0000\u038f\u038d\u0001\u0000\u0000\u0000\u038f\u0390\u0001\u0000"+
		"\u0000\u0000\u0390\u0391\u0001\u0000\u0000\u0000\u0391\u0392\u0005\u000e"+
		"\u0000\u0000\u0392\u0393\u0003\u00aaU\u0000\u0393\u0394\u0005\u000f\u0000"+
		"\u0000\u0394\u00a9\u0001\u0000\u0000\u0000\u0395\u0397\u0003\u00acV\u0000"+
		"\u0396\u0395\u0001\u0000\u0000\u0000\u0397\u039a\u0001\u0000\u0000\u0000"+
		"\u0398\u0396\u0001\u0000\u0000\u0000\u0398\u0399\u0001\u0000\u0000\u0000"+
		"\u0399\u00ab\u0001\u0000\u0000\u0000\u039a\u0398\u0001\u0000\u0000\u0000"+
		"\u039b\u039c\u0003\u0138\u009c\u0000\u039c\u039d\u0003V+\u0000\u039d\u039e"+
		"\u0003d2\u0000\u039e\u039f\u0005\u000b\u0000\u0000\u039f\u00ad\u0001\u0000"+
		"\u0000\u0000\u03a0\u03a1\u0005O\u0000\u0000\u03a1\u03a2\u0003\u0140\u00a0"+
		"\u0000\u03a2\u03a3\u0005(\u0000\u0000\u03a3\u03a4\u0005\u0010\u0000\u0000"+
		"\u03a4\u03a5\u0003\u0138\u009c\u0000\u03a5\u03a6\u0003\u00b0X\u0000\u03a6"+
		"\u03a7\u0005\u0011\u0000\u0000\u03a7\u03a8\u0005\u000e\u0000\u0000\u03a8"+
		"\u03a9\u0003\u00b2Y\u0000\u03a9\u03aa\u0005\u000f\u0000\u0000\u03aa\u00af"+
		"\u0001\u0000\u0000\u0000\u03ab\u03b3\u0003n7\u0000\u03ac\u03b3\u0003\u0084"+
		"B\u0000\u03ad\u03b3\u0003\u0086C\u0000\u03ae\u03b3\u0003\u008aE\u0000"+
		"\u03af\u03b3\u0003\u0088D\u0000\u03b0\u03b3\u0003\u00ba]\u0000\u03b1\u03b3"+
		"\u0003\u0018\f\u0000\u03b2\u03ab\u0001\u0000\u0000\u0000\u03b2\u03ac\u0001"+
		"\u0000\u0000\u0000\u03b2\u03ad\u0001\u0000\u0000\u0000\u03b2\u03ae\u0001"+
		"\u0000\u0000\u0000\u03b2\u03af\u0001\u0000\u0000\u0000\u03b2\u03b0\u0001"+
		"\u0000\u0000\u0000\u03b2\u03b1\u0001\u0000\u0000\u0000\u03b3\u00b1\u0001"+
		"\u0000\u0000\u0000\u03b4\u03b6\u0003\u00b4Z\u0000\u03b5\u03b4\u0001\u0000"+
		"\u0000\u0000\u03b6\u03b7\u0001\u0000\u0000\u0000\u03b7\u03b5\u0001\u0000"+
		"\u0000\u0000\u03b7\u03b8\u0001\u0000\u0000\u0000\u03b8\u00b3\u0001\u0000"+
		"\u0000\u0000\u03b9\u03bb\u0003\u00b6[\u0000\u03ba\u03b9\u0001\u0000\u0000"+
		"\u0000\u03bb\u03bc\u0001\u0000\u0000\u0000\u03bc\u03ba\u0001\u0000\u0000"+
		"\u0000\u03bc\u03bd\u0001\u0000\u0000\u0000\u03bd\u03be\u0001\u0000\u0000"+
		"\u0000\u03be\u03bf\u0003\u00b8\\\u0000\u03bf\u03c0\u0005\u000b\u0000\u0000"+
		"\u03c0\u00b5\u0001\u0000\u0000\u0000\u03c1\u03c8\u0003\u0138\u009c\u0000"+
		"\u03c2\u03c3\u0005S\u0000\u0000\u03c3\u03c4\u0003:\u001d\u0000\u03c4\u03c5"+
		"\u0005\f\u0000\u0000\u03c5\u03c9\u0001\u0000\u0000\u0000\u03c6\u03c7\u0005"+
		";\u0000\u0000\u03c7\u03c9\u0005\f\u0000\u0000\u03c8\u03c2\u0001\u0000"+
		"\u0000\u0000\u03c8\u03c6\u0001\u0000\u0000\u0000\u03c9\u00b7\u0001\u0000"+
		"\u0000\u0000\u03ca\u03cb\u0003\u0138\u009c\u0000\u03cb\u03cc\u0003V+\u0000"+
		"\u03cc\u03cd\u0003f3\u0000\u03cd\u00b9\u0001\u0000\u0000\u0000\u03ce\u03cf"+
		"\u0005?\u0000\u0000\u03cf\u03d0\u0003\u0140\u00a0\u0000\u03d0\u03d1\u0005"+
		"\u000e\u0000\u0000\u03d1\u03d6\u0003\u00bc^\u0000\u03d2\u03d3\u0005\r"+
		"\u0000\u0000\u03d3\u03d5\u0003\u00bc^\u0000\u03d4\u03d2\u0001\u0000\u0000"+
		"\u0000\u03d5\u03d8\u0001\u0000\u0000\u0000\u03d6\u03d4\u0001\u0000\u0000"+
		"\u0000\u03d6\u03d7\u0001\u0000\u0000\u0000\u03d7\u03d9\u0001\u0000\u0000"+
		"\u0000\u03d8\u03d6\u0001\u0000\u0000\u0000\u03d9\u03da\u0005\u000f\u0000"+
		"\u0000\u03da\u00bb\u0001\u0000\u0000\u0000\u03db\u03dc\u0003\u0140\u00a0"+
		"\u0000\u03dc\u00bd\u0001\u0000\u0000\u0000\u03dd\u03de\u0005/\u0000\u0000"+
		"\u03de\u03df\u0005\u0016\u0000\u0000\u03df\u03e0\u0003\u0138\u009c\u0000"+
		"\u03e0\u03e3\u0003X,\u0000\u03e1\u03e2\u0005\r\u0000\u0000\u03e2\u03e4"+
		"\u0003P(\u0000\u03e3\u03e1\u0001\u0000\u0000\u0000\u03e3\u03e4\u0001\u0000"+
		"\u0000\u0000\u03e4\u03e5\u0001\u0000\u0000\u0000\u03e5\u03e6\u0005\u0017"+
		"\u0000\u0000\u03e6\u00bf\u0001\u0000\u0000\u0000\u03e7\u03e8\u0005c\u0000"+
		"\u0000\u03e8\u03e9\u0005\u0016\u0000\u0000\u03e9\u03ec\u0003X,\u0000\u03ea"+
		"\u03eb\u0005\r\u0000\u0000\u03eb\u03ed\u0003P(\u0000\u03ec\u03ea\u0001"+
		"\u0000\u0000\u0000\u03ec\u03ed\u0001\u0000\u0000\u0000\u03ed\u03ee\u0001"+
		"\u0000\u0000\u0000\u03ee\u03ef\u0005\u0017\u0000\u0000\u03ef\u00c1\u0001"+
		"\u0000\u0000\u0000\u03f0\u03f1\u0005d\u0000\u0000\u03f1\u03f2\u0005\u0016"+
		"\u0000\u0000\u03f2\u03f3\u0003X,\u0000\u03f3\u03f4\u0005\r\u0000\u0000"+
		"\u03f4\u03f7\u0003X,\u0000\u03f5\u03f6\u0005\r\u0000\u0000\u03f6\u03f8"+
		"\u0003P(\u0000\u03f7\u03f5\u0001\u0000\u0000\u0000\u03f7\u03f8\u0001\u0000"+
		"\u0000\u0000\u03f8\u03f9\u0001\u0000\u0000\u0000\u03f9\u03fa\u0005\u0017"+
		"\u0000\u0000\u03fa\u00c3\u0001\u0000\u0000\u0000\u03fb\u0400\u0005\'\u0000"+
		"\u0000\u03fc\u03fd\u0005\u0016\u0000\u0000\u03fd\u03fe\u0003P(\u0000\u03fe"+
		"\u03ff\u0005\u0017\u0000\u0000\u03ff\u0401\u0001\u0000\u0000\u0000\u0400"+
		"\u03fc\u0001\u0000\u0000\u0000\u0400\u0401\u0001\u0000\u0000\u0000\u0401"+
		"\u00c5\u0001\u0000\u0000\u0000\u0402\u0407\u0005@\u0000\u0000\u0403\u0404"+
		"\u0005\u0016\u0000\u0000\u0404\u0405\u0003P(\u0000\u0405\u0406\u0005\u0017"+
		"\u0000\u0000\u0406\u0408\u0001\u0000\u0000\u0000\u0407\u0403\u0001\u0000"+
		"\u0000\u0000\u0407\u0408\u0001\u0000\u0000\u0000\u0408\u00c7\u0001\u0000"+
		"\u0000\u0000\u0409\u040b\u0005q\u0000\u0000\u040a\u040c\u0003\u00cae\u0000"+
		"\u040b\u040a\u0001\u0000\u0000\u0000\u040c\u040d\u0001\u0000\u0000\u0000"+
		"\u040d\u040b\u0001\u0000\u0000\u0000\u040d\u040e\u0001\u0000\u0000\u0000"+
		"\u040e\u00c9\u0001\u0000\u0000\u0000\u040f\u0410\u0005\u0012\u0000\u0000"+
		"\u0410\u0411\u0003P(\u0000\u0411\u0412\u0005\u0013\u0000\u0000\u0412\u00cb"+
		"\u0001\u0000\u0000\u0000\u0413\u0416\u0003\u00f2y\u0000\u0414\u0416\u0003"+
		"\u00f6{\u0000\u0415\u0413\u0001\u0000\u0000\u0000\u0415\u0414\u0001\u0000"+
		"\u0000\u0000\u0416\u00cd\u0001\u0000\u0000\u0000\u0417\u0418\u0005D\u0000"+
		"\u0000\u0418\u0419\u0003\u0140\u00a0\u0000\u0419\u041d\u0005\u000e\u0000"+
		"\u0000\u041a\u041c\u0003\u00acV\u0000\u041b\u041a\u0001\u0000\u0000\u0000"+
		"\u041c\u041f\u0001\u0000\u0000\u0000\u041d\u041b\u0001\u0000\u0000\u0000"+
		"\u041d\u041e\u0001\u0000\u0000\u0000\u041e\u0420\u0001\u0000\u0000\u0000"+
		"\u041f\u041d\u0001\u0000\u0000\u0000\u0420\u0421\u0005\u000f\u0000\u0000"+
		"\u0421\u00cf\u0001\u0000\u0000\u0000\u0422\u0424\u0003\u00d2i\u0000\u0423"+
		"\u0422\u0001\u0000\u0000\u0000\u0423\u0424\u0001\u0000\u0000\u0000\u0424"+
		"\u0425\u0001\u0000\u0000\u0000\u0425\u0426\u0003\u00d4j\u0000\u0426\u0427"+
		"\u0003\u0140\u00a0\u0000\u0427\u0429\u0003\u00d6k\u0000\u0428\u042a\u0003"+
		"\u00dcn\u0000\u0429\u0428\u0001\u0000\u0000\u0000\u0429\u042a\u0001\u0000"+
		"\u0000\u0000\u042a\u042c\u0001\u0000\u0000\u0000\u042b\u042d\u0003\u00de"+
		"o\u0000\u042c\u042b\u0001\u0000\u0000\u0000\u042c\u042d\u0001\u0000\u0000"+
		"\u0000\u042d\u00d1\u0001\u0000\u0000\u0000\u042e\u042f\u0005P\u0000\u0000"+
		"\u042f\u00d3\u0001\u0000\u0000\u0000\u0430\u0433\u0003\u0138\u009c\u0000"+
		"\u0431\u0434\u0003\u00e0p\u0000\u0432\u0434\u00056\u0000\u0000\u0433\u0431"+
		"\u0001\u0000\u0000\u0000\u0433\u0432\u0001\u0000\u0000\u0000\u0434\u00d5"+
		"\u0001\u0000\u0000\u0000\u0435\u043e\u0005\u0010\u0000\u0000\u0436\u043b"+
		"\u0003\u00d8l\u0000\u0437\u0438\u0005\r\u0000\u0000\u0438\u043a\u0003"+
		"\u00d8l\u0000\u0439\u0437\u0001\u0000\u0000\u0000\u043a\u043d\u0001\u0000"+
		"\u0000\u0000\u043b\u0439\u0001\u0000\u0000\u0000\u043b\u043c\u0001\u0000"+
		"\u0000\u0000\u043c\u043f\u0001\u0000\u0000\u0000\u043d\u043b\u0001\u0000"+
		"\u0000\u0000\u043e\u0436\u0001\u0000\u0000\u0000\u043e\u043f\u0001\u0000"+
		"\u0000\u0000\u043f\u0440\u0001\u0000\u0000\u0000\u0440\u0441\u0005\u0011"+
		"\u0000\u0000\u0441\u00d7\u0001\u0000\u0000\u0000\u0442\u0443\u0003\u0138"+
		"\u009c\u0000\u0443\u0444\u0003\u00dam\u0000\u0444\u0445\u0003\u0138\u009c"+
		"\u0000\u0445\u0446\u0003\u00e0p\u0000\u0446\u0447\u0003\u0138\u009c\u0000"+
		"\u0447\u0448\u0003h4\u0000\u0448\u00d9\u0001\u0000\u0000\u0000\u0449\u044a"+
		"\u0007\b\u0000\u0000\u044a\u00db\u0001\u0000\u0000\u0000\u044b\u044c\u0005"+
		"5\u0000\u0000\u044c\u044d\u0005\u0010\u0000\u0000\u044d\u0452\u0003\u0016"+
		"\u000b\u0000\u044e\u044f\u0005\r\u0000\u0000\u044f\u0451\u0003\u0016\u000b"+
		"\u0000\u0450\u044e\u0001\u0000\u0000\u0000\u0451\u0454\u0001\u0000\u0000"+
		"\u0000\u0452\u0450\u0001\u0000\u0000\u0000\u0452\u0453\u0001\u0000\u0000"+
		"\u0000\u0453\u0455\u0001\u0000\u0000\u0000\u0454\u0452\u0001\u0000\u0000"+
		"\u0000\u0455\u0456\u0005\u0011\u0000\u0000\u0456\u00dd\u0001\u0000\u0000"+
		"\u0000\u0457\u0458\u0005A\u0000\u0000\u0458\u0459\u0005\u0010\u0000\u0000"+
		"\u0459\u045e\u0005\t\u0000\u0000\u045a\u045b\u0005\r\u0000\u0000\u045b"+
		"\u045d\u0005\t\u0000\u0000\u045c\u045a\u0001\u0000\u0000\u0000\u045d\u0460"+
		"\u0001\u0000\u0000\u0000\u045e\u045c\u0001\u0000\u0000\u0000\u045e\u045f"+
		"\u0001\u0000\u0000\u0000\u045f\u0461\u0001\u0000\u0000\u0000\u0460\u045e"+
		"\u0001\u0000\u0000\u0000\u0461\u0462\u0005\u0011\u0000\u0000\u0462\u00df"+
		"\u0001\u0000\u0000\u0000\u0463\u0468\u0003\\.\u0000\u0464\u0468\u0003"+
		"\u00c4b\u0000\u0465\u0468\u0003\u00c6c\u0000\u0466\u0468\u0003\u0018\f"+
		"\u0000\u0467\u0463\u0001\u0000\u0000\u0000\u0467\u0464\u0001\u0000\u0000"+
		"\u0000\u0467\u0465\u0001\u0000\u0000\u0000\u0467\u0466\u0001\u0000\u0000"+
		"\u0000\u0468\u00e1\u0001\u0000\u0000\u0000\u0469\u046a\u0005N\u0000\u0000"+
		"\u046a\u046b\u0005\u0016\u0000\u0000\u046b\u046c\u0003P(\u0000\u046c\u046d"+
		"\u0005\r\u0000\u0000\u046d\u046e\u0003P(\u0000\u046e\u046f\u0005\u0017"+
		"\u0000\u0000\u046f\u00e3\u0001\u0000\u0000\u0000\u0470\u0471\u0005N\u0000"+
		"\u0000\u0471\u00e5\u0001\u0000\u0000\u0000\u0472\u0473\u0005G\u0000\u0000"+
		"\u0473\u00e7\u0001\u0000\u0000\u0000\u0474\u0475\u00051\u0000\u0000\u0475"+
		"\u0479\u0005q\u0000\u0000\u0476\u0477\u0005O\u0000\u0000\u0477\u0479\u0005"+
		"q\u0000\u0000\u0478\u0474\u0001\u0000\u0000\u0000\u0478\u0476\u0001\u0000"+
		"\u0000\u0000\u0479\u00e9\u0001\u0000\u0000\u0000\u047a\u047b\u0003\u0138"+
		"\u009c\u0000\u047b\u047c\u00050\u0000\u0000\u047c\u047d\u0003\u0138\u009c"+
		"\u0000\u047d\u047e\u0003\u00ecv\u0000\u047e\u047f\u0005\u000b\u0000\u0000"+
		"\u047f\u00eb\u0001\u0000\u0000\u0000\u0480\u0483\u0003\u0018\f\u0000\u0481"+
		"\u0483\u0005\t\u0000\u0000\u0482\u0480\u0001\u0000\u0000\u0000\u0482\u0481"+
		"\u0001\u0000\u0000\u0000\u0483\u00ed\u0001\u0000\u0000\u0000\u0484\u0485"+
		"\u0005]\u0000\u0000\u0485\u0486\u0003\u0016\u000b\u0000\u0486\u0487\u0005"+
		"\t\u0000\u0000\u0487\u00ef\u0001\u0000\u0000\u0000\u0488\u0489\u0005\\"+
		"\u0000\u0000\u0489\u048a\u0003\u0016\u000b\u0000\u048a\u048b\u0005\t\u0000"+
		"\u0000\u048b\u00f1\u0001\u0000\u0000\u0000\u048c\u048d\u00053\u0000\u0000"+
		"\u048d\u048e\u0005^\u0000\u0000\u048e\u048f\u0003\u0138\u009c\u0000\u048f"+
		"\u0490\u0003\u00e0p\u0000\u0490\u0491\u0003\u00f4z\u0000\u0491\u00f3\u0001"+
		"\u0000\u0000\u0000\u0492\u0493\u0003\u0138\u009c\u0000\u0493\u049e\u0003"+
		"h4\u0000\u0494\u049f\u0003\u00dcn\u0000\u0495\u0496\u0005\r\u0000\u0000"+
		"\u0496\u0497\u0003\u0138\u009c\u0000\u0497\u0498\u0003h4\u0000\u0498\u049a"+
		"\u0001\u0000\u0000\u0000\u0499\u0495\u0001\u0000\u0000\u0000\u049a\u049d"+
		"\u0001\u0000\u0000\u0000\u049b\u0499\u0001\u0000\u0000\u0000\u049b\u049c"+
		"\u0001\u0000\u0000\u0000\u049c\u049f\u0001\u0000\u0000\u0000\u049d\u049b"+
		"\u0001\u0000\u0000\u0000\u049e\u0494\u0001\u0000\u0000\u0000\u049e\u049b"+
		"\u0001\u0000\u0000\u0000\u049f\u00f5\u0001\u0000\u0000\u0000\u04a0\u04a1"+
		"\u0005^\u0000\u0000\u04a1\u04a2\u0003\u0138\u009c\u0000\u04a2\u04a3\u0003"+
		"\u00e0p\u0000\u04a3\u04a4\u0003\u00f8|\u0000\u04a4\u00f7\u0001\u0000\u0000"+
		"\u0000\u04a5\u04a6\u0003\u0138\u009c\u0000\u04a6\u04b1\u0003h4\u0000\u04a7"+
		"\u04b2\u0003\u00fa}\u0000\u04a8\u04a9\u0005\r\u0000\u0000\u04a9\u04aa"+
		"\u0003\u0138\u009c\u0000\u04aa\u04ab\u0003h4\u0000\u04ab\u04ad\u0001\u0000"+
		"\u0000\u0000\u04ac\u04a8\u0001\u0000\u0000\u0000\u04ad\u04b0\u0001\u0000"+
		"\u0000\u0000\u04ae\u04ac\u0001\u0000\u0000\u0000\u04ae\u04af\u0001\u0000"+
		"\u0000\u0000\u04af\u04b2\u0001\u0000\u0000\u0000\u04b0\u04ae\u0001\u0000"+
		"\u0000\u0000\u04b1\u04a7\u0001\u0000\u0000\u0000\u04b1\u04ae\u0001\u0000"+
		"\u0000\u0000\u04b2\u00f9\u0001\u0000\u0000\u0000\u04b3\u04b5\u0003\u00fc"+
		"~\u0000\u04b4\u04b6\u0003\u00fe\u007f\u0000\u04b5\u04b4\u0001\u0000\u0000"+
		"\u0000\u04b5\u04b6\u0001\u0000\u0000\u0000\u04b6\u04b9\u0001\u0000\u0000"+
		"\u0000\u04b7\u04b9\u0003\u00fe\u007f\u0000\u04b8\u04b3\u0001\u0000\u0000"+
		"\u0000\u04b8\u04b7\u0001\u0000\u0000\u0000\u04b9\u00fb\u0001\u0000\u0000"+
		"\u0000\u04ba\u04bb\u0005E\u0000\u0000\u04bb\u04bc\u0003\u0100\u0080\u0000"+
		"\u04bc\u00fd\u0001\u0000\u0000\u0000\u04bd\u04be\u0005$\u0000\u0000\u04be"+
		"\u04bf\u0003\u0100\u0080\u0000\u04bf\u00ff\u0001\u0000\u0000\u0000\u04c0"+
		"\u04c1\u0005\u0010\u0000\u0000\u04c1\u04c6\u0003\u0016\u000b\u0000\u04c2"+
		"\u04c3\u0005\r\u0000\u0000\u04c3\u04c5\u0003\u0016\u000b\u0000\u04c4\u04c2"+
		"\u0001\u0000\u0000\u0000\u04c5\u04c8\u0001\u0000\u0000\u0000\u04c6\u04c4"+
		"\u0001\u0000\u0000\u0000\u04c6\u04c7\u0001\u0000\u0000\u0000\u04c7\u04c9"+
		"\u0001\u0000\u0000\u0000\u04c8\u04c6\u0001\u0000\u0000\u0000\u04c9\u04ca"+
		"\u0005\u0011\u0000\u0000\u04ca\u0101\u0001\u0000\u0000\u0000\u04cb\u04ce"+
		"\u0003\u0106\u0083\u0000\u04cc\u04ce\u0003\u0104\u0082\u0000\u04cd\u04cb"+
		"\u0001\u0000\u0000\u0000\u04cd\u04cc\u0001\u0000\u0000\u0000\u04ce\u0103"+
		"\u0001\u0000\u0000\u0000\u04cf\u04d0\u0005b\u0000\u0000\u04d0\u04d1\u0005"+
		"q\u0000\u0000\u04d1\u0105\u0001\u0000\u0000\u0000\u04d2\u04d3\u0003\u0108"+
		"\u0084\u0000\u04d3\u04d4\u0005\u000e\u0000\u0000\u04d4\u04d5\u0003\u010e"+
		"\u0087\u0000\u04d5\u04d6\u0005\u000f\u0000\u0000\u04d6\u0107\u0001\u0000"+
		"\u0000\u0000\u04d7\u04d8\u0005b\u0000\u0000\u04d8\u04da\u0003\u0140\u00a0"+
		"\u0000\u04d9\u04db\u0003\u010c\u0086\u0000\u04da\u04d9\u0001\u0000\u0000"+
		"\u0000\u04da\u04db\u0001\u0000\u0000\u0000\u04db\u04dd\u0001\u0000\u0000"+
		"\u0000\u04dc\u04de\u0003\u010a\u0085\u0000\u04dd\u04dc\u0001\u0000\u0000"+
		"\u0000\u04dd\u04de\u0001\u0000\u0000\u0000\u04de\u0109\u0001\u0000\u0000"+
		"\u0000\u04df\u04e0\u0005I\u0000\u0000\u04e0\u04e5\u0003\u0016\u000b\u0000"+
		"\u04e1\u04e2\u0005\r\u0000\u0000\u04e2\u04e4\u0003\u0016\u000b\u0000\u04e3"+
		"\u04e1\u0001\u0000\u0000\u0000\u04e4\u04e7\u0001\u0000\u0000\u0000\u04e5"+
		"\u04e3\u0001\u0000\u0000\u0000\u04e5\u04e6\u0001\u0000\u0000\u0000\u04e6"+
		"\u010b\u0001\u0000\u0000\u0000\u04e7\u04e5\u0001\u0000\u0000\u0000\u04e8"+
		"\u04e9\u0005\f\u0000\u0000\u04e9\u04ea\u0003\u0016\u000b\u0000\u04ea\u010d"+
		"\u0001\u0000\u0000\u0000\u04eb\u04ed\u0003\u0110\u0088\u0000\u04ec\u04eb"+
		"\u0001\u0000\u0000\u0000\u04ed\u04f0\u0001\u0000\u0000\u0000\u04ee\u04ec"+
		"\u0001\u0000\u0000\u0000\u04ee\u04ef\u0001\u0000\u0000\u0000\u04ef\u010f"+
		"\u0001\u0000\u0000\u0000\u04f0\u04ee\u0001\u0000\u0000\u0000\u04f1\u0504"+
		"\u0003\u0138\u009c\u0000\u04f2\u04f3\u0003\u0112\u0089\u0000\u04f3\u04f4"+
		"\u0005\u000b\u0000\u0000\u04f4\u0505\u0001\u0000\u0000\u0000\u04f5\u04f6"+
		"\u0003\u0116\u008b\u0000\u04f6\u04f7\u0005\u000b\u0000\u0000\u04f7\u0505"+
		"\u0001\u0000\u0000\u0000\u04f8\u04f9\u0003\u0118\u008c\u0000\u04f9\u04fa"+
		"\u0005\u000b\u0000\u0000\u04fa\u0505\u0001\u0000\u0000\u0000\u04fb\u04fc"+
		"\u0003\u011a\u008d\u0000\u04fc\u04fd\u0005\u000b\u0000\u0000\u04fd\u0505"+
		"\u0001\u0000\u0000\u0000\u04fe\u04ff\u0003\u011c\u008e\u0000\u04ff\u0500"+
		"\u0005\u000b\u0000\u0000\u0500\u0505\u0001\u0000\u0000\u0000\u0501\u0502"+
		"\u0003\u00ccf\u0000\u0502\u0503\u0005\u000b\u0000\u0000\u0503\u0505\u0001"+
		"\u0000\u0000\u0000\u0504\u04f2\u0001\u0000\u0000\u0000\u0504\u04f5\u0001"+
		"\u0000\u0000\u0000\u0504\u04f8\u0001\u0000\u0000\u0000\u0504\u04fb\u0001"+
		"\u0000\u0000\u0000\u0504\u04fe\u0001\u0000\u0000\u0000\u0504\u0501\u0001"+
		"\u0000\u0000\u0000\u0505\u0111\u0001\u0000\u0000\u0000\u0506\u0507\u0005"+
		"Y\u0000\u0000\u0507\u0508\u0003\u0114\u008a\u0000\u0508\u0509\u0005q\u0000"+
		"\u0000\u0509\u0113\u0001\u0000\u0000\u0000\u050a\u050d\u0003\u0138\u009c"+
		"\u0000\u050b\u050e\u0003\u0018\f\u0000\u050c\u050e\u0005K\u0000\u0000"+
		"\u050d\u050b\u0001\u0000\u0000\u0000\u050d\u050c\u0001\u0000\u0000\u0000"+
		"\u050e\u0115\u0001\u0000\u0000\u0000\u050f\u0511\u0005+\u0000\u0000\u0510"+
		"\u0512\u0005V\u0000\u0000\u0511\u0510\u0001\u0000\u0000\u0000\u0511\u0512"+
		"\u0001\u0000\u0000\u0000\u0512\u0513\u0001\u0000\u0000\u0000\u0513\u0514"+
		"\u0003\u0114\u008a\u0000\u0514\u0515\u0005q\u0000\u0000\u0515\u0117\u0001"+
		"\u0000\u0000\u0000\u0516\u0517\u0005&\u0000\u0000\u0517\u0518\u0003\u0016"+
		"\u000b\u0000\u0518\u0519\u0005q\u0000\u0000\u0519\u0119\u0001\u0000\u0000"+
		"\u0000\u051a\u051b\u0005)\u0000\u0000\u051b\u051c\u0003\u0016\u000b\u0000"+
		"\u051c\u051d\u0005q\u0000\u0000\u051d\u011b\u0001\u0000\u0000\u0000\u051e"+
		"\u051f\u0005Z\u0000\u0000\u051f\u0520\u0003\u0016\u000b\u0000\u0520\u0521"+
		"\u0005q\u0000\u0000\u0521\u011d\u0001\u0000\u0000\u0000\u0522\u0523\u0003"+
		"\u0120\u0090\u0000\u0523\u0524\u0003\u0126\u0093\u0000\u0524\u011f\u0001"+
		"\u0000\u0000\u0000\u0525\u0526\u0005B\u0000\u0000\u0526\u0528\u0003\u0140"+
		"\u00a0\u0000\u0527\u0529\u0003\u0122\u0091\u0000\u0528\u0527\u0001\u0000"+
		"\u0000\u0000\u0528\u0529\u0001\u0000\u0000\u0000\u0529\u052b\u0001\u0000"+
		"\u0000\u0000\u052a\u052c\u0003\u010a\u0085\u0000\u052b\u052a\u0001\u0000"+
		"\u0000\u0000\u052b\u052c\u0001\u0000\u0000\u0000\u052c\u052d\u0001\u0000"+
		"\u0000\u0000\u052d\u052e\u0005`\u0000\u0000\u052e\u0530\u0003\u0016\u000b"+
		"\u0000\u052f\u0531\u0003\u0124\u0092\u0000\u0530\u052f\u0001\u0000\u0000"+
		"\u0000\u0530\u0531\u0001\u0000\u0000\u0000\u0531\u0121\u0001\u0000\u0000"+
		"\u0000\u0532\u0533\u0005\f\u0000\u0000\u0533\u0534\u0003\u0016\u000b\u0000"+
		"\u0534\u0123\u0001\u0000\u0000\u0000\u0535\u0536\u0005,\u0000\u0000\u0536"+
		"\u0537\u0003\u0016\u000b\u0000\u0537\u0125\u0001\u0000\u0000\u0000\u0538"+
		"\u053c\u0005\u000e\u0000\u0000\u0539\u053b\u0003\u0128\u0094\u0000\u053a"+
		"\u0539\u0001\u0000\u0000\u0000\u053b\u053e\u0001\u0000\u0000\u0000\u053c"+
		"\u053a\u0001\u0000\u0000\u0000\u053c\u053d\u0001\u0000\u0000\u0000\u053d"+
		"\u053f\u0001\u0000\u0000\u0000\u053e\u053c\u0001\u0000\u0000\u0000\u053f"+
		"\u0540\u0005\u000f\u0000\u0000\u0540\u0127\u0001\u0000\u0000\u0000\u0541"+
		"\u054a\u0003\u0010\b\u0000\u0542\u0545\u0003\u0138\u009c\u0000\u0543\u0546"+
		"\u0003\u012a\u0095\u0000\u0544\u0546\u0003\u012c\u0096\u0000\u0545\u0543"+
		"\u0001\u0000\u0000\u0000\u0545\u0544\u0001\u0000\u0000\u0000\u0546\u0547"+
		"\u0001\u0000\u0000\u0000\u0547\u0548\u0005\u000b\u0000\u0000\u0548\u054a"+
		"\u0001\u0000\u0000\u0000\u0549\u0541\u0001\u0000\u0000\u0000\u0549\u0542"+
		"\u0001\u0000\u0000\u0000\u054a\u0129\u0001\u0000\u0000\u0000\u054b\u054c"+
		"\u0005C\u0000\u0000\u054c\u054d\u0003\u0140\u00a0\u0000\u054d\u054f\u0005"+
		"\u0010\u0000\u0000\u054e\u0550\u00030\u0018\u0000\u054f\u054e\u0001\u0000"+
		"\u0000\u0000\u054f\u0550\u0001\u0000\u0000\u0000\u0550\u0551\u0001\u0000"+
		"\u0000\u0000\u0551\u0553\u0005\u0011\u0000\u0000\u0552\u0554\u0003\u00dc"+
		"n\u0000\u0553\u0552\u0001\u0000\u0000\u0000\u0553\u0554\u0001\u0000\u0000"+
		"\u0000\u0554\u012b\u0001\u0000\u0000\u0000\u0555\u0556\u00054\u0000\u0000"+
		"\u0556\u0557\u0003\u0140\u00a0\u0000\u0557\u0559\u0005\u0010\u0000\u0000"+
		"\u0558\u055a\u00030\u0018\u0000\u0559\u0558\u0001\u0000\u0000\u0000\u0559"+
		"\u055a\u0001\u0000\u0000\u0000\u055a\u055b\u0001\u0000\u0000\u0000\u055b"+
		"\u055d\u0005\u0011\u0000\u0000\u055c\u055e\u0003\u00dcn\u0000\u055d\u055c"+
		"\u0001\u0000\u0000\u0000\u055d\u055e\u0001\u0000\u0000\u0000\u055e\u012d"+
		"\u0001\u0000\u0000\u0000\u055f\u0563\u0003\u0134\u009a\u0000\u0560\u0563"+
		"\u0003\u0132\u0099\u0000\u0561\u0563\u0003\u0130\u0098\u0000\u0562\u055f"+
		"\u0001\u0000\u0000\u0000\u0562\u0560\u0001\u0000\u0000\u0000\u0562\u0561"+
		"\u0001\u0000\u0000\u0000\u0563\u012f\u0001\u0000\u0000\u0000\u0564\u0566"+
		"\u0005W\u0000\u0000\u0565\u0564\u0001\u0000\u0000\u0000\u0565\u0566\u0001"+
		"\u0000\u0000\u0000\u0566\u0567\u0001\u0000\u0000\u0000\u0567\u0568\u0005"+
		"8\u0000\u0000\u0568\u0569\u0005q\u0000\u0000\u0569\u0131\u0001\u0000\u0000"+
		"\u0000\u056a\u056b\u0005W\u0000\u0000\u056b\u056c\u00058\u0000\u0000\u056c"+
		"\u056d\u0003\u0140\u00a0\u0000\u056d\u056e\u0003&\u0013\u0000\u056e\u0572"+
		"\u0005\u000e\u0000\u0000\u056f\u0571\u0003\u0010\b\u0000\u0570\u056f\u0001"+
		"\u0000\u0000\u0000\u0571\u0574\u0001\u0000\u0000\u0000\u0572\u0570\u0001"+
		"\u0000\u0000\u0000\u0572\u0573\u0001\u0000\u0000\u0000\u0573\u0575\u0001"+
		"\u0000\u0000\u0000\u0574\u0572\u0001\u0000\u0000\u0000\u0575\u0576\u0005"+
		"\u000f\u0000\u0000\u0576\u0133\u0001\u0000\u0000\u0000\u0577\u0578\u0003"+
		"\u0136\u009b\u0000\u0578\u057c\u0005\u000e\u0000\u0000\u0579\u057b\u0003"+
		"*\u0015\u0000\u057a\u0579\u0001\u0000\u0000\u0000\u057b\u057e\u0001\u0000"+
		"\u0000\u0000\u057c\u057a\u0001\u0000\u0000\u0000\u057c\u057d\u0001\u0000"+
		"\u0000\u0000\u057d\u057f\u0001\u0000\u0000\u0000\u057e\u057c\u0001\u0000"+
		"\u0000\u0000\u057f\u0580\u0005\u000f\u0000\u0000\u0580\u0135\u0001\u0000"+
		"\u0000\u0000\u0581\u0583\u0005-\u0000\u0000\u0582\u0581\u0001\u0000\u0000"+
		"\u0000\u0582\u0583\u0001\u0000\u0000\u0000\u0583\u0584\u0001\u0000\u0000"+
		"\u0000\u0584\u0585\u00058\u0000\u0000\u0585\u0586\u0003\u0140\u00a0\u0000"+
		"\u0586\u0587\u0003&\u0013\u0000\u0587\u0137\u0001\u0000\u0000\u0000\u0588"+
		"\u058a\u0003\u013a\u009d\u0000\u0589\u0588\u0001\u0000\u0000\u0000\u058a"+
		"\u058d\u0001\u0000\u0000\u0000\u058b\u0589\u0001\u0000\u0000\u0000\u058b"+
		"\u058c\u0001\u0000\u0000\u0000\u058c\u0139\u0001\u0000\u0000\u0000\u058d"+
		"\u058b\u0001\u0000\u0000\u0000\u058e\u058f\u0005#\u0000\u0000\u058f\u0594"+
		"\u0003\u0018\f\u0000\u0590\u0591\u0005\u0010\u0000\u0000\u0591\u0592\u0003"+
		"\u013c\u009e\u0000\u0592\u0593\u0005\u0011\u0000\u0000\u0593\u0595\u0001"+
		"\u0000\u0000\u0000\u0594\u0590\u0001\u0000\u0000\u0000\u0594\u0595\u0001"+
		"\u0000\u0000\u0000\u0595\u013b\u0001\u0000\u0000\u0000\u0596\u05a0\u0003"+
		":\u001d\u0000\u0597\u059c\u0003\u013e\u009f\u0000\u0598\u0599\u0005\r"+
		"\u0000\u0000\u0599\u059b\u0003\u013e\u009f\u0000\u059a\u0598\u0001\u0000"+
		"\u0000\u0000\u059b\u059e\u0001\u0000\u0000\u0000\u059c\u059a\u0001\u0000"+
		"\u0000\u0000\u059c\u059d\u0001\u0000\u0000\u0000\u059d\u05a0\u0001\u0000"+
		"\u0000\u0000\u059e\u059c\u0001\u0000\u0000\u0000\u059f\u0596\u0001\u0000"+
		"\u0000\u0000\u059f\u0597\u0001\u0000\u0000\u0000\u05a0\u013d\u0001\u0000"+
		"\u0000\u0000\u05a1\u05a2\u0005q\u0000\u0000\u05a2\u05a3\u0005\u001e\u0000"+
		"\u0000\u05a3\u05a4\u0003:\u001d\u0000\u05a4\u013f\u0001\u0000\u0000\u0000"+
		"\u05a5\u05a6\u0003\u0138\u009c\u0000\u05a6\u05a7\u0005q\u0000\u0000\u05a7"+
		"\u0141\u0001\u0000\u0000\u0000}\u0145\u014b\u0174\u017c\u0183\u018b\u0191"+
		"\u0196\u019b\u01b4\u01bc\u01c5\u01cc\u01d4\u01d7\u01e8\u01f2\u01f8\u0200"+
		"\u0207\u020a\u0212\u0215\u021c\u0223\u022e\u0232\u023b\u0258\u0261\u0269"+
		"\u0271\u0279\u0281\u0289\u0290\u029a\u02ae\u02b5\u02ba\u02bf\u02ca\u02d2"+
		"\u02d9\u02e0\u02e8\u02ee\u02f8\u02fc\u0302\u030d\u0313\u031a\u031f\u0325"+
		"\u0335\u033f\u034f\u0351\u0358\u0363\u036b\u0371\u0379\u0388\u038f\u0398"+
		"\u03b2\u03b7\u03bc\u03c8\u03d6\u03e3\u03ec\u03f7\u0400\u0407\u040d\u0415"+
		"\u041d\u0423\u0429\u042c\u0433\u043b\u043e\u0452\u045e\u0467\u0478\u0482"+
		"\u049b\u049e\u04ae\u04b1\u04b5\u04b8\u04c6\u04cd\u04da\u04dd\u04e5\u04ee"+
		"\u0504\u050d\u0511\u0528\u052b\u0530\u053c\u0545\u0549\u054f\u0553\u0559"+
		"\u055d\u0562\u0565\u0572\u057c\u0582\u058b\u0594\u059c\u059f";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}