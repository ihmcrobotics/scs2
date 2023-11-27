// Generated from IDL.g4 by ANTLR 4.13.1
package us.ihmc.scs2.session.mcap.omgidl_parser;
import org.antlr.v4.runtime.tree.ParseTreeListener;

/**
 * This interface defines a complete listener for a parse tree produced by
 * {@link IDLParser}.
 */
public interface IDLListener extends ParseTreeListener {
	/**
	 * Enter a parse tree produced by {@link IDLParser#specification}.
	 * @param ctx the parse tree
	 */
	void enterSpecification(IDLParser.SpecificationContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#specification}.
	 * @param ctx the parse tree
	 */
	void exitSpecification(IDLParser.SpecificationContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#definition}.
	 * @param ctx the parse tree
	 */
	void enterDefinition(IDLParser.DefinitionContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#definition}.
	 * @param ctx the parse tree
	 */
	void exitDefinition(IDLParser.DefinitionContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#module}.
	 * @param ctx the parse tree
	 */
	void enterModule(IDLParser.ModuleContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#module}.
	 * @param ctx the parse tree
	 */
	void exitModule(IDLParser.ModuleContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#interface_or_forward_decl}.
	 * @param ctx the parse tree
	 */
	void enterInterface_or_forward_decl(IDLParser.Interface_or_forward_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#interface_or_forward_decl}.
	 * @param ctx the parse tree
	 */
	void exitInterface_or_forward_decl(IDLParser.Interface_or_forward_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#interface_decl}.
	 * @param ctx the parse tree
	 */
	void enterInterface_decl(IDLParser.Interface_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#interface_decl}.
	 * @param ctx the parse tree
	 */
	void exitInterface_decl(IDLParser.Interface_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#forward_decl}.
	 * @param ctx the parse tree
	 */
	void enterForward_decl(IDLParser.Forward_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#forward_decl}.
	 * @param ctx the parse tree
	 */
	void exitForward_decl(IDLParser.Forward_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#interface_header}.
	 * @param ctx the parse tree
	 */
	void enterInterface_header(IDLParser.Interface_headerContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#interface_header}.
	 * @param ctx the parse tree
	 */
	void exitInterface_header(IDLParser.Interface_headerContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#interface_body}.
	 * @param ctx the parse tree
	 */
	void enterInterface_body(IDLParser.Interface_bodyContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#interface_body}.
	 * @param ctx the parse tree
	 */
	void exitInterface_body(IDLParser.Interface_bodyContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#export_}.
	 * @param ctx the parse tree
	 */
	void enterExport_(IDLParser.Export_Context ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#export_}.
	 * @param ctx the parse tree
	 */
	void exitExport_(IDLParser.Export_Context ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#interface_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void enterInterface_inheritance_spec(IDLParser.Interface_inheritance_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#interface_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void exitInterface_inheritance_spec(IDLParser.Interface_inheritance_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#interface_name}.
	 * @param ctx the parse tree
	 */
	void enterInterface_name(IDLParser.Interface_nameContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#interface_name}.
	 * @param ctx the parse tree
	 */
	void exitInterface_name(IDLParser.Interface_nameContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#a_scoped_name}.
	 * @param ctx the parse tree
	 */
	void enterA_scoped_name(IDLParser.A_scoped_nameContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#a_scoped_name}.
	 * @param ctx the parse tree
	 */
	void exitA_scoped_name(IDLParser.A_scoped_nameContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#scoped_name}.
	 * @param ctx the parse tree
	 */
	void enterScoped_name(IDLParser.Scoped_nameContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#scoped_name}.
	 * @param ctx the parse tree
	 */
	void exitScoped_name(IDLParser.Scoped_nameContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value}.
	 * @param ctx the parse tree
	 */
	void enterValue(IDLParser.ValueContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value}.
	 * @param ctx the parse tree
	 */
	void exitValue(IDLParser.ValueContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_forward_decl}.
	 * @param ctx the parse tree
	 */
	void enterValue_forward_decl(IDLParser.Value_forward_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_forward_decl}.
	 * @param ctx the parse tree
	 */
	void exitValue_forward_decl(IDLParser.Value_forward_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_box_decl}.
	 * @param ctx the parse tree
	 */
	void enterValue_box_decl(IDLParser.Value_box_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_box_decl}.
	 * @param ctx the parse tree
	 */
	void exitValue_box_decl(IDLParser.Value_box_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_abs_decl}.
	 * @param ctx the parse tree
	 */
	void enterValue_abs_decl(IDLParser.Value_abs_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_abs_decl}.
	 * @param ctx the parse tree
	 */
	void exitValue_abs_decl(IDLParser.Value_abs_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_decl}.
	 * @param ctx the parse tree
	 */
	void enterValue_decl(IDLParser.Value_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_decl}.
	 * @param ctx the parse tree
	 */
	void exitValue_decl(IDLParser.Value_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_header}.
	 * @param ctx the parse tree
	 */
	void enterValue_header(IDLParser.Value_headerContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_header}.
	 * @param ctx the parse tree
	 */
	void exitValue_header(IDLParser.Value_headerContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void enterValue_inheritance_spec(IDLParser.Value_inheritance_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void exitValue_inheritance_spec(IDLParser.Value_inheritance_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_name}.
	 * @param ctx the parse tree
	 */
	void enterValue_name(IDLParser.Value_nameContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_name}.
	 * @param ctx the parse tree
	 */
	void exitValue_name(IDLParser.Value_nameContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_element}.
	 * @param ctx the parse tree
	 */
	void enterValue_element(IDLParser.Value_elementContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_element}.
	 * @param ctx the parse tree
	 */
	void exitValue_element(IDLParser.Value_elementContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#state_member}.
	 * @param ctx the parse tree
	 */
	void enterState_member(IDLParser.State_memberContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#state_member}.
	 * @param ctx the parse tree
	 */
	void exitState_member(IDLParser.State_memberContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#init_decl}.
	 * @param ctx the parse tree
	 */
	void enterInit_decl(IDLParser.Init_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#init_decl}.
	 * @param ctx the parse tree
	 */
	void exitInit_decl(IDLParser.Init_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#init_param_decls}.
	 * @param ctx the parse tree
	 */
	void enterInit_param_decls(IDLParser.Init_param_declsContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#init_param_decls}.
	 * @param ctx the parse tree
	 */
	void exitInit_param_decls(IDLParser.Init_param_declsContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#init_param_decl}.
	 * @param ctx the parse tree
	 */
	void enterInit_param_decl(IDLParser.Init_param_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#init_param_decl}.
	 * @param ctx the parse tree
	 */
	void exitInit_param_decl(IDLParser.Init_param_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#init_param_attribute}.
	 * @param ctx the parse tree
	 */
	void enterInit_param_attribute(IDLParser.Init_param_attributeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#init_param_attribute}.
	 * @param ctx the parse tree
	 */
	void exitInit_param_attribute(IDLParser.Init_param_attributeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#const_decl}.
	 * @param ctx the parse tree
	 */
	void enterConst_decl(IDLParser.Const_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#const_decl}.
	 * @param ctx the parse tree
	 */
	void exitConst_decl(IDLParser.Const_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#const_type}.
	 * @param ctx the parse tree
	 */
	void enterConst_type(IDLParser.Const_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#const_type}.
	 * @param ctx the parse tree
	 */
	void exitConst_type(IDLParser.Const_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#const_exp}.
	 * @param ctx the parse tree
	 */
	void enterConst_exp(IDLParser.Const_expContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#const_exp}.
	 * @param ctx the parse tree
	 */
	void exitConst_exp(IDLParser.Const_expContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#or_expr}.
	 * @param ctx the parse tree
	 */
	void enterOr_expr(IDLParser.Or_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#or_expr}.
	 * @param ctx the parse tree
	 */
	void exitOr_expr(IDLParser.Or_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#xor_expr}.
	 * @param ctx the parse tree
	 */
	void enterXor_expr(IDLParser.Xor_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#xor_expr}.
	 * @param ctx the parse tree
	 */
	void exitXor_expr(IDLParser.Xor_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#and_expr}.
	 * @param ctx the parse tree
	 */
	void enterAnd_expr(IDLParser.And_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#and_expr}.
	 * @param ctx the parse tree
	 */
	void exitAnd_expr(IDLParser.And_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#shift_expr}.
	 * @param ctx the parse tree
	 */
	void enterShift_expr(IDLParser.Shift_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#shift_expr}.
	 * @param ctx the parse tree
	 */
	void exitShift_expr(IDLParser.Shift_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#add_expr}.
	 * @param ctx the parse tree
	 */
	void enterAdd_expr(IDLParser.Add_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#add_expr}.
	 * @param ctx the parse tree
	 */
	void exitAdd_expr(IDLParser.Add_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#mult_expr}.
	 * @param ctx the parse tree
	 */
	void enterMult_expr(IDLParser.Mult_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#mult_expr}.
	 * @param ctx the parse tree
	 */
	void exitMult_expr(IDLParser.Mult_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#unary_expr}.
	 * @param ctx the parse tree
	 */
	void enterUnary_expr(IDLParser.Unary_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#unary_expr}.
	 * @param ctx the parse tree
	 */
	void exitUnary_expr(IDLParser.Unary_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#unary_operator}.
	 * @param ctx the parse tree
	 */
	void enterUnary_operator(IDLParser.Unary_operatorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#unary_operator}.
	 * @param ctx the parse tree
	 */
	void exitUnary_operator(IDLParser.Unary_operatorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#primary_expr}.
	 * @param ctx the parse tree
	 */
	void enterPrimary_expr(IDLParser.Primary_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#primary_expr}.
	 * @param ctx the parse tree
	 */
	void exitPrimary_expr(IDLParser.Primary_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#literal}.
	 * @param ctx the parse tree
	 */
	void enterLiteral(IDLParser.LiteralContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#literal}.
	 * @param ctx the parse tree
	 */
	void exitLiteral(IDLParser.LiteralContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#positive_int_const}.
	 * @param ctx the parse tree
	 */
	void enterPositive_int_const(IDLParser.Positive_int_constContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#positive_int_const}.
	 * @param ctx the parse tree
	 */
	void exitPositive_int_const(IDLParser.Positive_int_constContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#type_decl}.
	 * @param ctx the parse tree
	 */
	void enterType_decl(IDLParser.Type_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#type_decl}.
	 * @param ctx the parse tree
	 */
	void exitType_decl(IDLParser.Type_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#type_declarator}.
	 * @param ctx the parse tree
	 */
	void enterType_declarator(IDLParser.Type_declaratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#type_declarator}.
	 * @param ctx the parse tree
	 */
	void exitType_declarator(IDLParser.Type_declaratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#type_spec}.
	 * @param ctx the parse tree
	 */
	void enterType_spec(IDLParser.Type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#type_spec}.
	 * @param ctx the parse tree
	 */
	void exitType_spec(IDLParser.Type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#simple_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterSimple_type_spec(IDLParser.Simple_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#simple_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitSimple_type_spec(IDLParser.Simple_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#bitfield_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterBitfield_type_spec(IDLParser.Bitfield_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#bitfield_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitBitfield_type_spec(IDLParser.Bitfield_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#base_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterBase_type_spec(IDLParser.Base_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#base_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitBase_type_spec(IDLParser.Base_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#template_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterTemplate_type_spec(IDLParser.Template_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#template_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitTemplate_type_spec(IDLParser.Template_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#constr_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterConstr_type_spec(IDLParser.Constr_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#constr_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitConstr_type_spec(IDLParser.Constr_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#simple_declarators}.
	 * @param ctx the parse tree
	 */
	void enterSimple_declarators(IDLParser.Simple_declaratorsContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#simple_declarators}.
	 * @param ctx the parse tree
	 */
	void exitSimple_declarators(IDLParser.Simple_declaratorsContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#declarators}.
	 * @param ctx the parse tree
	 */
	void enterDeclarators(IDLParser.DeclaratorsContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#declarators}.
	 * @param ctx the parse tree
	 */
	void exitDeclarators(IDLParser.DeclaratorsContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#declarator}.
	 * @param ctx the parse tree
	 */
	void enterDeclarator(IDLParser.DeclaratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#declarator}.
	 * @param ctx the parse tree
	 */
	void exitDeclarator(IDLParser.DeclaratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#simple_declarator}.
	 * @param ctx the parse tree
	 */
	void enterSimple_declarator(IDLParser.Simple_declaratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#simple_declarator}.
	 * @param ctx the parse tree
	 */
	void exitSimple_declarator(IDLParser.Simple_declaratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#complex_declarator}.
	 * @param ctx the parse tree
	 */
	void enterComplex_declarator(IDLParser.Complex_declaratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#complex_declarator}.
	 * @param ctx the parse tree
	 */
	void exitComplex_declarator(IDLParser.Complex_declaratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#floating_pt_type}.
	 * @param ctx the parse tree
	 */
	void enterFloating_pt_type(IDLParser.Floating_pt_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#floating_pt_type}.
	 * @param ctx the parse tree
	 */
	void exitFloating_pt_type(IDLParser.Floating_pt_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#integer_type}.
	 * @param ctx the parse tree
	 */
	void enterInteger_type(IDLParser.Integer_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#integer_type}.
	 * @param ctx the parse tree
	 */
	void exitInteger_type(IDLParser.Integer_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#signed_int}.
	 * @param ctx the parse tree
	 */
	void enterSigned_int(IDLParser.Signed_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#signed_int}.
	 * @param ctx the parse tree
	 */
	void exitSigned_int(IDLParser.Signed_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#signed_tiny_int}.
	 * @param ctx the parse tree
	 */
	void enterSigned_tiny_int(IDLParser.Signed_tiny_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#signed_tiny_int}.
	 * @param ctx the parse tree
	 */
	void exitSigned_tiny_int(IDLParser.Signed_tiny_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#signed_short_int}.
	 * @param ctx the parse tree
	 */
	void enterSigned_short_int(IDLParser.Signed_short_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#signed_short_int}.
	 * @param ctx the parse tree
	 */
	void exitSigned_short_int(IDLParser.Signed_short_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#signed_long_int}.
	 * @param ctx the parse tree
	 */
	void enterSigned_long_int(IDLParser.Signed_long_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#signed_long_int}.
	 * @param ctx the parse tree
	 */
	void exitSigned_long_int(IDLParser.Signed_long_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#signed_longlong_int}.
	 * @param ctx the parse tree
	 */
	void enterSigned_longlong_int(IDLParser.Signed_longlong_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#signed_longlong_int}.
	 * @param ctx the parse tree
	 */
	void exitSigned_longlong_int(IDLParser.Signed_longlong_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#unsigned_int}.
	 * @param ctx the parse tree
	 */
	void enterUnsigned_int(IDLParser.Unsigned_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#unsigned_int}.
	 * @param ctx the parse tree
	 */
	void exitUnsigned_int(IDLParser.Unsigned_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#unsigned_tiny_int}.
	 * @param ctx the parse tree
	 */
	void enterUnsigned_tiny_int(IDLParser.Unsigned_tiny_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#unsigned_tiny_int}.
	 * @param ctx the parse tree
	 */
	void exitUnsigned_tiny_int(IDLParser.Unsigned_tiny_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#unsigned_short_int}.
	 * @param ctx the parse tree
	 */
	void enterUnsigned_short_int(IDLParser.Unsigned_short_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#unsigned_short_int}.
	 * @param ctx the parse tree
	 */
	void exitUnsigned_short_int(IDLParser.Unsigned_short_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#unsigned_long_int}.
	 * @param ctx the parse tree
	 */
	void enterUnsigned_long_int(IDLParser.Unsigned_long_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#unsigned_long_int}.
	 * @param ctx the parse tree
	 */
	void exitUnsigned_long_int(IDLParser.Unsigned_long_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#unsigned_longlong_int}.
	 * @param ctx the parse tree
	 */
	void enterUnsigned_longlong_int(IDLParser.Unsigned_longlong_intContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#unsigned_longlong_int}.
	 * @param ctx the parse tree
	 */
	void exitUnsigned_longlong_int(IDLParser.Unsigned_longlong_intContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#char_type}.
	 * @param ctx the parse tree
	 */
	void enterChar_type(IDLParser.Char_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#char_type}.
	 * @param ctx the parse tree
	 */
	void exitChar_type(IDLParser.Char_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#wide_char_type}.
	 * @param ctx the parse tree
	 */
	void enterWide_char_type(IDLParser.Wide_char_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#wide_char_type}.
	 * @param ctx the parse tree
	 */
	void exitWide_char_type(IDLParser.Wide_char_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#boolean_type}.
	 * @param ctx the parse tree
	 */
	void enterBoolean_type(IDLParser.Boolean_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#boolean_type}.
	 * @param ctx the parse tree
	 */
	void exitBoolean_type(IDLParser.Boolean_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#octet_type}.
	 * @param ctx the parse tree
	 */
	void enterOctet_type(IDLParser.Octet_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#octet_type}.
	 * @param ctx the parse tree
	 */
	void exitOctet_type(IDLParser.Octet_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#any_type}.
	 * @param ctx the parse tree
	 */
	void enterAny_type(IDLParser.Any_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#any_type}.
	 * @param ctx the parse tree
	 */
	void exitAny_type(IDLParser.Any_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#object_type}.
	 * @param ctx the parse tree
	 */
	void enterObject_type(IDLParser.Object_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#object_type}.
	 * @param ctx the parse tree
	 */
	void exitObject_type(IDLParser.Object_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_decl}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_decl(IDLParser.Annotation_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_decl}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_decl(IDLParser.Annotation_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_def}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_def(IDLParser.Annotation_defContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_def}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_def(IDLParser.Annotation_defContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_header}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_header(IDLParser.Annotation_headerContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_header}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_header(IDLParser.Annotation_headerContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_inheritance_spec(IDLParser.Annotation_inheritance_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_inheritance_spec(IDLParser.Annotation_inheritance_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_body}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_body(IDLParser.Annotation_bodyContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_body}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_body(IDLParser.Annotation_bodyContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_member}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_member(IDLParser.Annotation_memberContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_member}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_member(IDLParser.Annotation_memberContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_forward_dcl}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_forward_dcl(IDLParser.Annotation_forward_dclContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_forward_dcl}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_forward_dcl(IDLParser.Annotation_forward_dclContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#bitset_type}.
	 * @param ctx the parse tree
	 */
	void enterBitset_type(IDLParser.Bitset_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#bitset_type}.
	 * @param ctx the parse tree
	 */
	void exitBitset_type(IDLParser.Bitset_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#bitfield}.
	 * @param ctx the parse tree
	 */
	void enterBitfield(IDLParser.BitfieldContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#bitfield}.
	 * @param ctx the parse tree
	 */
	void exitBitfield(IDLParser.BitfieldContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#bitfield_spec}.
	 * @param ctx the parse tree
	 */
	void enterBitfield_spec(IDLParser.Bitfield_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#bitfield_spec}.
	 * @param ctx the parse tree
	 */
	void exitBitfield_spec(IDLParser.Bitfield_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#bitmask_type}.
	 * @param ctx the parse tree
	 */
	void enterBitmask_type(IDLParser.Bitmask_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#bitmask_type}.
	 * @param ctx the parse tree
	 */
	void exitBitmask_type(IDLParser.Bitmask_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#bit_values}.
	 * @param ctx the parse tree
	 */
	void enterBit_values(IDLParser.Bit_valuesContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#bit_values}.
	 * @param ctx the parse tree
	 */
	void exitBit_values(IDLParser.Bit_valuesContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#struct_type}.
	 * @param ctx the parse tree
	 */
	void enterStruct_type(IDLParser.Struct_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#struct_type}.
	 * @param ctx the parse tree
	 */
	void exitStruct_type(IDLParser.Struct_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#member_list}.
	 * @param ctx the parse tree
	 */
	void enterMember_list(IDLParser.Member_listContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#member_list}.
	 * @param ctx the parse tree
	 */
	void exitMember_list(IDLParser.Member_listContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#member}.
	 * @param ctx the parse tree
	 */
	void enterMember(IDLParser.MemberContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#member}.
	 * @param ctx the parse tree
	 */
	void exitMember(IDLParser.MemberContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#union_type}.
	 * @param ctx the parse tree
	 */
	void enterUnion_type(IDLParser.Union_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#union_type}.
	 * @param ctx the parse tree
	 */
	void exitUnion_type(IDLParser.Union_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#switch_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterSwitch_type_spec(IDLParser.Switch_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#switch_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitSwitch_type_spec(IDLParser.Switch_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#switch_body}.
	 * @param ctx the parse tree
	 */
	void enterSwitch_body(IDLParser.Switch_bodyContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#switch_body}.
	 * @param ctx the parse tree
	 */
	void exitSwitch_body(IDLParser.Switch_bodyContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#case_stmt}.
	 * @param ctx the parse tree
	 */
	void enterCase_stmt(IDLParser.Case_stmtContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#case_stmt}.
	 * @param ctx the parse tree
	 */
	void exitCase_stmt(IDLParser.Case_stmtContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#case_label}.
	 * @param ctx the parse tree
	 */
	void enterCase_label(IDLParser.Case_labelContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#case_label}.
	 * @param ctx the parse tree
	 */
	void exitCase_label(IDLParser.Case_labelContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#element_spec}.
	 * @param ctx the parse tree
	 */
	void enterElement_spec(IDLParser.Element_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#element_spec}.
	 * @param ctx the parse tree
	 */
	void exitElement_spec(IDLParser.Element_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#enum_type}.
	 * @param ctx the parse tree
	 */
	void enterEnum_type(IDLParser.Enum_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#enum_type}.
	 * @param ctx the parse tree
	 */
	void exitEnum_type(IDLParser.Enum_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#enumerator}.
	 * @param ctx the parse tree
	 */
	void enterEnumerator(IDLParser.EnumeratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#enumerator}.
	 * @param ctx the parse tree
	 */
	void exitEnumerator(IDLParser.EnumeratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#sequence_type}.
	 * @param ctx the parse tree
	 */
	void enterSequence_type(IDLParser.Sequence_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#sequence_type}.
	 * @param ctx the parse tree
	 */
	void exitSequence_type(IDLParser.Sequence_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#set_type}.
	 * @param ctx the parse tree
	 */
	void enterSet_type(IDLParser.Set_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#set_type}.
	 * @param ctx the parse tree
	 */
	void exitSet_type(IDLParser.Set_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#map_type}.
	 * @param ctx the parse tree
	 */
	void enterMap_type(IDLParser.Map_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#map_type}.
	 * @param ctx the parse tree
	 */
	void exitMap_type(IDLParser.Map_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#string_type}.
	 * @param ctx the parse tree
	 */
	void enterString_type(IDLParser.String_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#string_type}.
	 * @param ctx the parse tree
	 */
	void exitString_type(IDLParser.String_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#wide_string_type}.
	 * @param ctx the parse tree
	 */
	void enterWide_string_type(IDLParser.Wide_string_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#wide_string_type}.
	 * @param ctx the parse tree
	 */
	void exitWide_string_type(IDLParser.Wide_string_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#array_declarator}.
	 * @param ctx the parse tree
	 */
	void enterArray_declarator(IDLParser.Array_declaratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#array_declarator}.
	 * @param ctx the parse tree
	 */
	void exitArray_declarator(IDLParser.Array_declaratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#fixed_array_size}.
	 * @param ctx the parse tree
	 */
	void enterFixed_array_size(IDLParser.Fixed_array_sizeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#fixed_array_size}.
	 * @param ctx the parse tree
	 */
	void exitFixed_array_size(IDLParser.Fixed_array_sizeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#attr_decl}.
	 * @param ctx the parse tree
	 */
	void enterAttr_decl(IDLParser.Attr_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#attr_decl}.
	 * @param ctx the parse tree
	 */
	void exitAttr_decl(IDLParser.Attr_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#except_decl}.
	 * @param ctx the parse tree
	 */
	void enterExcept_decl(IDLParser.Except_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#except_decl}.
	 * @param ctx the parse tree
	 */
	void exitExcept_decl(IDLParser.Except_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#op_decl}.
	 * @param ctx the parse tree
	 */
	void enterOp_decl(IDLParser.Op_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#op_decl}.
	 * @param ctx the parse tree
	 */
	void exitOp_decl(IDLParser.Op_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#op_attribute}.
	 * @param ctx the parse tree
	 */
	void enterOp_attribute(IDLParser.Op_attributeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#op_attribute}.
	 * @param ctx the parse tree
	 */
	void exitOp_attribute(IDLParser.Op_attributeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#op_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterOp_type_spec(IDLParser.Op_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#op_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitOp_type_spec(IDLParser.Op_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#parameter_decls}.
	 * @param ctx the parse tree
	 */
	void enterParameter_decls(IDLParser.Parameter_declsContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#parameter_decls}.
	 * @param ctx the parse tree
	 */
	void exitParameter_decls(IDLParser.Parameter_declsContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#param_decl}.
	 * @param ctx the parse tree
	 */
	void enterParam_decl(IDLParser.Param_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#param_decl}.
	 * @param ctx the parse tree
	 */
	void exitParam_decl(IDLParser.Param_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#param_attribute}.
	 * @param ctx the parse tree
	 */
	void enterParam_attribute(IDLParser.Param_attributeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#param_attribute}.
	 * @param ctx the parse tree
	 */
	void exitParam_attribute(IDLParser.Param_attributeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#raises_expr}.
	 * @param ctx the parse tree
	 */
	void enterRaises_expr(IDLParser.Raises_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#raises_expr}.
	 * @param ctx the parse tree
	 */
	void exitRaises_expr(IDLParser.Raises_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#context_expr}.
	 * @param ctx the parse tree
	 */
	void enterContext_expr(IDLParser.Context_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#context_expr}.
	 * @param ctx the parse tree
	 */
	void exitContext_expr(IDLParser.Context_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#param_type_spec}.
	 * @param ctx the parse tree
	 */
	void enterParam_type_spec(IDLParser.Param_type_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#param_type_spec}.
	 * @param ctx the parse tree
	 */
	void exitParam_type_spec(IDLParser.Param_type_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#fixed_pt_type}.
	 * @param ctx the parse tree
	 */
	void enterFixed_pt_type(IDLParser.Fixed_pt_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#fixed_pt_type}.
	 * @param ctx the parse tree
	 */
	void exitFixed_pt_type(IDLParser.Fixed_pt_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#fixed_pt_const_type}.
	 * @param ctx the parse tree
	 */
	void enterFixed_pt_const_type(IDLParser.Fixed_pt_const_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#fixed_pt_const_type}.
	 * @param ctx the parse tree
	 */
	void exitFixed_pt_const_type(IDLParser.Fixed_pt_const_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#value_base_type}.
	 * @param ctx the parse tree
	 */
	void enterValue_base_type(IDLParser.Value_base_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#value_base_type}.
	 * @param ctx the parse tree
	 */
	void exitValue_base_type(IDLParser.Value_base_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#constr_forward_decl}.
	 * @param ctx the parse tree
	 */
	void enterConstr_forward_decl(IDLParser.Constr_forward_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#constr_forward_decl}.
	 * @param ctx the parse tree
	 */
	void exitConstr_forward_decl(IDLParser.Constr_forward_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#import_decl}.
	 * @param ctx the parse tree
	 */
	void enterImport_decl(IDLParser.Import_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#import_decl}.
	 * @param ctx the parse tree
	 */
	void exitImport_decl(IDLParser.Import_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#imported_scope}.
	 * @param ctx the parse tree
	 */
	void enterImported_scope(IDLParser.Imported_scopeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#imported_scope}.
	 * @param ctx the parse tree
	 */
	void exitImported_scope(IDLParser.Imported_scopeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#type_id_decl}.
	 * @param ctx the parse tree
	 */
	void enterType_id_decl(IDLParser.Type_id_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#type_id_decl}.
	 * @param ctx the parse tree
	 */
	void exitType_id_decl(IDLParser.Type_id_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#type_prefix_decl}.
	 * @param ctx the parse tree
	 */
	void enterType_prefix_decl(IDLParser.Type_prefix_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#type_prefix_decl}.
	 * @param ctx the parse tree
	 */
	void exitType_prefix_decl(IDLParser.Type_prefix_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#readonly_attr_spec}.
	 * @param ctx the parse tree
	 */
	void enterReadonly_attr_spec(IDLParser.Readonly_attr_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#readonly_attr_spec}.
	 * @param ctx the parse tree
	 */
	void exitReadonly_attr_spec(IDLParser.Readonly_attr_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#readonly_attr_declarator}.
	 * @param ctx the parse tree
	 */
	void enterReadonly_attr_declarator(IDLParser.Readonly_attr_declaratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#readonly_attr_declarator}.
	 * @param ctx the parse tree
	 */
	void exitReadonly_attr_declarator(IDLParser.Readonly_attr_declaratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#attr_spec}.
	 * @param ctx the parse tree
	 */
	void enterAttr_spec(IDLParser.Attr_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#attr_spec}.
	 * @param ctx the parse tree
	 */
	void exitAttr_spec(IDLParser.Attr_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#attr_declarator}.
	 * @param ctx the parse tree
	 */
	void enterAttr_declarator(IDLParser.Attr_declaratorContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#attr_declarator}.
	 * @param ctx the parse tree
	 */
	void exitAttr_declarator(IDLParser.Attr_declaratorContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#attr_raises_expr}.
	 * @param ctx the parse tree
	 */
	void enterAttr_raises_expr(IDLParser.Attr_raises_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#attr_raises_expr}.
	 * @param ctx the parse tree
	 */
	void exitAttr_raises_expr(IDLParser.Attr_raises_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#get_excep_expr}.
	 * @param ctx the parse tree
	 */
	void enterGet_excep_expr(IDLParser.Get_excep_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#get_excep_expr}.
	 * @param ctx the parse tree
	 */
	void exitGet_excep_expr(IDLParser.Get_excep_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#set_excep_expr}.
	 * @param ctx the parse tree
	 */
	void enterSet_excep_expr(IDLParser.Set_excep_exprContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#set_excep_expr}.
	 * @param ctx the parse tree
	 */
	void exitSet_excep_expr(IDLParser.Set_excep_exprContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#exception_list}.
	 * @param ctx the parse tree
	 */
	void enterException_list(IDLParser.Exception_listContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#exception_list}.
	 * @param ctx the parse tree
	 */
	void exitException_list(IDLParser.Exception_listContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#component}.
	 * @param ctx the parse tree
	 */
	void enterComponent(IDLParser.ComponentContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#component}.
	 * @param ctx the parse tree
	 */
	void exitComponent(IDLParser.ComponentContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#component_forward_decl}.
	 * @param ctx the parse tree
	 */
	void enterComponent_forward_decl(IDLParser.Component_forward_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#component_forward_decl}.
	 * @param ctx the parse tree
	 */
	void exitComponent_forward_decl(IDLParser.Component_forward_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#component_decl}.
	 * @param ctx the parse tree
	 */
	void enterComponent_decl(IDLParser.Component_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#component_decl}.
	 * @param ctx the parse tree
	 */
	void exitComponent_decl(IDLParser.Component_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#component_header}.
	 * @param ctx the parse tree
	 */
	void enterComponent_header(IDLParser.Component_headerContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#component_header}.
	 * @param ctx the parse tree
	 */
	void exitComponent_header(IDLParser.Component_headerContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#supported_interface_spec}.
	 * @param ctx the parse tree
	 */
	void enterSupported_interface_spec(IDLParser.Supported_interface_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#supported_interface_spec}.
	 * @param ctx the parse tree
	 */
	void exitSupported_interface_spec(IDLParser.Supported_interface_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#component_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void enterComponent_inheritance_spec(IDLParser.Component_inheritance_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#component_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void exitComponent_inheritance_spec(IDLParser.Component_inheritance_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#component_body}.
	 * @param ctx the parse tree
	 */
	void enterComponent_body(IDLParser.Component_bodyContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#component_body}.
	 * @param ctx the parse tree
	 */
	void exitComponent_body(IDLParser.Component_bodyContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#component_export}.
	 * @param ctx the parse tree
	 */
	void enterComponent_export(IDLParser.Component_exportContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#component_export}.
	 * @param ctx the parse tree
	 */
	void exitComponent_export(IDLParser.Component_exportContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#provides_decl}.
	 * @param ctx the parse tree
	 */
	void enterProvides_decl(IDLParser.Provides_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#provides_decl}.
	 * @param ctx the parse tree
	 */
	void exitProvides_decl(IDLParser.Provides_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#interface_type}.
	 * @param ctx the parse tree
	 */
	void enterInterface_type(IDLParser.Interface_typeContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#interface_type}.
	 * @param ctx the parse tree
	 */
	void exitInterface_type(IDLParser.Interface_typeContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#uses_decl}.
	 * @param ctx the parse tree
	 */
	void enterUses_decl(IDLParser.Uses_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#uses_decl}.
	 * @param ctx the parse tree
	 */
	void exitUses_decl(IDLParser.Uses_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#emits_decl}.
	 * @param ctx the parse tree
	 */
	void enterEmits_decl(IDLParser.Emits_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#emits_decl}.
	 * @param ctx the parse tree
	 */
	void exitEmits_decl(IDLParser.Emits_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#publishes_decl}.
	 * @param ctx the parse tree
	 */
	void enterPublishes_decl(IDLParser.Publishes_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#publishes_decl}.
	 * @param ctx the parse tree
	 */
	void exitPublishes_decl(IDLParser.Publishes_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#consumes_decl}.
	 * @param ctx the parse tree
	 */
	void enterConsumes_decl(IDLParser.Consumes_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#consumes_decl}.
	 * @param ctx the parse tree
	 */
	void exitConsumes_decl(IDLParser.Consumes_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#home_decl}.
	 * @param ctx the parse tree
	 */
	void enterHome_decl(IDLParser.Home_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#home_decl}.
	 * @param ctx the parse tree
	 */
	void exitHome_decl(IDLParser.Home_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#home_header}.
	 * @param ctx the parse tree
	 */
	void enterHome_header(IDLParser.Home_headerContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#home_header}.
	 * @param ctx the parse tree
	 */
	void exitHome_header(IDLParser.Home_headerContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#home_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void enterHome_inheritance_spec(IDLParser.Home_inheritance_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#home_inheritance_spec}.
	 * @param ctx the parse tree
	 */
	void exitHome_inheritance_spec(IDLParser.Home_inheritance_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#primary_key_spec}.
	 * @param ctx the parse tree
	 */
	void enterPrimary_key_spec(IDLParser.Primary_key_specContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#primary_key_spec}.
	 * @param ctx the parse tree
	 */
	void exitPrimary_key_spec(IDLParser.Primary_key_specContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#home_body}.
	 * @param ctx the parse tree
	 */
	void enterHome_body(IDLParser.Home_bodyContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#home_body}.
	 * @param ctx the parse tree
	 */
	void exitHome_body(IDLParser.Home_bodyContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#home_export}.
	 * @param ctx the parse tree
	 */
	void enterHome_export(IDLParser.Home_exportContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#home_export}.
	 * @param ctx the parse tree
	 */
	void exitHome_export(IDLParser.Home_exportContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#factory_decl}.
	 * @param ctx the parse tree
	 */
	void enterFactory_decl(IDLParser.Factory_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#factory_decl}.
	 * @param ctx the parse tree
	 */
	void exitFactory_decl(IDLParser.Factory_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#finder_decl}.
	 * @param ctx the parse tree
	 */
	void enterFinder_decl(IDLParser.Finder_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#finder_decl}.
	 * @param ctx the parse tree
	 */
	void exitFinder_decl(IDLParser.Finder_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#event}.
	 * @param ctx the parse tree
	 */
	void enterEvent(IDLParser.EventContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#event}.
	 * @param ctx the parse tree
	 */
	void exitEvent(IDLParser.EventContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#event_forward_decl}.
	 * @param ctx the parse tree
	 */
	void enterEvent_forward_decl(IDLParser.Event_forward_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#event_forward_decl}.
	 * @param ctx the parse tree
	 */
	void exitEvent_forward_decl(IDLParser.Event_forward_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#event_abs_decl}.
	 * @param ctx the parse tree
	 */
	void enterEvent_abs_decl(IDLParser.Event_abs_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#event_abs_decl}.
	 * @param ctx the parse tree
	 */
	void exitEvent_abs_decl(IDLParser.Event_abs_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#event_decl}.
	 * @param ctx the parse tree
	 */
	void enterEvent_decl(IDLParser.Event_declContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#event_decl}.
	 * @param ctx the parse tree
	 */
	void exitEvent_decl(IDLParser.Event_declContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#event_header}.
	 * @param ctx the parse tree
	 */
	void enterEvent_header(IDLParser.Event_headerContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#event_header}.
	 * @param ctx the parse tree
	 */
	void exitEvent_header(IDLParser.Event_headerContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annapps}.
	 * @param ctx the parse tree
	 */
	void enterAnnapps(IDLParser.AnnappsContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annapps}.
	 * @param ctx the parse tree
	 */
	void exitAnnapps(IDLParser.AnnappsContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_appl}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_appl(IDLParser.Annotation_applContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_appl}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_appl(IDLParser.Annotation_applContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_appl_params}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_appl_params(IDLParser.Annotation_appl_paramsContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_appl_params}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_appl_params(IDLParser.Annotation_appl_paramsContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#annotation_appl_param}.
	 * @param ctx the parse tree
	 */
	void enterAnnotation_appl_param(IDLParser.Annotation_appl_paramContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#annotation_appl_param}.
	 * @param ctx the parse tree
	 */
	void exitAnnotation_appl_param(IDLParser.Annotation_appl_paramContext ctx);
	/**
	 * Enter a parse tree produced by {@link IDLParser#identifier}.
	 * @param ctx the parse tree
	 */
	void enterIdentifier(IDLParser.IdentifierContext ctx);
	/**
	 * Exit a parse tree produced by {@link IDLParser#identifier}.
	 * @param ctx the parse tree
	 */
	void exitIdentifier(IDLParser.IdentifierContext ctx);
}