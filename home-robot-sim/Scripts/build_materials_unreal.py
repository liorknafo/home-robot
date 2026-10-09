"""Run through Unreal's Python commandlet to build native material assets."""
import unreal
from pathlib import Path
R=Path(__file__).resolve().parents[1];tools=unreal.AssetToolsHelpers.get_asset_tools();dest='/Game/RobotMaterials'
for name,file,rough in [('Terrazzo','terrazzo-floor.png',.30),('WhitePaint','white-paint.png',.87)]:
    task=unreal.AssetImportTask();task.set_editor_property('filename',str(R/'assets'/file));task.set_editor_property('destination_path',dest);task.set_editor_property('destination_name','T_'+name);task.set_editor_property('automated',True);task.set_editor_property('replace_existing',True);task.set_editor_property('save',True);tools.import_asset_tasks([task]);texture=unreal.load_asset(dest+'/T_'+name)
    material=unreal.load_asset(dest+'/M_'+name)
    if material is None:material=tools.create_asset('M_'+name,dest,unreal.Material,unreal.MaterialFactoryNew())
    unreal.MaterialEditingLibrary.delete_all_material_expressions(material)
    tex=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionTextureSampleParameter2D,-400,0);tex.set_editor_property('parameter_name','SurfaceColor');tex.set_editor_property('texture',texture)
    unreal.MaterialEditingLibrary.connect_material_property(tex,'RGB',unreal.MaterialProperty.MP_BASE_COLOR)
    for key,val,prop,y in [('Roughness',rough,unreal.MaterialProperty.MP_ROUGHNESS,200),('Specular',.4,unreal.MaterialProperty.MP_SPECULAR,300),('Metallic',0.,unreal.MaterialProperty.MP_METALLIC,400)]:
        node=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionScalarParameter,-400,y);node.set_editor_property('parameter_name',key);node.set_editor_property('default_value',val);unreal.MaterialEditingLibrary.connect_material_property(node,'',prop)
    unreal.MaterialEditingLibrary.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material)
    print('SURFACE_ASSET_READY',dest+'/M_'+name)
material=unreal.load_asset(dest+'/M_Prop')
if material is None:material=tools.create_asset('M_Prop',dest,unreal.Material,unreal.MaterialFactoryNew())
unreal.MaterialEditingLibrary.delete_all_material_expressions(material)
color=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionVertexColor,-300,0)
assert unreal.MaterialEditingLibrary.connect_material_property(color,'',unreal.MaterialProperty.MP_BASE_COLOR), 'Vertex color connection failed'
for key,val,prop,y in [('Roughness',.72,unreal.MaterialProperty.MP_ROUGHNESS,200),('Specular',.35,unreal.MaterialProperty.MP_SPECULAR,300)]:
    node=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionScalarParameter,-300,y);node.set_editor_property('parameter_name',key);node.set_editor_property('default_value',val);unreal.MaterialEditingLibrary.connect_material_property(node,'',prop)
unreal.MaterialEditingLibrary.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material)
