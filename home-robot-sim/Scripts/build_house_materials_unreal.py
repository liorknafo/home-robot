"""Import original house surfaces and build textured vertex-colour PBR materials."""
import unreal
from pathlib import Path
R=Path(__file__).resolve().parents[1];tools=unreal.AssetToolsHelpers.get_asset_tools();dest='/Game/HouseMaterials';edit=unreal.MaterialEditingLibrary
for image in sorted((R/'assets/house').glob('*.png')):
    name=image.stem
    task=unreal.AssetImportTask();task.set_editor_property('filename',str(image));task.set_editor_property('destination_path',dest);task.set_editor_property('destination_name','T_'+name);task.set_editor_property('automated',True);task.set_editor_property('replace_existing',True);task.set_editor_property('save',True);tools.import_asset_tasks([task]);texture=unreal.load_asset(dest+'/T_'+name)
    material=unreal.load_asset(dest+'/M_'+name)
    if material is None:material=tools.create_asset('M_'+name,dest,unreal.Material,unreal.MaterialFactoryNew())
    edit.delete_all_material_expressions(material);material.set_editor_property('two_sided',True)
    tex=edit.create_material_expression(material,unreal.MaterialExpressionTextureSampleParameter2D,-600,0);tex.set_editor_property('parameter_name','SurfaceColor');tex.set_editor_property('texture',texture)
    vertex=edit.create_material_expression(material,unreal.MaterialExpressionVertexColor,-600,170)
    mul=edit.create_material_expression(material,unreal.MaterialExpressionMultiply,-300,0)
    assert edit.connect_material_expressions(tex,'RGB',mul,'A')
    assert edit.connect_material_expressions(vertex,'',mul,'B')
    assert edit.connect_material_property(mul,'',unreal.MaterialProperty.MP_BASE_COLOR)
    rough={'wood':.54,'fabric':.87,'ceramic':.22,'metal':.27,'mirror':.03,'plastic':.38,'rubber':.82,'leaf':.58,'fruit':.45,'art':.85}.get(name,.7)
    for key,val,prop,y in [('Roughness',rough,unreal.MaterialProperty.MP_ROUGHNESS,300),('Specular',.45,unreal.MaterialProperty.MP_SPECULAR,400),('Metallic',1. if name in ['metal','mirror'] else 0.,unreal.MaterialProperty.MP_METALLIC,500)]:
        node=edit.create_material_expression(material,unreal.MaterialExpressionScalarParameter,-300,y);node.set_editor_property('parameter_name',key);node.set_editor_property('default_value',val);edit.connect_material_property(node,'',prop)
    edit.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material)
    print('HOUSE_MATERIAL_READY',name)
