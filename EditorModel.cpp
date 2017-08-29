#include "EditorModel.h"
#include "thirdpart/tinyobj/tiny_obj_loader.h"
#include <iostream>
#include "Assertion.h"
#include "D3D11FastRenderer.h"
#include "thirdpart/imgui/imgui.h"
#include "Input.h"
#include "Color32.h"

#include "../pbr_renderer.h"
#include "../camera.h"
#include "../film.h"
#include "../material.h"

#include<windows.h>  
//打开保存文件对话框  
#include<Commdlg.h>  

std::vector<Mesh*>& EditorScene::read_model(const char* filename, int kd_threshold, Direct3D11Renderer* _d3d11_renderer, ShaderID bound_shader, bool release_data)
{
  std::vector<Mesh*>& mesh_sets = loaded_meshes[std::string(filename)];
  if (mesh_sets.empty())
    /** load file */
  {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    std::string path;
    path = RBPathTool::get_file_directory(filename);
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename, path.data());

    if (!err.empty())
    {
      std::cout << err << std::endl;
    }

    if (!ret)
    {
      g_logger->debug_print("读取文件%s失败\n", filename);
      getchar();
    }

    for (int i = 0; i < shapes.size(); ++i)
    {

      Mesh* mesh = new Mesh();
      mesh->name = shapes[i].name;

      std::vector<Vertex_PNT> d3d11_vert;
      std::vector<u32> d3d11_idx;

      mesh->tex.resize(attrib.texcoords.size() / 2);
      mesh->vertices.resize(attrib.vertices.size() / 3);
      mesh->normals.resize(attrib.normals.size() / 3);
      memcpy(&mesh->tex[0], &attrib.texcoords[0], attrib.texcoords.size()*sizeof(f32));
      memcpy(&mesh->normals[0], &attrib.normals[0], attrib.normals.size()*sizeof(f32));
      memcpy(&mesh->vertices[0], &attrib.vertices[0], attrib.vertices.size()*sizeof(f32));

      //Rerange纹理坐标0~1
      
      for (int i = 0; i < mesh->tex.size(); ++i)
      {
         RBVector2& uv = mesh->tex[i];
         uv.x = uv.x - (int)uv.x;
         uv.x = uv.x < 0 ? 1 + uv.x : uv.x;
         uv.y = uv.y - (int)uv.y;
         uv.y = uv.y < 0 ? 1 + uv.y : uv.y;
      }
      

      mesh->vertex_idx.resize(shapes[i].mesh.indices.size());
      mesh->tex_idx.resize(shapes[i].mesh.indices.size());
      mesh->nomral_idx.resize(shapes[i].mesh.indices.size());
      d3d11_idx.resize(shapes[i].mesh.indices.size());
      d3d11_vert.resize(shapes[i].mesh.indices.size());
      for (int k = 0; k < shapes[i].mesh.indices.size(); ++k)
      {
        //CHECK(3 == shapes[i].mesh.num_face_vertices[k]);
        mesh->vertex_idx[k] = shapes[i].mesh.indices[k].vertex_index;
        mesh->tex_idx[k] = shapes[i].mesh.indices[k].texcoord_index;
        mesh->nomral_idx[k] = shapes[i].mesh.indices[k].normal_index;

        //todo:移除重复顶点重定位索引
        Vertex_PNT vv;
        vv.pos = mesh->vertices[shapes[i].mesh.indices[k].vertex_index];
        vv.normal = mesh->normals[shapes[i].mesh.indices[k].normal_index];
		if (shapes[i].mesh.indices[k].texcoord_index == -1)
			vv.texcoord = RBVector2(0,0);
		else
			vv.texcoord = mesh->tex[shapes[i].mesh.indices[k].texcoord_index];
        d3d11_vert[k] = vv;
        d3d11_idx[k] = k;
      }

      mesh->bound.reset();

      mesh->triangles.resize(shapes[i].mesh.num_face_vertices.size());
      for (int j = 0; j < mesh->triangles.size(); j++)
      {
        int base_index = j * 3;
        int base_index_uv = j * 3;
        mesh->triangles[j] = new OLPBRTriangle_
          (
          //pos
          mesh->vertices[mesh->vertex_idx[base_index]], mesh->vertices[mesh->vertex_idx[base_index + 1]], mesh->vertices[mesh->vertex_idx[base_index + 2]],
          //normal
          mesh->normals[mesh->nomral_idx[base_index]], mesh->normals[mesh->nomral_idx[base_index + 1]], mesh->normals[mesh->nomral_idx[base_index + 2]],
          //tex
		  mesh->tex_idx[base_index_uv]==-1?RBVector2(0,0):mesh->tex[mesh->tex_idx[base_index_uv]],
		  mesh->tex_idx[base_index_uv+1] == -1 ? RBVector2(0, 0) : mesh->tex[mesh->tex_idx[base_index_uv + 1]],
		  mesh->tex_idx[base_index_uv+2] == -1 ? RBVector2(0, 0) : mesh->tex[mesh->tex_idx[base_index_uv + 2]]
          );
        mesh->bound.include(mesh->triangles[j]->bound());

      }
      mesh->bound.fix_thin();
      if (shapes[i].mesh.num_face_vertices.size()>kd_threshold)
      {
        //build kdtree
        mesh->kdtree = new OLPBRKDTree_();
        //shapes[i].mesh.num_face_vertices.size()*10k
        mesh->kdtree->init_mem((1 << 10) * 10 * shapes[i].mesh.num_face_vertices.size());
        std::vector<int> index;
        index.resize(mesh->triangles.size());
        for (int i = 0; i < index.size(); ++i)
        {
          index[i] = i;
        }
        int dp = 1;
        if (0 != mesh->triangles.size())
        {
          //kdtree_root->leaf = false;
          dp = (int)((8) + 1.3f*RBMath::log_2(mesh->triangles.size()));
        }
        dp = RBMath::get_min(dp, 20);
        mesh->kdtree->build_kdtree(mesh->bound, mesh->triangles, index, dp, mesh->kdtree->root);
      }

      mesh->d3d11_buffer.ib = _d3d11_renderer->addIndexBuffer(d3d11_idx.size(), sizeof(u32), BufferAccess::STATIC, d3d11_idx.data());
      mesh->d3d11_buffer.ib_size = d3d11_idx.size();
      mesh->d3d11_buffer.vb = _d3d11_renderer->addVertexBuffer(d3d11_vert.size()*sizeof(Vertex_PNT), BufferAccess::STATIC, d3d11_vert.data());

      //todo:to many vfs!
      FormatDesc fd[3];
      fd[0].format = AttributeFormat::FORMAT_FLOAT;
      fd[0].size = 3;
      fd[0].stream = 0;
      fd[0].type = AttributeType::TYPE_VERTEX;
      fd[1].format = AttributeFormat::FORMAT_FLOAT;
      fd[1].size = 3;
      fd[1].stream = 0;
      fd[1].type = AttributeType::TYPE_NORMAL;
      fd[2].format = AttributeFormat::FORMAT_FLOAT;
      fd[2].size = 2;
      fd[2].stream = 0;
      fd[2].type = AttributeType::TYPE_TEXCOORD;

      mesh->d3d11_buffer.vf = _d3d11_renderer->addVertexFormat(fd, 3, bound_shader);

      mesh_sets.push_back(mesh);

      if (release_data)
      {
        mesh->release_data();
      }
    }
  }
  imported_file_names.push_back(filename);
  return mesh_sets;
}

void EditorScene::save_scene(const char* filename)
{
  if (imported_file_names.size() == 0)
    return;



  //储存当前objec的材质链接
  int obj_size = 1;
  for (auto &it : objects)
  {
    obj_size += it->sub_object.size();
  }

  int *mem_mat_link = new int[obj_size];
  int* mem_mat_link_head = mem_mat_link;
  mem_mat_link[0] = obj_size - 1;
  int k = 1;
  for (auto &it : objects)
  {
    for (int i = 0; i < it->sub_object.size(); ++i)
    {
      ModelObjectInstance* its = (ModelObjectInstance*)it->sub_object[i];
      mem_mat_link[k++] = its->material_id;
    }
  }

  CHECK(k == obj_size);

  //储存材质库
  int mat_size = sizeof(int);
  for (auto& mat : material_lib)
  {
    if (mat)
      mat_size += mat->material_data->get_serialize_size() + sizeof(int);
    else
      mat_size += sizeof(int);
  }
  char* mem_mat = new char[mat_size];
  char* mem_mat_head = mem_mat;
  *((int*)mem_mat) = (int)material_lib.size();
  mem_mat += sizeof(int);
  char* p = mem_mat;
  for (auto& mat : material_lib)
  {
    if (mat)
    {
      size_t size = mat->material_data->get_serialize_size();
      *((int*)p) = size;
      p += sizeof(int);
      mat->material_data->serialize(p);
      //p += size;
    }
    else
    {
      *((int*)p) = 0;
      p += sizeof(int);
    }
  }

  CHECK((p - mem_mat + sizeof(int)) == mat_size);

  char iname[256];
  memset(iname, 0, 256);
  memcpy(iname,imported_file_names[0].data(),256);

  char* mem = new char[obj_size*sizeof(int) + mat_size + 256];
  char* mem_head = mem;
  memcpy(mem,iname,256);
  mem += 256;
  memcpy(mem, mem_mat_link_head, obj_size*sizeof(int));
  mem += obj_size*sizeof(int);
  memcpy(mem, mem_mat_head, mat_size);

  delete[] mem_mat_link_head;
  delete[] mem_mat_head;


  std::string new_name = filename;
  std::ofstream fout(new_name.c_str(), std::ios::binary);
  fout.write(mem_head, obj_size*sizeof(int) + mat_size + 256);
  fout.close();


  delete[] mem_head;

}

bool EditorScene::load_scene(const char* filename, Direct3D11Renderer* d3d11_render, PreViewRenderer* pre_r, ShaderID bound_shader)
{
  std::ifstream fin(filename, std::ios::binary);
  if (!fin)
  {
    printf("read %s failed!\n", "mat_cache.sf");
    return false;
  }
  fin.seekg(0, fin.end);
  int read_size = fin.tellg();
  fin.seekg(0, fin.beg);
  char* mem = new char[read_size];
  fin.read(mem, read_size);
  fin.close();

  char* pp = mem;
  char iname[256];
  memcpy(iname,pp,256);
  pp += 256;

  
  std::vector<Mesh*>& itk = read_model(iname, 32, d3d11_render, bound_shader);
  ObjectInstanceSet*  a = new ObjectInstanceSet();

  a->local_2_world.traslate(0, 0,0);
  a->world_2_local = a->local_2_world.get_inverse_slow();
  for (int i = 0; i < itk.size(); ++i)
  {
    ModelObjectInstance* meshi = new ModelObjectInstance();
    meshi->hide = false;
    meshi->object_ref = itk[i];
    meshi->localtox.set_identity();
    meshi->xtolocal = meshi->localtox.get_inverse_slow();

    a->sub_object.push_back(meshi);
  }

  objects.push_back(a);



  int* p = (int*)pp;

  int objsize = *p;
  p++;
  int idx = 0;
  for (auto &it : objects)
  {
    for (int k = 0; k < it->sub_object.size(); ++k)
    {
      ModelObjectInstance* its = (ModelObjectInstance*)it->sub_object[k];
      its->material_id = *p;
      p++;
      idx++;
    }
  }

  CHECK((p - (int*)pp) == idx + 1);

  const char* pm = (char*)p;
  int size_mat = *((int*)pm);
  pm += sizeof(int);
  for (int s = 0; s < size_mat; ++s)
  {
    int sizet = *((int*)pm);
    pm += sizeof(int);
    LibMaterial* libmat = new LibMaterial(d3d11_render, pre_r);
    MaterialMetaData* mat = nullptr;
    if (sizet != 0)
    {
      int type = *((int*)pm);

      switch (type)
      {
      case MaterialMetaData::E_MM_METAL:
        mat = new MetalMaterialMetaData();
        break;
      case MaterialMetaData::E_MM_GLASS:
        mat = new GlassMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_MATTE:
        mat = new MatteMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_SIMPLEGLASS:
        mat = new SimpleGlassMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_FRESNEL_BLEND:
        mat = new FresnelBlendMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_PLASTIC:
        mat = new PlasticMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_MIRROR:
        mat = new MirrorMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_HAIR:
        mat = new HairMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_SUBSTRATE:
        mat = new SubstrateMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_TRANSLUCENT:
        mat = new TranslucentMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_UBER:
        mat = new UberMaterialMetaData();

        break;
      case MaterialMetaData::E_MM_DISNEY:
        mat = new DisneyMaterialMetaData(false);
        break;
      case MaterialMetaData::E_MM_BLANK:
        mat = new BlankMaterialMetaData();
        break;
      case MaterialMetaData::E_MM_ALPHA_DISNEY:
        mat = new DisneyMaterialMetaData(true);
        break;
      case MaterialMetaData::E_MM_MIX:
        printf("Don not support now!\n");
        break;
      default:
        printf("funcking material!\n");
        break;
      }
      if (mat)
        mat->deserialize(pm, sizet);

      libmat->material_data = mat;

      material_lib[s] = libmat;

      //pm += sizet;
    }
    else
    {
      material_lib[s] = nullptr;
    }
  }

  CHECK(pm - mem == read_size);
  delete[] mem;

  return true;
}

void EditorScene::process_obj(const char* filename)
{
  /*
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    std::string path;
    path = RBPathTool::get_file_directory(filename);
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename, path.data());

    if (!err.empty())
    {
    std::cout << err << std::endl;
    }

    if (!ret)
    {
    g_logger->debug_print("读取文件%s失败\n", filename);
    getchar();
    }

    for (int i = 0; i < shapes.size(); ++i)
    {

    Mesh* mesh = new Mesh();
    mesh->name = shapes[i].name;

    std::vector<Vertex_PNT> d3d11_vert;
    std::vector<u32> d3d11_idx;

    mesh->tex.resize(attrib.texcoords.size() / 2);
    mesh->vertices.resize(attrib.vertices.size() / 3);
    mesh->normals.resize(attrib.normals.size() / 3);
    memcpy(&mesh->tex[0], &attrib.texcoords[0], attrib.texcoords.size()*sizeof(f32));
    memcpy(&mesh->normals[0], &attrib.normals[0], attrib.normals.size()*sizeof(f32));
    memcpy(&mesh->vertices[0], &attrib.vertices[0], attrib.vertices.size()*sizeof(f32));

    mesh->vertex_idx.resize(shapes[i].mesh.indices.size());
    mesh->tex_idx.resize(shapes[i].mesh.indices.size());
    mesh->nomral_idx.resize(shapes[i].mesh.indices.size());
    d3d11_idx.resize(shapes[i].mesh.indices.size());
    d3d11_vert.resize(shapes[i].mesh.indices.size());
    for (int k = 0; k < shapes[i].mesh.indices.size(); ++k)
    {
    //CHECK(3 == shapes[i].mesh.num_face_vertices[k]);
    mesh->vertex_idx[k] = shapes[i].mesh.indices[k].vertex_index;
    mesh->tex_idx[k] = shapes[i].mesh.indices[k].texcoord_index;
    mesh->nomral_idx[k] = shapes[i].mesh.indices[k].normal_index;

    //todo:移除重复顶点重定位索引
    Vertex_PNT vv;
    vv.pos = mesh->vertices[shapes[i].mesh.indices[k].vertex_index];
    vv.normal = mesh->normals[shapes[i].mesh.indices[k].normal_index];
    vv.texcoord = mesh->tex[shapes[i].mesh.indices[k].texcoord_index];
    d3d11_vert[k] = vv;
    d3d11_idx[k] = k;
    }

    mesh->bound.reset();

    mesh->triangles.resize(shapes[i].mesh.num_face_vertices.size());
    for (int j = 0; j < mesh->triangles.size(); j++)
    {
    int base_index = j * 3;
    int base_index_uv = j * 2;
    mesh->triangles[j] = new OLPBRTriangle_
    (
    //pos
    mesh->vertices[mesh->vertex_idx[base_index]], mesh->vertices[mesh->vertex_idx[base_index + 1]], mesh->vertices[mesh->vertex_idx[base_index + 2]],
    //normal
    mesh->normals[mesh->nomral_idx[base_index]], mesh->normals[mesh->nomral_idx[base_index + 1]], mesh->normals[mesh->nomral_idx[base_index + 2]],
    //tex
    mesh->tex[mesh->tex_idx[base_index_uv]], mesh->tex[mesh->tex_idx[base_index_uv + 1]], mesh->tex[mesh->tex_idx[base_index_uv + 2]]
    );
    mesh->bound.include(mesh->triangles[j]->bound());

    }
    mesh->bound.fix_thin();
    if (shapes[i].mesh.num_face_vertices.size()>kd_threshold)
    {
    //build kdtree
    mesh->kdtree = new OLPBRKDTree_();
    //shapes[i].mesh.num_face_vertices.size()*10k
    mesh->kdtree->init_mem((1 << 10) * 10 * shapes[i].mesh.num_face_vertices.size());
    std::vector<int> index;
    index.resize(mesh->triangles.size());
    for (int i = 0; i < index.size(); ++i)
    {
    index[i] = i;
    }
    int dp = 1;
    if (0 != mesh->triangles.size())
    {
    //kdtree_root->leaf = false;
    dp = (int)((8) + 1.3f*RBMath::log_2(mesh->triangles.size()));
    }
    dp = RBMath::get_min(dp, 20);
    mesh->kdtree->build_kdtree(mesh->bound, mesh->triangles, index, dp, mesh->kdtree->root);
    }

    mesh->d3d11_buffer.ib = _d3d11_renderer->addIndexBuffer(d3d11_idx.size(), sizeof(u32), BufferAccess::STATIC, d3d11_idx.data());
    mesh->d3d11_buffer.ib_size = d3d11_idx.size();
    mesh->d3d11_buffer.vb = _d3d11_renderer->addVertexBuffer(d3d11_vert.size()*sizeof(Vertex_PNT), BufferAccess::STATIC, d3d11_vert.data());

    //todo:to many vfs!
    FormatDesc fd[3];
    fd[0].format = AttributeFormat::FORMAT_FLOAT;
    fd[0].size = 3;
    fd[0].stream = 0;
    fd[0].type = AttributeType::TYPE_VERTEX;
    fd[1].format = AttributeFormat::FORMAT_FLOAT;
    fd[1].size = 3;
    fd[1].stream = 0;
    fd[1].type = AttributeType::TYPE_NORMAL;
    fd[2].format = AttributeFormat::FORMAT_FLOAT;
    fd[2].size = 2;
    fd[2].stream = 0;
    fd[2].type = AttributeType::TYPE_TEXCOORD;

    mesh->d3d11_buffer.vf = _d3d11_renderer->addVertexFormat(fd, 3, bound_shader);


    mesh->release_data();

    }


    std::string new_name = ".sf";
    new_name = filename + new_name;
    std::ofstream fout(new_name, std::ios::binary);
    */


}

void EditorScene::clear_scene(Direct3D11Renderer* d3d11_renderer)
{

	//释放对象
  for (int i = 0; i < objects.size(); ++i)
  {
    for (int j = 0; j < objects[i]->sub_object.size(); ++j)
    {
      delete objects[i]->sub_object[j];
    }
    delete objects[i];
  }
  objects.clear();
  //释放kdtree
  if (scene_kdtree)
    scene_kdtree->free_mem();
  //释放加载的mesh
  for (std::map<std::string, std::vector<Mesh*>>::iterator it = loaded_meshes.begin(); it != loaded_meshes.end();++it)
  {
    for (int i = 0; i < it->second.size(); ++i)
    {
		it->second[i]->release_d3d11_buffer(d3d11_renderer);
	  delete it->second[i];
    }
  }
  loaded_meshes.clear();

  //释放材质
  for (int i = 0; i < material_lib.size();++i)
  {
	  if (!material_lib[i])
		  continue;
	  material_lib[i]->release(d3d11_renderer);
	  delete material_lib[i];
  }
  memset(material_lib.data(), 0, material_lib.size()*sizeof(void*));
  imported_file_names.clear();
  hold_scene = "";
}

Mesh::~Mesh()
{
	release_data();
	for (int j = 0; j<triangles.size(); j++)
	{
		delete triangles[j];
	}
	if (kdtree)
		kdtree->free_mem();
}

void Mesh::release_d3d11_buffer(Direct3D11Renderer* d3d11_renderer)
{
	d3d11_renderer->removeVertexFormat(d3d11_buffer.vf);
	d3d11_renderer->removeIndexBuffer(d3d11_buffer.ib);
	d3d11_renderer->removeVertexBuffer(d3d11_buffer.vb);
}

void Mesh::release_data()
{
  vertex_idx.clear();
  vertex_idx.swap(vector<int>());
  tex_idx.clear();
  tex_idx.swap(vector<int>());
  nomral_idx.clear();
  nomral_idx.swap(vector<int>());
  tex.clear();
  tex.swap(vector<RBVector2>());
  normals.clear();
  normals.swap(vector<RBVector3>());
  vertices.clear();
  vertices.swap(vector<RBVector3>());

}

#include "../pbr_renderer.h"

void EditorSceneView::init(Direct3D11Renderer* d3d11_renderer, ID3D11RenderTargetView* b)
{
  scene_camera = new RBCamera();
  scene_camera->set_position(0, 0, 0);
  RBVector4 position(0, 0, -1);
  scene_camera->set_target(position);
  scene_camera->set_fov_y(60);
  scene_camera->set_ratio(16.f / 9.f);
  scene_camera->set_near_panel(0.01f);
  scene_camera->set_far_panel(2000.f);
  _cam_move_speed = RBVector2(150, 150);
  _cam_rotate_speed = RBVector2(5, 5);

  scene_model = new EditorScene();
  scene_model->init(150);

  _d3d11_renderer = d3d11_renderer;
  _back_buffer_view = b;

  selected_mesh_instance = nullptr;

  pre_renderer = new PreViewRenderer();
  pre_renderer->init();

  prod_renderer = new ProductionRenderer();
  prod_renderer->init();

  selected_material_id = -1;
  hit_sphere = nullptr;
  is_use_ibl = true;

  ass_scale = RBVector2(0.1f,0.1f);

  show_help = false;
  show_material_editor = false;
  show_render = false;
  show_camera_setting = false;
  show_debug_tool = false;

  //初始化dock
  node_editor_dock.initialize("Node Editor", true, ImVec2(), [&](ImVec2 area) {
    if (cur_node_editor)
      cur_node_editor->render();

  });

  post_dock.initialize("Post Effect", true, ImVec2(250, 300), [&](ImVec2 area) {
    on_show_result_gui();
  });


  create_material_editor.initialize("Create Material", true, ImVec2(250, 300), [&](ImVec2 area) {
    on_create_material_gui();
  });
  debug_dock.initialize("Debug", true, ImVec2(250, 300), [&](ImVec2 area) {
    on_check_brdf_gui();
  });


  material_editor.initialize("Material", true, ImVec2(), [&](ImVec2 area){
    on_material_edit_gui();
  });
  object_editor.initialize("Properties", true, ImVec2(), [&](ImVec2 area){
    ImGuizmo::BeginFrame();
    if (selected_mesh_instance)
    {
      selected_mesh_instance->on_gui(this);
    }
  });

  render_dock.initialize("Render", true, ImVec2(250, 300), [&](ImVec2 area) {
    on_render_gui();
  });
  camera_dock.initialize("Camera", true, ImVec2(250, 300), [&](ImVec2 area) {
    on_set_camera_gui();
  });

  dockspace.dock(&post_dock, ImGuiDock::DockSlot::Right, 250, true);
  dockspace.dockWith(&node_editor_dock, &post_dock, ImGuiDock::DockSlot::Tab, 0, false);
  dockspace.dockWith(&debug_dock, &post_dock, ImGuiDock::DockSlot::Tab, 0, false);
  //dockspace.dockWith(&node_editor_dock, &post_dock, ImGuiDock::DockSlot::Tab, 0, true);


  right_window.dock(&material_editor, ImGuiDock::DockSlot::Right,0, true);
  right_window.dockWith(&object_editor,&material_editor, ImGuiDock::DockSlot::Tab, 0, false);

  right_window.dockWith(&render_dock, &material_editor, ImGuiDock::DockSlot::Bottom, 0, false);
  right_window.dockWith(&camera_dock,&render_dock, ImGuiDock::DockSlot::Tab, 0, false);
  right_window.dockWith(&create_material_editor, &camera_dock, ImGuiDock::DockSlot::Tab, 0, true);

}

void EditorSceneView::load_sys_assets()
{
  //load shader
  auto sh1 = D3D11ShaderCompiler::compile(_d3d11_renderer->device, "./shaders/editor/simple_v.hlsl", "main", "vs_5_0");
  auto sh2 = D3D11ShaderCompiler::compile(_d3d11_renderer->device, "./shaders/editor/simple_p.hlsl", "main", "ps_5_0");
  mode_render_shader = _d3d11_renderer->addShader(sh1, 0, sh2, D3D11ShaderCompiler::reflect_shader);

}


void EditorSceneView::create_rhires(int ww, int wh)
{
  _ds_enable = _d3d11_renderer->addDepthState(true, true, D3D11_COMPARISON_LESS, true, 0xFF, 0xFF,
    D3D11_COMPARISON_ALWAYS, D3D11_COMPARISON_ALWAYS,
    D3D11_STENCIL_OP_KEEP, D3D11_STENCIL_OP_KEEP,
    D3D11_STENCIL_OP_INCR, D3D11_STENCIL_OP_DECR,
    D3D11_STENCIL_OP_KEEP, D3D11_STENCIL_OP_KEEP);
  _bs_enable = _d3d11_renderer->addBlendState(D3D11_BLEND_SRC_ALPHA, D3D11_BLEND_INV_SRC_ALPHA,
    D3D11_BLEND_ONE, D3D11_BLEND_ZERO,
    D3D11_BLEND_OP_ADD, D3D11_BLEND_OP_ADD,
    D3D11_COLOR_WRITE_ENABLE_ALL);
  _rs = _d3d11_renderer->addRasterizerState(CULL_BACK, SOLID, false, false, 0, 0);
  _buffer_depth_stencil = _d3d11_renderer->addRenderDepth(ww, wh, 1, FORMAT::FORMAT_D24S8);
  {
    D3D11_TEXTURE2D_DESC desc;
    rb_memzero(&desc, sizeof(desc));
    desc.Width = 200;
    desc.Height = 200;
    desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;
    desc.Usage = D3D11_USAGE_DYNAMIC;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.ArraySize = 1;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    HRESULT res = _d3d11_renderer->device->CreateTexture2D(&desc, 0, &tex);
    if (res != S_OK)
    {
      g_logger->debug(WIP_ERROR, "Couldn't create dynamic texture!");
    }

    _ui_show_tex = _d3d11_renderer->addTexture(tex, 0);
  }

  RBColor32* dao = new RBColor32[200 * 200];
  for (int i = 0; i < 40000; ++i)
  {
    dao[i].r = 125;
    dao[i].g = 36;
    dao[i].b = 200;
    dao[i].a = 255;
  }

  D3D11_MAPPED_SUBRESOURCE resource;
  _d3d11_renderer->context->Map(_d3d11_renderer->textures[_ui_show_tex].texture,
    0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
  u8* da = reinterpret_cast<u8*>(resource.pData);
  u8* dao_c = (u8*)dao;
  for (int a = 0; a < 200; ++a)
  {
    memcpy(da, dao_c, sizeof(RBColor32) * 200);
    da += resource.RowPitch;
    dao_c += 200 * sizeof(RBColor32);
  }
  _d3d11_renderer->context->Unmap(_d3d11_renderer->textures[_ui_show_tex].texture, 0);
  delete[] dao;
}

void EditorSceneView::init_assistant_object()
{
  const char* filename = "./Res/editor/hit_sphere.obj";

    /** load file */
  {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    std::string path;
    path = RBPathTool::get_file_directory(filename);
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename, path.data());

    if (!err.empty())
    {
      std::cout << err << std::endl;
    }

    if (!ret)
    {
      g_logger->debug_print("读取文件%s失败\n", filename);
      getchar();
    }

    for (int i = 0; i < shapes.size(); ++i)
    {

      Mesh* mesh = new Mesh();
      mesh->name = shapes[i].name;

      std::vector<Vertex_PNT> d3d11_vert;
      std::vector<u32> d3d11_idx;

      mesh->tex.resize(attrib.texcoords.size() / 2);
      mesh->vertices.resize(attrib.vertices.size() / 3);
      mesh->normals.resize(attrib.normals.size() / 3);
      memcpy(&mesh->tex[0], &attrib.texcoords[0], attrib.texcoords.size()*sizeof(f32));
      memcpy(&mesh->normals[0], &attrib.normals[0], attrib.normals.size()*sizeof(f32));
      memcpy(&mesh->vertices[0], &attrib.vertices[0], attrib.vertices.size()*sizeof(f32));

      mesh->vertex_idx.resize(shapes[i].mesh.indices.size());
      mesh->tex_idx.resize(shapes[i].mesh.indices.size());
      mesh->nomral_idx.resize(shapes[i].mesh.indices.size());
      d3d11_idx.resize(shapes[i].mesh.indices.size());
      d3d11_vert.resize(shapes[i].mesh.indices.size());
      for (int k = 0; k < shapes[i].mesh.indices.size(); ++k)
      {
        //CHECK(3 == shapes[i].mesh.num_face_vertices[k]);
        mesh->vertex_idx[k] = shapes[i].mesh.indices[k].vertex_index;
        mesh->tex_idx[k] = shapes[i].mesh.indices[k].texcoord_index;
        mesh->nomral_idx[k] = shapes[i].mesh.indices[k].normal_index;

        //todo:移除重复顶点重定位索引
        Vertex_PNT vv;
        vv.pos = mesh->vertices[shapes[i].mesh.indices[k].vertex_index];
        vv.normal = mesh->normals[shapes[i].mesh.indices[k].normal_index];
        if (shapes[i].mesh.indices[k].texcoord_index == -1)
          vv.texcoord = RBVector2(0, 0);
        else
          vv.texcoord = mesh->tex[shapes[i].mesh.indices[k].texcoord_index];
        d3d11_vert[k] = vv;
        d3d11_idx[k] = k;
      }

      mesh->bound.reset();

      mesh->triangles.resize(shapes[i].mesh.num_face_vertices.size());
      for (int j = 0; j < mesh->triangles.size(); j++)
      {
        int base_index = j * 3;
        int base_index_uv = j * 2;
        mesh->triangles[j] = new OLPBRTriangle_
          (
          //pos
          mesh->vertices[mesh->vertex_idx[base_index]], mesh->vertices[mesh->vertex_idx[base_index + 1]], mesh->vertices[mesh->vertex_idx[base_index + 2]],
          //normal
          mesh->normals[mesh->nomral_idx[base_index]], mesh->normals[mesh->nomral_idx[base_index + 1]], mesh->normals[mesh->nomral_idx[base_index + 2]],
          //tex
          mesh->tex_idx[base_index_uv] == -1 ? RBVector2(0, 0) : mesh->tex[mesh->tex_idx[base_index_uv]],
          mesh->tex_idx[base_index_uv + 1] == -1 ? RBVector2(0, 0) : mesh->tex[mesh->tex_idx[base_index_uv + 1]],
          mesh->tex_idx[base_index_uv + 2] == -1 ? RBVector2(0, 0) : mesh->tex[mesh->tex_idx[base_index_uv + 2]]
          );
        mesh->bound.include(mesh->triangles[j]->bound());

      }
      mesh->bound.fix_thin();
      if (shapes[i].mesh.num_face_vertices.size()>1000)
      {
        //build kdtree
        mesh->kdtree = new OLPBRKDTree_();
        //shapes[i].mesh.num_face_vertices.size()*10k
        mesh->kdtree->init_mem((1 << 10) * 10 * shapes[i].mesh.num_face_vertices.size());
        std::vector<int> index;
        index.resize(mesh->triangles.size());
        for (int i = 0; i < index.size(); ++i)
        {
          index[i] = i;
        }
        int dp = 1;
        if (0 != mesh->triangles.size())
        {
          //kdtree_root->leaf = false;
          dp = (int)((8) + 1.3f*RBMath::log_2(mesh->triangles.size()));
        }
        dp = RBMath::get_min(dp, 20);
        mesh->kdtree->build_kdtree(mesh->bound, mesh->triangles, index, dp, mesh->kdtree->root);
      }

      mesh->d3d11_buffer.ib = _d3d11_renderer->addIndexBuffer(d3d11_idx.size(), sizeof(u32), BufferAccess::STATIC, d3d11_idx.data());
      mesh->d3d11_buffer.ib_size = d3d11_idx.size();
      mesh->d3d11_buffer.vb = _d3d11_renderer->addVertexBuffer(d3d11_vert.size()*sizeof(Vertex_PNT), BufferAccess::STATIC, d3d11_vert.data());

      //todo:to many vfs!
      FormatDesc fd[3];
      fd[0].format = AttributeFormat::FORMAT_FLOAT;
      fd[0].size = 3;
      fd[0].stream = 0;
      fd[0].type = AttributeType::TYPE_VERTEX;
      fd[1].format = AttributeFormat::FORMAT_FLOAT;
      fd[1].size = 3;
      fd[1].stream = 0;
      fd[1].type = AttributeType::TYPE_NORMAL;
      fd[2].format = AttributeFormat::FORMAT_FLOAT;
      fd[2].size = 2;
      fd[2].stream = 0;
      fd[2].type = AttributeType::TYPE_TEXCOORD;

      mesh->d3d11_buffer.vf = _d3d11_renderer->addVertexFormat(fd, 3, mode_render_shader);
      mesh->release_data();
      
      scene_model->loaded_assistant_meshes[string(filename)].push_back(mesh);

    }
  }

  //brdf
  //OLPBRLambertianReflectionBrdf

  brdf.push_back(new MicrofacetReflectionBrdf());
}

void EditorSceneView::init_d3d11(int ww, int wh)
{
  _d3d11_renderer->setBlendState(_bs_enable, 0xffffffff);
  _d3d11_renderer->setDepthState(_ds_enable);
  _d3d11_renderer->setRasterizerState(_rs);


  _d3d11_renderer->setViewport(ww, wh);
  _d3d11_renderer->setFrameBuffer(_back_buffer_view, _d3d11_renderer->getDSV(_buffer_depth_stencil));
  _d3d11_renderer->changeToMainFramebuffer();

  this->ww = ww;
  this->wh = wh;

  //init_hit_sphere();
}

void EditorSceneView::update(f32 dt)
{

  if (Input::get_sys_key_down(WIP_MOUSE_LBUTTON))
  {
    f32 mx = Input::get_mouse_x();
    f32 my = Input::get_mouse_y();
    f32 vw = _d3d11_renderer->viewportWidth;
    f32 vh = _d3d11_renderer->viewportHeight;
    f32 rx = (mx / vw) * 2 - 1;
    f32 ry = ((vh - my) / vh) * 2 - 1;
    f32 dist = scene_camera->_near_panel;
    f32 fovy = scene_camera->_fovy;
    f32 ratio = scene_camera->_ratio;
    f32 cam_y = dist*RBMath::tan(DEG2RAD(fovy)*0.5f)*ry;
    f32 cam_x = dist*RBMath::tan(DEG2RAD(fovy)*0.5f)*ratio*rx;
    f32 cam_z = dist;
    RBVector3 cam_ray(cam_x, cam_y, cam_z);

    RBVector3 rp;
    RBMatrix p;
    scene_camera->get_view_matrix(p);
    rp = p.get_inverse_slow().transform_vector3(cam_ray);

    OLPBRRay_ ray;
    ray.o = scene_camera->_position;
    ray.d = rp.get_normalized();



    OLPBRItersc_ isc;
    MeshInstance* hp = scene_model->cast_ray(ray, isc);
    if (hp)
    {
      RBVector3 sp = ray.o + ray.d*isc.dist_;
	  if (hit_sphere)
		hit_sphere->local_2_world.set_translation(sp);
      if (selected_mesh_instance)
        selected_mesh_instance->selected = false;
      selected_mesh_instance = hp;


      ModelObjectInstance* model = (ModelObjectInstance*)selected_mesh_instance;
      selected_material_id = model->material_id;


      hp->selected = true;
    }
  }
  scene_camera->control_update(dt);

}

bool EditorSceneView::gui_update(f32 dt)
{
  static f32 vd[3] = {0};

  bool skip = false;
  if (ImGui::BeginMainMenuBar())
  {
    if (ImGui::BeginMenu("File"))
    {
      if (ImGui::MenuItem("Open..", "CTRL+O"))
      {
        do
        {
          if (1)
          {
            std::string name;
            OPENFILENAME ofn = { 0 };
            TCHAR strFilename[MAX_PATH] = { 0 };
            ofn.lStructSize = sizeof(OPENFILENAME);
            ofn.hwndOwner = NULL;
            ofn.lpstrFilter = TEXT("Scene Flie\0*.sce\0\0");
            ofn.nFilterIndex = 1;
            ofn.lpstrFile = strFilename;
            ofn.nMaxFile = sizeof(strFilename);
            ofn.lpstrInitialDir = ".\\scene";
            ofn.lpstrTitle = TEXT("请选择一个场景文件");
            ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
            if (GetOpenFileName(&ofn))
            {
              name = strFilename;
            }
            else
            {
              break;
            }
            if (name == scene_model->hold_scene)
              break;
            if (scene_model->hold_scene != "")
            {
              //清空场景
              reset();
              scene_model->clear_scene(_d3d11_renderer);
            }
            if (scene_model->load_scene(name.data(), _d3d11_renderer, pre_renderer, mode_render_shader))
            {
              scene_model->hold_scene = name;
            }
            else
            {
              scene_model->hold_scene = "";
            }
          }
        } while (false);
      }
      if (ImGui::MenuItem("Save..", "CTRL+S")) 
      {
        do
        {
          if (1)
          {


            if (scene_model->hold_scene == "")
            {
              if (scene_model->imported_file_names.size() == 0)
              {
                break;
              }
              else
              {
                std::string name;
                OPENFILENAME ofn = { 0 };
                TCHAR strFilename[MAX_PATH] = { 0 };
                ofn.lStructSize = sizeof(OPENFILENAME);
                ofn.hwndOwner = NULL;
                ofn.lpstrFilter = TEXT("Scene Flie\0*.sce\0\0");
                ofn.nFilterIndex = 1;
                ofn.lpstrFile = strFilename;
                ofn.nMaxFile = sizeof(strFilename);
                ofn.lpstrInitialDir = ".\\scene";
                ofn.lpstrTitle = TEXT("保存到");
                ofn.Flags = OFN_PATHMUSTEXIST | OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT;
                ofn.lpstrDefExt = TEXT("sce");
                if (GetSaveFileName(&ofn))
                {
                  name = strFilename;
                }
                else
                {
                  break;
                }
                scene_model->save_scene(name.data());
                scene_model->hold_scene = name;
              }
            }
            else
            {
              CHECK(scene_model->imported_file_names[0] != "");
              scene_model->save_scene(scene_model->hold_scene.data());

            }
          }
        } while (false);
      }
      if (ImGui::MenuItem("Import..", "CTRL+N")) 
      {
        do
        {
          if (1)
          {

            std::string name;
            OPENFILENAME ofn = { 0 };
            TCHAR strFilename[MAX_PATH] = { 0 };
            ofn.lStructSize = sizeof(OPENFILENAME);
            ofn.hwndOwner = NULL;
            ofn.lpstrFilter = TEXT("Obj Flie\0*.obj\0\0");
            ofn.nFilterIndex = 1;
            ofn.lpstrFile = strFilename;
            ofn.nMaxFile = sizeof(strFilename);
            ofn.lpstrInitialDir = ".\\model";
            ofn.lpstrTitle = TEXT("请选择一个OBJ文件");
            ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
            if (GetOpenFileName(&ofn))
            {
              name = strFilename;
            }
            else
            {
              break;
            }
            if (scene_model->hold_scene != "")
            {
              scene_model->save_scene(scene_model->hold_scene.data());
              reset();
              scene_model->clear_scene(_d3d11_renderer);

            }
            if (scene_model->imported_file_names.size() >= 1)
            {
              //MessageBox(NULL, "当前仅支持导入一个obj.", TEXT("Note"), 0);
              //break;
              reset();
              scene_model->clear_scene(_d3d11_renderer);

            }
            std::vector<Mesh*>& itk = scene_model->read_model(name.c_str(), 5, _d3d11_renderer, mode_render_shader);
            ObjectInstanceSet*  a = new ObjectInstanceSet();



            a->local_2_world.traslate(vd[0], vd[1], vd[2]);
            a->world_2_local = a->local_2_world.get_inverse_slow();
            for (int i = 0; i < itk.size(); ++i)
            {
              ModelObjectInstance* meshi = new ModelObjectInstance();
              meshi->hide = false;
              meshi->object_ref = itk[i];
              meshi->localtox.set_identity();
              meshi->xtolocal = meshi->localtox.get_inverse_slow();

              a->sub_object.push_back(meshi);
            }

            scene_model->objects.push_back(a);
          }




        } while (false);
      }

      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Tools"))
    {
      ImGui::MenuItem("Material Editor", "Ctrl+M", &show_material_editor);
      ImGui::Separator();
      ImGui::MenuItem("Rendering Setting", "Ctrl+R", &show_render);
      ImGui::MenuItem("Camera Setting", "Ctrl+C", &show_camera_setting);
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help"))
    {
      ImGui::MenuItem("Help", "Ctrl+H", &show_help);
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Debug"))
    {
      ImGui::MenuItem("Debug Tools", "Ctrl+D", &show_debug_tool);
      ImGui::EndMenu();
    }



    ImGui::Value("FPS", 1.f / dt);
    ImGui::EndMainMenuBar();

  }
  if (show_help)
  { 
    ImGui::Begin("Help",&show_help,ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::CollapsingHeader("Obj Export"))
    {
      ImGui::TextWrapped("Check flip X/Z!");
    }
    ImGui::SliderFloat2("ass scale", (f32*)&ass_scale, 0.f, 1.f);
    ImGui::End();
  }
  /*
  ImGui::SetNextWindowSize(ImVec2(100, 50));
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::Begin("FPS");
  ImGui::Value("FPS", 1.f / dt);
  ImGui::End();
  */

#if 0
  ImGui::Begin("");
  ImGui::InputFloat3("position", vd);

  do
  {
    if (ImGui::Button("Open..", ImVec2(100, 50)))
    {
      std::string name;
      OPENFILENAME ofn = { 0 };
      TCHAR strFilename[MAX_PATH] = { 0 };
      ofn.lStructSize = sizeof(OPENFILENAME);
      ofn.hwndOwner = NULL;
      ofn.lpstrFilter = TEXT("Scene Flie\0*.sce\0\0");
      ofn.nFilterIndex = 1;
      ofn.lpstrFile = strFilename;
      ofn.nMaxFile = sizeof(strFilename);
      ofn.lpstrInitialDir = NULL;
      ofn.lpstrTitle = TEXT("请选择一个场景文件");
      ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
      if (GetOpenFileName(&ofn))
      {
        name = strFilename;
      }
      else
      {
        break;
      }
      if (name == scene_model->hold_scene)
        break;
      if (scene_model->hold_scene != "")
      {
        //清空场景
		  reset();
		  scene_model->clear_scene(_d3d11_renderer);
      }
      if (scene_model->load_scene(name.data(), _d3d11_renderer, pre_renderer, mode_render_shader))
      {
        scene_model->hold_scene = name;
      }
      else
      {
        scene_model->hold_scene = "";
      }
    }
  } while (false);

  do
  {
	  if (ImGui::Button("Save..", ImVec2(100, 50)))
	  {


		  if (scene_model->hold_scene == "")
		  {
			  if (scene_model->imported_file_names.size() == 0)
			  {
				  break;
			  }
			  else
			  {
				  std::string name;
				  OPENFILENAME ofn = { 0 };
				  TCHAR strFilename[MAX_PATH] = { 0 };
				  ofn.lStructSize = sizeof(OPENFILENAME);
				  ofn.hwndOwner = NULL;
				  ofn.lpstrFilter = TEXT("Scene Flie\0*.sce\0\0");
				  ofn.nFilterIndex = 1;
				  ofn.lpstrFile = strFilename;
				  ofn.nMaxFile = sizeof(strFilename);
				  ofn.lpstrInitialDir = NULL;
				  ofn.lpstrTitle = TEXT("保存到");
				  ofn.Flags = OFN_PATHMUSTEXIST | OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT;
				  ofn.lpstrDefExt = TEXT("sce");
				  if (GetSaveFileName(&ofn))
				  {
					  name = strFilename;
				  }
				  else
				  {
					  break;
				  }
				  scene_model->save_scene(name.data());
				  scene_model->hold_scene = name;
			  }
		  }
		  else
		  {
			  CHECK(scene_model->imported_file_names[0]!="");
			  scene_model->save_scene(scene_model->hold_scene.data());

		  }
	  }
  }
  while (false);

  if (ImGui::Button("Save as..", ImVec2(100, 50)))
  {

  }

  do
  {
    bool bt = ImGui::Button("Import..", ImVec2(100, 50));
    if (bt)
    {

      std::string name;
      OPENFILENAME ofn = { 0 };
      TCHAR strFilename[MAX_PATH] = { 0 };
      ofn.lStructSize = sizeof(OPENFILENAME);
      ofn.hwndOwner = NULL;
      ofn.lpstrFilter = TEXT("Obj Flie\0*.obj\0\0");
      ofn.nFilterIndex = 1;
      ofn.lpstrFile = strFilename;
      ofn.nMaxFile = sizeof(strFilename);
      ofn.lpstrInitialDir = NULL;
      ofn.lpstrTitle = TEXT("请选择一个OBJ文件");
      ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
      if (GetOpenFileName(&ofn))
      {
        name = strFilename;
      }
      else
      {
        break;
      }
      if (scene_model->hold_scene != "")
      {
        scene_model->save_scene(scene_model->hold_scene.data());
        reset();
        scene_model->clear_scene(_d3d11_renderer);

      }
      if (scene_model->imported_file_names.size() >= 1)
      {
        //MessageBox(NULL, "当前仅支持导入一个obj.", TEXT("Note"), 0);
        //break;
        reset();
        scene_model->clear_scene(_d3d11_renderer);

      }
      std::vector<Mesh*>& itk = scene_model->read_model(name.c_str(), 5, _d3d11_renderer, mode_render_shader);
      ObjectInstanceSet*  a = new ObjectInstanceSet();



      a->local_2_world.traslate(vd[0], vd[1], vd[2]);
      a->world_2_local = a->local_2_world.get_inverse_slow();
      for (int i = 0; i < itk.size(); ++i)
      {
        ModelObjectInstance* meshi = new ModelObjectInstance();
        meshi->hide = false;
        meshi->object_ref = itk[i];
        meshi->localtox.set_identity();
        meshi->xtolocal = meshi->localtox.get_inverse_slow();

        a->sub_object.push_back(meshi);
      }

      scene_model->objects.push_back(a);
    }




  } while (false);




  ImGui::End();
#endif

  /*
  ImGui::SetNextWindowSize(ImVec2(256, 286));
  ImGui::SetNextWindowPos(ImVec2(1020, 426));
  ImGui::Begin("Object Properties");
  if (selected_mesh_instance)
  {
    selected_mesh_instance->on_gui(this);
  }
  ImGui::End();

  ImGui::SetNextWindowSize(ImVec2(256, 397));
  ImGui::SetNextWindowPos(ImVec2(1020,23));
  ImGui::Begin("Material Edit");
  on_material_edit_gui();
  ImGui::End();
  */

  /*
  on_show_result_gui();

  if (show_material_editor) on_create_material_gui();
  if (show_render) on_render_gui();
  //on_save_read_mat_gui();
  if(show_camera_setting) on_set_camera_gui();
  if(show_debug_tool) on_check_brdf_gui();
  */
  on_dock_gui();
  skip = ImGui::IsAnyItemHovered() || ImGui::IsMouseHoveringAnyWindow();




  return skip;

}

void EditorSceneView::render()
{

  _d3d11_renderer->setDepthState(_ds_enable);
  _d3d11_renderer->clear(true, true, true, reinterpret_cast<const float*>(&RBColorf::gray), 1, 1);

  _d3d11_renderer->setShader(mode_render_shader);

  RBMatrix v;
  scene_camera->get_view_matrix(v);
  RBMatrix p;
  scene_camera->get_perspective_matrix(p);
  _d3d11_renderer->setShaderConstant4x4f("v", v);
  _d3d11_renderer->setShaderConstant4x4f("o", p);
  _d3d11_renderer->setRasterizerState(_rs);
  _d3d11_renderer->setShaderConstant2f("scale", ass_scale);
  _d3d11_renderer->setShaderConstant1f("ass", 0.f);

  for (auto &it : scene_model->objects)
  {
    RBMatrix m = it->local_2_world;

    for (int i = 0; i < it->sub_object.size(); ++i)
    {
      ModelObjectInstance* its = (ModelObjectInstance*)it->sub_object[i];
      if (its->selected)
      {
        _d3d11_renderer->setShaderConstant4f("shade_param", RBColorf::red);
      }
      else
      {
        if (its->material_id == -1)
          _d3d11_renderer->setShaderConstant4f("shade_param", RBColorf::white);
        else
        {
          _d3d11_renderer->setShaderConstant4f("shade_param", scene_model->material_lib[its->material_id]->material_data->reg_color);
        }


      }
      auto&buffer = its->object_ref->d3d11_buffer;

      RBMatrix l2x = its->localtox;
      _d3d11_renderer->setVertexFormat(buffer.vf);
      RBMatrix tm = l2x*m;
      _d3d11_renderer->setShaderConstant4x4f("m", tm);
      _d3d11_renderer->setVertexBuffer(0, buffer.vb);
      _d3d11_renderer->setIndexBuffer(buffer.ib);
      _d3d11_renderer->apply();
      _d3d11_renderer->drawElements(Primitives::PRIM_TRIANGLES, 0, buffer.ib_size, 0, 0);
    }

  }

  //draw assistant
  _d3d11_renderer->setShaderConstant1f("ass", 1.f);
  for (auto &it : scene_model->assistant_objects)
  {
    RBMatrix m = it->local_2_world;

    for (auto& its : it->sub_object)
    {
      if (its->selected)
      {
        _d3d11_renderer->setShaderConstant4f("shade_param", RBColorf::red);
      }
      else
      {
        _d3d11_renderer->setShaderConstant4f("shade_param", RBColorf::white);

      }
      auto&buffer = its->object_ref->d3d11_buffer;

      RBMatrix l2x = its->localtox;
      _d3d11_renderer->setVertexFormat(buffer.vf);
      RBMatrix tm = l2x*m;
      _d3d11_renderer->setShaderConstant4x4f("m", tm);
      _d3d11_renderer->setVertexBuffer(0, buffer.vb);
      _d3d11_renderer->setIndexBuffer(buffer.ib);
      _d3d11_renderer->apply();
      _d3d11_renderer->drawElements(Primitives::PRIM_TRIANGLES, 0, buffer.ib_size, 0, 0);
    }

  }
}

void EditorSceneView::reset()
{
	cur_metadata.clear();
	scene_camera->set_position(0, 0, 0);
	RBVector4 position(0, 0, -1);
	scene_camera->set_target(position);
	scene_camera->set_fov_y(60);
	scene_camera->set_ratio(16.f / 9.f);
	scene_camera->set_near_panel(0.01f);
	scene_camera->set_far_panel(2000.f);
	_cam_move_speed = RBVector2(50, 50);
	_cam_rotate_speed = RBVector2(5, 5);

	selected_mesh_instance = nullptr;
	selected_material_id = -1;


}

bool EditorSceneView::on_create_material_gui()
{
  {
    //ImGui::Begin("Create Material");




    ImGui::PushStyleVar(ImGuiStyleVar_ChildWindowRounding, 5.0f);
    ImGui::BeginChild("Sub2", ImVec2(0, 300), true);

    ImGui::Columns(ImGui::GetWindowWidth() / 100);
    static int s_id = -1;
    for (int i = 0; i < scene_model->material_lib.size(); i++)
    {
      //imgui regard the button as the same one which has the same label including imageButton
      auto* mat = scene_model->material_lib[i];
      if (!mat )
      {
        char s[32] = { 0 };
        sprintf(s, "empty \nslot\n %d", i);
        if (ImGui::Button(s, ImVec2(50, 50)))
        {
          printf("select %d\n", i);
          s_id = i;
          selected_material_id = i;
          ImGui::OpenPopup("select mat");

        }
      }
      else
      {
        if (mat->_ui_show_tex == -1)
        {
          if (ImGui::Button("Blend Mat"))
          {
            selected_material_id = i;
            s_id = i;
          }
        }
        else
        if (ImGui::ImageButton(_d3d11_renderer->textures[mat->_ui_show_tex].srv, ImVec2(50, 50)))
        {
          selected_material_id = i;
          s_id = i;
        }
      }
      ImGui::NextColumn();


    }
    //没有材质
    const char* items[] = { "Metal", "MicrofactGlass", "Diffuse",
      "SimpleGlass", "FresnelBlend", "Plastic", "Mirror", "Hair", "Substrate", "Translucent", "Uber", "Mix", "Disney", "Blank", "Disney Alpha" };
    static int selected_item_idx = -1;
    //ImGui::Combo("Select a material", &selected_item_idx, items, 5);
    if (ImGui::BeginPopup("select mat"))
    {
      ImGui::Separator();
      for (int i = 0; i < ARRAY_COUNT(items); i++)
        if (ImGui::Selectable(items[i]))
        {
          auto*& mt = scene_model->material_lib[selected_material_id];
          if (mt)
          {
            mt->offline_material->release_parameter();
            delete mt->offline_material;
            delete mt->material_data;
            mt->release(_d3d11_renderer);
            delete mt;
            mt = nullptr;
          }
          mt = new LibMaterial(_d3d11_renderer, pre_renderer);
          switch (i)
          {
          case MaterialMetaData::E_MM_METAL:
            mt->material_data = new MetalMaterialMetaData();
            mt->name = "Metal";
            break;
          case MaterialMetaData::E_MM_GLASS:
            mt->material_data = new GlassMaterialMetaData();
            mt->name = "Glass";

            break;
          case MaterialMetaData::E_MM_MATTE:
            mt->material_data = new MatteMaterialMetaData();
            mt->name = "Diffuse";

            break;
          case MaterialMetaData::E_MM_SIMPLEGLASS:
            mt->material_data = new SimpleGlassMaterialMetaData();
            mt->name = "Simple Glass";


            break;
          case MaterialMetaData::E_MM_FRESNEL_BLEND:
            mt->material_data = new FresnelBlendMaterialMetaData();
            mt->name = "Fresnel Blend";


            break;
          case MaterialMetaData::E_MM_PLASTIC:
            mt->material_data = new PlasticMaterialMetaData();
            mt->name = "Plastic";

            break;
          case MaterialMetaData::E_MM_MIRROR:
            mt->material_data = new MirrorMaterialMetaData();
            mt->name = "Mirror";

            break;
          case MaterialMetaData::E_MM_HAIR:
            mt->material_data = new HairMaterialMetaData();
            mt->name = "hair";


            break;
          case MaterialMetaData::E_MM_SUBSTRATE:
            mt->material_data = new SubstrateMaterialMetaData();
            mt->name = "Substrate";


            break;
          case MaterialMetaData::E_MM_TRANSLUCENT:
            mt->material_data = new TranslucentMaterialMetaData();
            mt->name = "Translucent";


            break;
          case MaterialMetaData::E_MM_UBER:
            mt->material_data = new UberMaterialMetaData();
            mt->name = "Parameter";


            break;
          case MaterialMetaData::E_MM_DISNEY:
            mt->name = "Disney";
            mt->material_data = new DisneyMaterialMetaData(false);
            break;

          case MaterialMetaData::E_MM_MIX:
            mt->material_data = new MixMaterialMetaData(this,mt);
            mt->release(_d3d11_renderer);
            mt->name = "Mix";
            break;

          case MaterialMetaData::E_MM_BLANK:
            mt->material_data = new BlankMaterialMetaData();
            mt->name = "blank";
            break;

          case MaterialMetaData::E_MM_ALPHA_DISNEY:
            mt->material_data = new DisneyMaterialMetaData(true);
            mt->name = "Alpha disney";
            break;
            
          default:
            printf("funcking material!\n");
            break;
          }
        }
      ImGui::EndPopup();
    }

    ImGui::EndChild();
    ImGui::PopStyleVar();

    if (s_id != -1)
      ImGui::Text("Selected [%d]", s_id);

    //ImGui::End();
  }
  return true;
}

bool EditorSceneView::on_material_edit_gui()
{
  if (selected_material_id != -1 && scene_model->material_lib[selected_material_id] && scene_model->material_lib[selected_material_id]->material_data)
  {
    char s[1024] = {0};
    memcpy(s, scene_model->material_lib[selected_material_id]->name.c_str(), scene_model->material_lib[selected_material_id]->name.size());
    ImGui::InputText("name", s, scene_model->material_lib[selected_material_id]->name.size());
    scene_model->material_lib[selected_material_id]->name = s;

    bool a = scene_model->material_lib[selected_material_id]->material_data->on_gui(_d3d11_renderer, pre_renderer, scene_model->material_lib[selected_material_id]->_ui_show_tex);
    if (ImGui::Button("generate offline material"))
    {
      if (scene_model->material_lib[selected_material_id]->offline_material)
      {
        scene_model->material_lib[selected_material_id]->offline_material->release_parameter();
        delete scene_model->material_lib[selected_material_id]->offline_material;
        scene_model->material_lib[selected_material_id]->offline_material = nullptr;
      }
      scene_model->material_lib[selected_material_id]->offline_material = scene_model->material_lib[selected_material_id]->material_data->create_material();
      scene_model->material_lib[selected_material_id]->material_data->change_applied();
    }
    bool b = (scene_model->material_lib[selected_material_id]->offline_material == nullptr) ? false : true;
    ImGui::Checkbox("offline mat", &b);
    return true;
  }
  return false;
}



bool EditorSceneView::on_render_gui()
{
  //ImGui::Begin("Render");
  static int v[2] = { 800, 600 };
  ImGui::InputInt2("Res", v);
  ImGui::Checkbox("Use IBL", &is_use_ibl);
  if (ImGui::Button("Change IBL"))
  {
    do
    {
      std::string name;
      OPENFILENAME ofn = { 0 };
      TCHAR strFilename[MAX_PATH] = { 0 };
      ofn.lStructSize = sizeof(OPENFILENAME);
      ofn.hwndOwner = NULL;
      ofn.lpstrFilter = TEXT("EXR Flie\0*.exr\0\0");
      ofn.nFilterIndex = 1;
      ofn.lpstrFile = strFilename;
      ofn.nMaxFile = sizeof(strFilename);
      ofn.lpstrInitialDir = NULL;
      ofn.lpstrTitle = TEXT("请选择一个EXR文件");
      ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
      if (GetOpenFileName(&ofn))
      {
        name = strFilename;
      }
      else
      {
        break;
      }

      prod_renderer->change_ibl(name.data());
    } while (0);


  }
  const char items[] = { "Path Tracer\0Eye Light\0Direct Lighting\0IGI\0SPPM\0BDPT\0VCM\0PTDebug\0"};
  static int selected_item_idx = 0;
  static f32 thresold = 1;
  //ImGui::Combo("Select a material", &selected_item_idx, items, 5);
  /*
  if (ImGui::BeginPopup("Select a integrator"))
  {
    for (int i = 0; i < ARRAY_COUNT(items); i++)
      if (ImGui::Selectable(items[i]))
      {
        selected_item_idx = i;
      }
    ImGui::EndPopup();
  }
  */
  ImGui::Combo("Integrator", &selected_item_idx, items);
  if (selected_item_idx == 7)
  {
    ImGui::SliderFloat("Thresold", &thresold,0.f,10.f);
    ImGui::ColorButton(ImVec4(0, 0, 0, 1)); ImGui::SameLine(); ImGui::Text("<%f",thresold);
    ImGui::ColorButton(ImVec4(0, 1, 0, 1)); 
    ImGui::SameLine();
    ImGui::ColorButton(ImVec4(1,1,0,1));
    ImGui::SameLine();
    ImGui::ColorButton(ImVec4(1,0.6,0,1));
    ImGui::SameLine();
    ImGui::ColorButton(ImVec4(1,0.3,0,1));
    ImGui::SameLine();
    ImGui::ColorButton(ImVec4(1,0,0,1));
    ImGui::Text("%f~%f", thresold,thresold + 1); 
    ImGui::Text("%f~%f",  thresold + 1,thresold+2); 
    ImGui::Text("%f~%f", thresold + 2,thresold+3); 
    ImGui::Text("%f~%f", thresold + 3,thresold+4); 
    ImGui::Text(">%f", thresold + 4); 

  }

  static int spp = 1;
  ImGui::InputInt("SPP", &spp);
  if (ImGui::Button("Render Image"))
  {

    _d3d11_renderer->removeTexture(_ui_show_tex);
    {
      D3D11_TEXTURE2D_DESC desc;
      rb_memzero(&desc, sizeof(desc));
      desc.Width = v[0];
      desc.Height = v[1];
      desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
      desc.MipLevels = 1;
      desc.SampleDesc.Count = 1;
      desc.SampleDesc.Quality = 0;
      desc.Usage = D3D11_USAGE_DYNAMIC;
      desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
      desc.ArraySize = 1;
      desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
      HRESULT res = _d3d11_renderer->device->CreateTexture2D(&desc, 0, &tex);
      if (res != S_OK)
      {
        g_logger->debug(WIP_ERROR, "Couldn't create dynamic texture!");
      }

      _ui_show_tex = _d3d11_renderer->addTexture(tex, 0);
    }

    prod_renderer->convergence_thresold = thresold;
    prod_renderer->set_render_res(v[0], v[1]);
    prod_renderer->set_camera(*scene_camera);
    prod_renderer->collect_object(scene_model->objects, scene_model->material_lib);
    prod_renderer->render(spp, _ui_show_tex, _d3d11_renderer, (ProductionRenderer::IntegType) selected_item_idx,is_use_ibl);

  }

  ImGui::Separator();
  //if (ImGui::Button("Build Scene"))
  //{
    //prod_renderer->collect_object(scene_model->objects, scene_model->material_lib);
  //}

  //if (ImGui::Button("Update Material"))
  //{
    //prod_renderer->update_material(scene_model->objects, scene_model->material_lib);
  //}


  //ImGui::End();
  return false;
}

void EditorSceneView::on_show_result_gui()
{
  //ImGui::Begin("Result");
  int w = prod_renderer->cam->film_->xres;
  int h = prod_renderer->cam->film_->yres;

  static f32 scale = 1;
  ImGui::SliderFloat("", &scale, 0.1, 2.0);

  ImGui::Image(_d3d11_renderer->textures[_ui_show_tex].srv, ImVec2(w*scale,h*scale));

  ImGui::Separator();
  if (ImGui::Button("Glow"))
  {
    prod_renderer->cam->film_->restore();
    prod_renderer->cam->film_->tone_mapping();
    prod_renderer->cam->film_->post();
    unsigned char* dao3 = prod_renderer->cam->film_->output_pix();
    RBColor32* dao = new RBColor32[prod_renderer->cam->film_->xres* prod_renderer->cam->film_->yres];
    for (int i = 0; i < prod_renderer->cam->film_->xres* prod_renderer->cam->film_->yres; ++i)
    {
      dao[i].r = dao3[i * 3 + 2];
      dao[i].g = dao3[i * 3 + 1];
      dao[i].b = dao3[i * 3];
      dao[i].a = 255;

    }

    D3D11_MAPPED_SUBRESOURCE resource;
    _d3d11_renderer->context->Map(_d3d11_renderer->textures[_ui_show_tex].texture,
      0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
    u8* da = reinterpret_cast<u8*>(resource.pData);
    u8* dao_c = (u8*)dao;
    //GPU有自己的内存布局，不要整块拷贝
    for (int a = 0; a < prod_renderer->cam->film_->yres; ++a)
    {
      memcpy(da, dao_c, sizeof(RBColor32)*prod_renderer->cam->film_->xres);
      da += resource.RowPitch;
      dao_c += prod_renderer->cam->film_->xres*sizeof(RBColor32);
    }
    _d3d11_renderer->context->Unmap(_d3d11_renderer->textures[_ui_show_tex].texture, 0);
    delete[] dao;
  }
  if (ImGui::Button("Clear post"))
  {
    prod_renderer->cam->film_->restore();

    unsigned char* dao3 = prod_renderer->cam->film_->output_pix();
    RBColor32* dao = new RBColor32[prod_renderer->cam->film_->xres* prod_renderer->cam->film_->yres];
    for (int i = 0; i < prod_renderer->cam->film_->xres* prod_renderer->cam->film_->yres; ++i)
    {
      dao[i].r = dao3[i * 3 + 2];
      dao[i].g = dao3[i * 3 + 1];
      dao[i].b = dao3[i * 3];
      dao[i].a = 255;

    }

    D3D11_MAPPED_SUBRESOURCE resource;
    _d3d11_renderer->context->Map(_d3d11_renderer->textures[_ui_show_tex].texture,
      0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
    u8* da = reinterpret_cast<u8*>(resource.pData);
    u8* dao_c = (u8*)dao;
    //GPU有自己的内存布局，不要整块拷贝
    for (int a = 0; a < prod_renderer->cam->film_->yres; ++a)
    {
      memcpy(da, dao_c, sizeof(RBColor32)*prod_renderer->cam->film_->xres);
      da += resource.RowPitch;
      dao_c += prod_renderer->cam->film_->xres*sizeof(RBColor32);
    }
    _d3d11_renderer->context->Unmap(_d3d11_renderer->textures[_ui_show_tex].texture, 0);
    delete[] dao;
  }
  if (ImGui::Button("Save"))
  {
    std::string name = "";
    if(RBPathTool::save_file(name,".\\Image"))
    {
      prod_renderer->cam->film_->save_pix(name.data());
    }

  }
  //ImGui::End();
}

void EditorSceneView::on_save_read_mat_gui()
{
	/*
  ImGui::Begin("save");
  if (ImGui::Button("Save"))
  {
    scene_model->save_scene();
  }
  if (ImGui::Button("Load"))
  {
    scene_model->load_scene(_d3d11_renderer, pre_renderer);
  }
  ImGui::End();
  */
}

void EditorSceneView::on_set_camera_gui()
{
  //ImGui::Begin("Camera Parameter");
  ImGui::LabelText("","Current Speed");
  ImGui::LabelText("", "x:%f,y:%f", scene_camera->_cam_move_speed.x, scene_camera->_cam_move_speed.y);
  ImGui::LabelText("", "Current FOVY");
  ImGui::LabelText("", "%f", scene_camera->_fovy);
  ImGui::LabelText("", "Current Ratio");
  ImGui::LabelText("", "%f", scene_camera->_ratio);
  ImGui::InputFloat("X", &scene_camera->_cam_move_speed.x);
  ImGui::InputFloat("Y", &scene_camera->_cam_move_speed.y);
  ImGui::SliderFloat("FOV Y",&scene_camera->_fovy, 0.f, 180.f, "%.1f");
  ImGui::SliderFloat("Far panel", &scene_camera->_far_panel, scene_camera->_near_panel, 10000.f);
  ImGui::SliderFloat("Ratio", &scene_camera->_ratio,1.f,2.0f);
  if (ImGui::Button("16:9(window)")) scene_camera->_ratio =16.f/9.f;
  if (ImGui::Button("16:10")) scene_camera->_ratio = 1.6f;
  if (ImGui::Button("4:3")) scene_camera->_ratio = 4.f/3.f;
  if (ImGui::Button("1:1")) scene_camera->_ratio = 1.f;



  //ImGui::End();
}

void EditorSceneView::on_check_brdf_gui()
{
  //ImGui::Begin("brdf check");
  for (auto it : brdf)
  {
    it->on_gui();
  }
  //ImGui::End();

  ImGui::Separator();
  if(ImGui::Button("Change preview ibl"))
  {
    do
    {
      std::string name;
      OPENFILENAME ofn = { 0 };
      TCHAR strFilename[MAX_PATH] = { 0 };
      ofn.lStructSize = sizeof(OPENFILENAME);
      ofn.hwndOwner = NULL;
      ofn.lpstrFilter = TEXT("EXR Flie\0*.exr\0\0");
      ofn.nFilterIndex = 1;
      ofn.lpstrFile = strFilename;
      ofn.nMaxFile = sizeof(strFilename);
      ofn.lpstrInitialDir = NULL;
      ofn.lpstrTitle = TEXT("请选择一个EXR文件");
      ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;
      if (GetOpenFileName(&ofn))
      {
        name = strFilename;
      }
      else
      {
        break;
      }

      pre_renderer->change_ibl(name.data());
    } while (0);
  }

}





class NodeBase : public ImGui::Node {
protected:
  mutable std::string type;
public:
  ImGui::NodeGraphEditor* nge = nullptr;
  std::string info;
  const char* getInfo() const override
  {
    if (type.empty()) {
      type = std::string(NodeTypeStr[getType()]) + "\n" + info;
    }
    return type.c_str();
  }

  const char* getTooltip() const override
  {
    return info.c_str();
  }

  virtual OLPBRMaterial* get_material() const
  {
    return nullptr; 
  }

  virtual bool onInit() { return true; }
  bool setup(ImGui::NodeGraphEditor* nge, const ImVec2& pos, const char* inputSlots, const char* outputSlots, NodeType nodeTypeID)
  {
    init(NodeTypeStr[int(nodeTypeID)], pos, inputSlots, outputSlots, int(nodeTypeID));
    this->nge = nge;
    this->info = NodeTooltipStr[int(nodeTypeID)];
    return this->onInit();
  }
  NodeBase* copy();
  bool acceptsLink(Node* inputNode) override
  {
    return (inputNode) != nullptr;
  }
  void onEdited() override
  {
    markDirty();
  }

  virtual void markDirty()
  {
    ImVector<ImGui::Node *> nodes;
    nge->getOutputNodesForNodeAndSlot(this, 0, nodes);
    for (ImGui::Node* node : nodes) {
      NodeBase* n = dynamic_cast<NodeBase*>(node);
      if (n == nullptr) {
        node->onEdited();
        continue;
      }
      n->markDirty();
    }
  }
};


class ConstantNode : public NodeBase {
public:
  float constant = 0.0f;
public:
  static ConstantNode* Create(const ImVec2& pos, ImGui::NodeGraphEditor* nge)
  {
    ConstantNode* node = (ConstantNode*)ImGui::MemAlloc(sizeof(ConstantNode)); new(node)ConstantNode();
    if (!node->setup(nge, pos, nullptr, "constant", NodeType::Constant)) {
      return nullptr;
    }
    node->fields.addField(&node->constant, 1, "Value", nullptr, 8, -10,10);
    return node;
  }
};


class MixNode : public NodeBase
{ 
  RBColorf c = RBColorf::white;
  TextureID _ui_show_tex = -1;
  ID3D11Texture2D* tex = nullptr;
  int pre_spp = 1;
  OLPBRMaterial* gen_mat = nullptr;
protected: 
	bool canBeCopied() const override { return false; } 

public: 

  bool render(float nodeWidth) override
  {
    const bool retVal = NodeBase::render(nodeWidth);
    node_editor_param *node_param = (node_editor_param*)nge->user_ptr;
    EditorSceneView* scene_view = node_param->editor_view;
    LibMaterial* meta_data = node_param->meta_data;


    if (_ui_show_tex == -1)
    {
      D3D11_TEXTURE2D_DESC desc;
      rb_memzero(&desc, sizeof(desc));
      desc.Width = scene_view->pre_renderer->cam->film_->xres;
      desc.Height = scene_view->pre_renderer->cam->film_->yres;
      desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
      desc.MipLevels = 1;
      desc.SampleDesc.Count = 1;
      desc.SampleDesc.Quality = 0;
      desc.Usage = D3D11_USAGE_DYNAMIC;
      desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
      desc.ArraySize = 1;
      desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
      HRESULT res = scene_view->_d3d11_renderer->device->CreateTexture2D(&desc, 0, &tex);
      if (res != S_OK)
      {
        g_logger->debug(WIP_ERROR, "Couldn't create dynamic texture!");
      }

      _ui_show_tex = scene_view->_d3d11_renderer->addTexture(tex, 0);

      if (meta_data)
      {
        //meta_data->release(scene_view->_d3d11_renderer);

        meta_data->_ui_show_tex = _ui_show_tex;
        meta_data->tex = tex;

      }
    }

    ImGui::SliderInt("SPP", &pre_spp, 1, 500);

    if (ImGui::Button("Preview"))
    {
      auto* mat = get_material();
      if (gen_mat)
      {
        delete gen_mat;
        gen_mat = nullptr;

      }
      gen_mat = mat;
      scene_view->pre_renderer->render(mat,pre_spp);
      //delete mat;
      unsigned char* dao3 = scene_view->pre_renderer->cam->film_->output_pix();
      RBColor32* dao = new RBColor32[scene_view->pre_renderer->cam->film_->xres* scene_view->pre_renderer->cam->film_->yres];
      for (int i = 0; i < scene_view->pre_renderer->cam->film_->xres* scene_view->pre_renderer->cam->film_->yres; ++i)
      {
        int y = i / scene_view->pre_renderer->cam->film_->xres;
        int x = i % scene_view->pre_renderer->cam->film_->xres;

          dao[i].r = dao3[i * 3 + 2];
          dao[i].g = dao3[i * 3 + 1];
          dao[i].b = dao3[i * 3];
          dao[i].a = 255;
        
      }

      D3D11_MAPPED_SUBRESOURCE resource;
      scene_view->_d3d11_renderer->context->Map(scene_view->_d3d11_renderer->textures[_ui_show_tex].texture,
        0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
      u8* da = reinterpret_cast<u8*>(resource.pData);
      u8* dao_c = (u8*)dao;
      for (int a = 0; a < scene_view->pre_renderer->cam->film_->yres; ++a)
      {
        memcpy(da, dao_c, sizeof(RBColor32)*scene_view->pre_renderer->cam->film_->xres);
        da += resource.RowPitch;
        dao_c += scene_view->pre_renderer->cam->film_->xres*sizeof(RBColor32);
      }
      scene_view->_d3d11_renderer->context->Unmap(scene_view->_d3d11_renderer->textures[_ui_show_tex].texture, 0);

      /*
      D3D11_MAPPED_SUBRESOURCE resource;
      scene_view->_d3d11_renderer->context->Map(scene_view->_d3d11_renderer->textures[_ui_show_tex].texture,
        0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
      u8* da = reinterpret_cast<u8*>(resource.pData);
      u8* dao_c = (u8*)dao;
      for (int a = 0; a < scene_view->pre_renderer->cam->film_->yres; ++a)
      {
        memcpy(da, dao_c, sizeof(RBColor32)*scene_view->pre_renderer->cam->film_->xres);
        da += resource.RowPitch;
        dao_c += scene_view->pre_renderer->cam->film_->xres*sizeof(RBColor32);
      }
      scene_view->_d3d11_renderer->context->Unmap(scene_view->_d3d11_renderer->textures[_ui_show_tex].texture, 0);
      */
      delete[] dao;
    }

    if (_ui_show_tex>=0)
    {
      ImGui::Image(scene_view->_d3d11_renderer->textures[_ui_show_tex].srv, ImVec2(200,200));
    }

    if (ImGui::Button("Create Material"))
    {
      if (!gen_mat)
      {
        auto* mat = get_material();
        gen_mat = mat;
      }
      meta_data->offline_material = gen_mat;
      
    }

    return retVal;
  }

  virtual OLPBRMaterial* get_material() const
  {
    const int n = getNumInputSlots();
    CHECK(n==2);
    OLPBRMaterial* mats[2];
    for (int i = 0; i < n; ++i) 
    {
      NodeBase* in = dynamic_cast<NodeBase*>(nge->getInputNodeForNodeAndSlot(this, i));
      if (in == nullptr) 
      {
        continue;
      }
      mats[i] = in->get_material();
    }
    OLPBRConstantTexture<RBColorf>* colt = new OLPBRConstantTexture<RBColorf>(c);
    OLPBRMixMaterial * mix_mat = new OLPBRMixMaterial(mats[0], mats[1], colt);


    return mix_mat;
  }
  static MixNode* Create(const ImVec2& pos, ImGui::NodeGraphEditor* nge)
  { 
    MixNode* node = (MixNode*)ImGui::MemAlloc(sizeof(MixNode)); new(node)MixNode();
		if (!node->setup(nge, pos, "val1;val2", "result", NodeType::Mix)) 
    { 
			return nullptr; 
    } 
		node->setOpen(true); 
    node->fields.addFieldColor((f32*)&node->c, false, "weight", "val1*weight+(1-weight)*val2");
		return node; 
	} 
};

class MixMaskNode : public NodeBase
{
  std::string mask_path="";
  TextureID _ui_show_tex = -1;
  ID3D11Texture2D* tex = nullptr;
  int pre_spp = 1;
  OLPBRMaterial* gen_mat = nullptr;
protected:
  bool canBeCopied() const override { return false; }

public:

  bool render(float nodeWidth) override
  {
    const bool retVal = NodeBase::render(nodeWidth);
    node_editor_param *node_param = (node_editor_param*)nge->user_ptr;
    EditorSceneView* scene_view = node_param->editor_view;
    LibMaterial* meta_data = node_param->meta_data;

    if (_ui_show_tex == -1)
    {
      D3D11_TEXTURE2D_DESC desc;
      rb_memzero(&desc, sizeof(desc));
      desc.Width = scene_view->pre_renderer->cam->film_->xres;
      desc.Height = scene_view->pre_renderer->cam->film_->yres;
      desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
      desc.MipLevels = 1;
      desc.SampleDesc.Count = 1;
      desc.SampleDesc.Quality = 0;
      desc.Usage = D3D11_USAGE_DYNAMIC;
      desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
      desc.ArraySize = 1;
      desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
      HRESULT res = scene_view->_d3d11_renderer->device->CreateTexture2D(&desc, 0, &tex);
      if (res != S_OK)
      {
        g_logger->debug(WIP_ERROR, "Couldn't create dynamic texture!");
      }

      _ui_show_tex = scene_view->_d3d11_renderer->addTexture(tex, 0);

      if (meta_data)
      {
        //meta_data->release(scene_view->_d3d11_renderer);
        meta_data->_ui_show_tex = _ui_show_tex;
        meta_data->tex = tex;

      }
    }

    ImGui::Text(mask_path.data());
    ImGui::SameLine();
    if (ImGui::Button("Mask.."))
    {
      RBPathTool::open_file(mask_path, "Image Flie\0*.png;*.jpg\0\0");
    }

    ImGui::SliderInt("SPP", &pre_spp, 1, 500);

    if (ImGui::Button("Preview"))
    {
      auto* mat = get_material();
      if (gen_mat)
      {
        delete gen_mat;
        gen_mat = nullptr;

      }
      gen_mat = mat;
      scene_view->pre_renderer->render(mat, pre_spp);
      //delete mat;
      unsigned char* dao3 = scene_view->pre_renderer->cam->film_->output_pix();
      RBColor32* dao = new RBColor32[scene_view->pre_renderer->cam->film_->xres* scene_view->pre_renderer->cam->film_->yres];
      for (int i = 0; i < scene_view->pre_renderer->cam->film_->xres* scene_view->pre_renderer->cam->film_->yres; ++i)
      {
        int y = i / scene_view->pre_renderer->cam->film_->xres;
        int x = i % scene_view->pre_renderer->cam->film_->xres;

        dao[i].r = dao3[i * 3 + 2];
        dao[i].g = dao3[i * 3 + 1];
        dao[i].b = dao3[i * 3];
        dao[i].a = 255;

      }

      D3D11_MAPPED_SUBRESOURCE resource;
      scene_view->_d3d11_renderer->context->Map(scene_view->_d3d11_renderer->textures[_ui_show_tex].texture,
        0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
      u8* da = reinterpret_cast<u8*>(resource.pData);
      u8* dao_c = (u8*)dao;
      for (int a = 0; a < scene_view->pre_renderer->cam->film_->yres; ++a)
      {
        memcpy(da, dao_c, sizeof(RBColor32)*scene_view->pre_renderer->cam->film_->xres);
        da += resource.RowPitch;
        dao_c += scene_view->pre_renderer->cam->film_->xres*sizeof(RBColor32);
      }
      scene_view->_d3d11_renderer->context->Unmap(scene_view->_d3d11_renderer->textures[_ui_show_tex].texture, 0);
      delete[] dao;
    }

    if (_ui_show_tex >= 0)
    {
      ImGui::Image(scene_view->_d3d11_renderer->textures[_ui_show_tex].srv, ImVec2(200, 200));
    }

    if (ImGui::Button("Create Material"))
    {
      if (!gen_mat)
      {
        auto* mat = get_material();
        gen_mat = mat;
      }
      meta_data->offline_material = gen_mat;
    }

    return retVal;
  }

  virtual OLPBRMaterial* get_material() const
  {
    if (mask_path == "")
      return nullptr;
    const int n = getNumInputSlots();
    CHECK(n == 2);
    OLPBRMaterial* mats[2];
    for (int i = 0; i < n; ++i)
    {
      NodeBase* in = dynamic_cast<NodeBase*>(nge->getInputNodeForNodeAndSlot(this, i));
      if (in == nullptr)
      {
        continue;
      }
      mats[i] = in->get_material();
    }
    OLPBRTexture<RBColorf>* colt = new OLPBRImageTexture(mask_path.data());
    OLPBRMixMaterial * mix_mat = new OLPBRMixMaterial(mats[0], mats[1], colt);


    return mix_mat;
  }
  static MixMaskNode* Create(const ImVec2& pos, ImGui::NodeGraphEditor* nge)
  {
    MixMaskNode* node = (MixMaskNode*)ImGui::MemAlloc(sizeof(MixMaskNode)); new(node)MixMaskNode();
    if (!node->setup(nge, pos, "val1;val2", "result", NodeType::MixMask))
    {
      return nullptr;
    }
    node->setOpen(true);
    return node;
  }
};


class MulNode : public NodeBase
{
protected:
  bool canBeCopied() const override { return false; }
public:
  static MulNode* Create(const ImVec2& pos, ImGui::NodeGraphEditor* nge)
  {
    MulNode* node = (MulNode*)ImGui::MemAlloc(sizeof(MulNode)); new(node)MulNode();
    if (!node->setup(nge, pos, "val1;val1", "result", NodeType::Mul))
    {
      return nullptr;
    }
    node->setOpen(false);
    return node;
  }
};



class ConstantColorNode : public NodeBase {
public:
  RBColorf constant = 0.0f;
public:
  static ConstantColorNode* Create(const ImVec2& pos, ImGui::NodeGraphEditor* nge)
  {
    ConstantColorNode* node = (ConstantColorNode*)ImGui::MemAlloc(sizeof(ConstantColorNode)); new(node)ConstantColorNode();
    if (!node->setup(nge, pos, nullptr, "constant color", NodeType::ConstantColor)) 
    {
      return nullptr;
    }
    node->fields.addField((f32*)&node->constant, 4, "Color", nullptr, 3, 0, 1);
    return node;
  }
};



class MaterialRefNode : public NodeBase {
public:
  int mat_id_ref = -1;
  virtual OLPBRMaterial* get_material() const
  {
    node_editor_param *node_param = (node_editor_param*)nge->user_ptr;
    EditorSceneView* scene_view = node_param->editor_view;
    LibMaterial* meta_data = node_param->meta_data;

    if (mat_id_ref == -1)
      return nullptr;
    auto* mat = scene_view->scene_model->material_lib[mat_id_ref];
    if (!mat)
    {
      return nullptr;
    }
    if (!mat->offline_material)
    {
      mat->offline_material = mat->material_data->create_material();
    }
    return mat->offline_material;
  }
  bool render(float nodeWidth) override
  {
    const bool retVal = NodeBase::render(nodeWidth);
    node_editor_param *node_param = (node_editor_param*)nge->user_ptr;
    EditorSceneView* scene_view = node_param->editor_view;
    LibMaterial* meta_data = node_param->meta_data;

    if (startBrowseDialogNextFrame)
    {

      startBrowseDialogNextFrame = false;

    }

    if (ImGui::Button("Select a Material"))
    {
      ImGui::OpenPopup("select mat");

    }

    ImGui::PushStyleVar(ImGuiStyleVar_ChildWindowRounding, 5.0f);

    if (ImGui::BeginPopup("select mat"))
    {

      for (int i = 0; i < scene_view->scene_model->material_lib.size(); i++)
      {
        //imgui regard the button as the same one which has the same label including imageButton
        auto* mat = scene_view->scene_model->material_lib[i];
        if (!mat)
        {
          char s[32] = { 0 };
          sprintf(s, "empty \nslot\n %d", i);
          if (ImGui::Button(s, ImVec2(30, 30)))
          {
            printf("select %d\n", i);
            mat_id_ref = -1;
            ImGui::CloseCurrentPopup();
          }
        }
        else
        {
          if (mat->material_data->type != MaterialMetaData::E_MM_MIX)
          if (ImGui::ImageButton(scene_view->_d3d11_renderer->textures[mat->_ui_show_tex].srv, ImVec2(30, 30)))
          {
            mat_id_ref = i;
            ImGui::CloseCurrentPopup();
          }
        }

        if ((i+1)%4!=0)
          ImGui::SameLine();


      }
      ImGui::EndPopup();

    }

    ImGui::PopStyleVar();

    LibMaterial* mat_ref = nullptr;
    if (mat_id_ref>=0)
    {
      mat_ref = scene_view->scene_model->material_lib[mat_id_ref];
    }
    if (mat_ref)
    { 
      ImGui::Image(scene_view->_d3d11_renderer->textures[mat_ref->_ui_show_tex].srv, ImVec2(100,100));
    }

    if(ImGui::Button("Select this.."))
    {
      scene_view->selected_material_id = mat_id_ref;
    }

    return retVal;
  }

  bool startBrowseDialogNextFrame = false;

public:
  static MaterialRefNode* Create(const ImVec2& pos, ImGui::NodeGraphEditor* nge)
  {
    MaterialRefNode* node = (MaterialRefNode*)ImGui::MemAlloc(sizeof(MaterialRefNode)); new(node)MaterialRefNode();
    if (!node->setup(nge, pos, nullptr, "material link", NodeType::MatrialRef))
    {
      return nullptr;
    }

    char aaa[111];
    ImGui::FieldInfo* f = NULL; 
    //f = &(node->fields.addFieldTextEditAndBrowseButton(aaa, 111, "caonima", "gogogo here we are!", ImGuiInputTextFlags_EnterReturnsTrue, (void*)node));
    //node->fields.addField((f32*)&node->mat_id_ref, 1, "Color", nullptr, 3, 0, 1);

    return node;
  }

  void onEditField(ImGui::FieldInfo& /*f*/, int widgetIndex) {
    //fprintf(stderr,"TextureNode::onEditField(\"%s\",%i);\n",f.label,widgetIndex);
    if (widgetIndex == 1)         startBrowseDialogNextFrame = true;  // browsing button pressed
    //else if (widgetIndex == 0)    processPath(imagePath);             // text edited (= "return" pressed in out case)
  }

  static void StaticEditFieldCallback(ImGui::FieldInfo& f, int widgetIndex) {
    reinterpret_cast<MaterialRefNode*>(f.userData)->onEditField(f, widgetIndex);
  }

};




ImGui::Node* nodeFactory(int nodeType, const ImVec2& pos) 
{
  switch (nodeType)
  {
  case (int)NodeType::Constant:
    return ConstantNode::Create(pos,nullptr);
    break;
  case (int)NodeType::ConstantColor:
    return ConstantColorNode::Create(pos, nullptr);

    break;
  case (int)NodeType::MatrialRef:
    return MaterialRefNode::Create(pos, nullptr);

    break;
  case (int)NodeType::Mix:
    return MixNode::Create(pos, 0);
    break;
  case (int)NodeType::MixMask:
    return MixMaskNode::Create(pos, 0);
    break;
  default:
    break;
  }

  return nullptr;
}

void linkCallback(const ImGui::NodeLink& link, ImGui::NodeGraphEditor::LinkState state, ImGui::NodeGraphEditor& editor) 
{
  switch (state)
  {
  case ImGui::NodeGraphEditor::LS_ADDED:
  {
    if (link.OutputNode->getType() == (int)NodeType::Mix)
    {
      if ((link.InputNode->getType() != (int)NodeType::Mix) && (link.InputNode->getType() != (int)NodeType::MatrialRef) &&
        (link.InputNode->getType() != (int)NodeType::MixMask))
      {
        printf("Cant link these types!\n");
        editor.removeLink(link.InputNode, link.InputSlot, link.OutputNode, link.OutputSlot);
      }
    }
  }
    break;
  case ImGui::NodeGraphEditor::LS_DELETED:
  {

  }
    break;
  default:
    break;
  }
}

void nodeCallback(ImGui::Node*& node, ImGui::NodeGraphEditor::NodeState state, ImGui::NodeGraphEditor& editor)
{
  switch (state)
  {
  case ImGui::NodeGraphEditor::NS_ADDED:
  {
    printf("Add a node\n");
    ((NodeBase*)node)->nge = &editor;
  }
    break;
  case ImGui::NodeGraphEditor::NS_DELETED:
  {
    printf("delete a node\n");
  }
    break;
  case ImGui::NodeGraphEditor::NS_EDITED:
  {
    printf("node edited\n");
  }
    break;
  default:
    break;
  }
}

void EditorSceneView::on_dock_gui()
{
  //node editor
  /*
  if (node_editor.isInited())
  {
    node_editor.registerNodeTypes(NodeTypeStr, 6, nodeFactory, nullptr, -1);
    node_editor.setNodeCallback(nodeCallback);
    node_editor.setLinkCallback(linkCallback);


    node_editor.show_style_editor = false;
    node_editor.show_load_save_buttons = true;
    node_editor.show_connection_names = false;
    node_editor.show_left_pane = true;
    node_editor.user_ptr = this;
  }
  */

  ImGui::SetNextWindowPos(ImVec2(ww*0.8, 
    ImGui::GetItemsLineHeightWithSpacing()
    ));
  ImGui::SetNextWindowSize(ImVec2(ww*0.2,
    wh - ImGui::GetTextLineHeightWithSpacing()
    ));

  ImGui::Begin("Tools", 0,   ImGuiWindowFlags_NoResize);
  right_window.updateAndDraw(ImGui::GetContentRegionAvail());
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(0,
    ImGui::GetItemsLineHeightWithSpacing()
    ));
  ImGui::SetNextWindowSize(ImVec2(ww*0.8,
    wh - ImGui::GetTextLineHeightWithSpacing()
    ));

  ImGui::Begin("Window", &open_left_editors,  ImGuiWindowFlags_NoResize);
  dockspace.updateAndDraw(ImGui::GetContentRegionAvail());
  ImGui::End();

  /*
  ImGui::Begin("s");
  {
    static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::ROTATE);
    static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::WORLD);
    if (ImGui::IsKeyPressed(90))
      mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
    if (ImGui::IsKeyPressed(69))
      mCurrentGizmoOperation = ImGuizmo::ROTATE;
    if (ImGui::IsKeyPressed(82)) // r Key
      mCurrentGizmoOperation = ImGuizmo::SCALE;
    if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
      mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
      mCurrentGizmoOperation = ImGuizmo::ROTATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
      mCurrentGizmoOperation = ImGuizmo::SCALE;
    float matrixTranslation[3], matrixRotation[3], matrixScale[3];
    ImGuizmo::DecomposeMatrixToComponents(matrix.m16, matrixTranslation, matrixRotation, matrixScale);
    ImGui::InputFloat3("Tr", matrixTranslation, 3);
    ImGui::InputFloat3("Rt", matrixRotation, 3);
    ImGui::InputFloat3("Sc", matrixScale, 3);
    ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix.m16);

    if (mCurrentGizmoOperation != ImGuizmo::SCALE)
    {
      if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
        mCurrentGizmoMode = ImGuizmo::LOCAL;
      ImGui::SameLine();
      if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
        mCurrentGizmoMode = ImGuizmo::WORLD;
    }
    static bool useSnap(false);
    if (ImGui::IsKeyPressed(83))
      useSnap = !useSnap;
    ImGui::Checkbox("", &useSnap);
    ImGui::SameLine();
    vec_t snap;
    switch (mCurrentGizmoOperation)
    {
    case ImGuizmo::TRANSLATE:
      snap = config.mSnapTranslation;
      ImGui::InputFloat3("Snap", &snap.x);
      break;
    case ImGuizmo::ROTATE:
      snap = config.mSnapRotation;
      ImGui::InputFloat("Angle Snap", &snap.x);
      break;
    case ImGuizmo::SCALE:
      snap = config.mSnapScale;
      ImGui::InputFloat("Scale Snap", &snap.x);
      break;
    }
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    ImGuizmo::Manipulate(camera.mView.m16, camera.mProjection.m16, mCurrentGizmoOperation, mCurrentGizmoMode, matrix.m16, NULL, useSnap ? &snap.x : NULL);
  }
  ImGui::End();
  */

}

EditorSceneView::~EditorSceneView()
{
	delete scene_camera;
	delete scene_model;
	delete pre_renderer;
}

bool GlassMaterialMetaData::on_gui(Direct3D11Renderer* renderer, PreViewRenderer* pre_r, int texid)
{
  on_change_color_gui();
  bool change = false;
  change |= ImGui::SliderFloat3("Kr", (float*)&kr, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&kr));
  select_a_image(kr_t, "Image Flie\0*.png;*.jpg\0\0");

  change |= ImGui::SliderFloat3("Kt", (float*)&kt, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&kt));
  select_a_image(kt_t, "Image Flie\0*.png;*.jpg\0\0");

  change |= ImGui::SliderFloat2("u,v roughness", (float*)&uv_roughness, 0.f, 1.f);
  change |= ImGui::InputFloat("eta", &eta);
  change |= ImGui::Checkbox("remap roughness", &remap_roghness);
  ImGui::LabelText("", "Remapping roughness makes 0~1 to be linear roughness.");
  if (change)
    this->change = change;
  if (texid == -1)
    return change;

  ImGui::Spacing();

  ImGui::SliderInt("SPP", &pre_spp, 1, 128);
  preview(pre_r, texid, renderer);



  return change;
}



#include "../material.h"

OLPBRMaterial* GlassMaterialMetaData::create_material()
{
  OLPBRTexture<RBColorf>* kr1;
  OLPBRTexture<RBColorf>* kt1;


  if (kr_t=="")
    kr1 = (new OLPBRConstantTexture<RBColorf>(kr));
  else
    kr1 = new OLPBRImageTexture(kr_t.data());
  if (kt_t == "")
    kt1 = new OLPBRConstantTexture<RBColorf>(kt);
  else
    kt1 = new OLPBRImageTexture(kt_t.data());
  auto ur1 = new OLPBRConstantTexture<f32>(uv_roughness.x);
  auto vr1 = new OLPBRConstantTexture<f32>(uv_roughness.y);
  auto eata1 = new OLPBRConstantTexture<f32>(eta);

  return new OLPBRGlassMaterial(kr1, kt1, ur1, vr1, eata1, remap_roghness);

}

bool MetalMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid)
{
  on_change_color_gui();
  bool change = false;
  change |= ImGui::InputFloat3("Eta 0~1k", (float*)&eta);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&eta));
  change |= ImGui::SliderFloat3("K", (float*)&k, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&k));
  select_a_image(k_t, "Image Flie\0*.png;*.jpg\0\0");
  change |= ImGui::SliderFloat2("u,v roughness", (float*)&uv_roughness, 0.f, 1.f);
  select_a_image(roughness_t, "Image Flie\0*.png;*.jpg\0\0");

  if (change)
    this->change = change;
  if (texid == -1)
    return change;

  ImGui::Spacing();
  static bool c = true;
  if (ImGui::Button("check"))
  {
    auto* m = create_material();
    c = m->check_convergence();
  }
  ImGui::Checkbox("check", &c);
  ImGui::SliderInt("SPP", &pre_spp, 1, 128);

  preview(pre_r, texid, renderer);



  return change;

}

class OLPBRMaterial* MetalMaterialMetaData::create_material()
{
  OLPBRTexture<RBColorf>* k1;

  auto eta1 = new OLPBRConstantTexture<RBColorf>(eta);
  if (k_t == "")
    k1 = new OLPBRConstantTexture<RBColorf>(k);
  else
    k1 = new OLPBRImageTexture(k_t.data());

  if (roughness_t=="")
  { 
  auto ur1 = new OLPBRConstantTexture<f32>(uv_roughness.x);
  auto vr1 = new OLPBRConstantTexture<f32>(uv_roughness.y);


  return new OLPBRMetalMaterial(eta1, k1, nullptr, ur1, vr1, true);
  }
  else
  {
    return new OLPBRMetalMaterial(eta1, k1, new OLPBRImageTextureRounghness(roughness_t.data()), nullptr, nullptr, true);
  }
}

LibMaterial::~LibMaterial()
{
	delete offline_material;
	delete material_data;
}

LibMaterial::LibMaterial(Direct3D11Renderer* _d3d11_renderer, PreViewRenderer* pre_render)
{
  D3D11_TEXTURE2D_DESC desc;
  rb_memzero(&desc, sizeof(desc));
  desc.Width = pre_render->cam->film_->xres;
  desc.Height = pre_render->cam->film_->yres;
  desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.SampleDesc.Quality = 0;
  desc.Usage = D3D11_USAGE_DYNAMIC;
  desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
  desc.ArraySize = 1;
  desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
  HRESULT res = _d3d11_renderer->device->CreateTexture2D(&desc, 0, &tex);
  if (res != S_OK)
  {
    g_logger->debug(WIP_ERROR, "Couldn't create dynamic texture!");
  }

  _ui_show_tex = _d3d11_renderer->addTexture(tex, 0);
  offline_material = nullptr;
  material_data = nullptr;

  //this->_d3d11_renderer = _d3d11_renderer;
}


void MaterialMetaData::preview(PreViewRenderer* pre_r, int texid, Direct3D11Renderer* renderer)
{
  if (ImGui::Button("Preview"))
  {

    pre_r->render(this);
    unsigned char* dao3 = pre_r->cam->film_->output_pix();
    RBColor32* dao = new RBColor32[pre_r->cam->film_->xres* pre_r->cam->film_->yres];
    for (int i = 0; i < pre_r->cam->film_->xres* pre_r->cam->film_->yres; ++i)
    {
      int y = i / pre_r->cam->film_->xres;
      int x = i % pre_r->cam->film_->xres;
      if (x > pre_r->cam->film_->xres*0.8&&x<pre_r->cam->film_->xres*0.95&&y>pre_r->cam->film_->yres*0.8&&y < pre_r->cam->film_->yres*0.95)
      {
        dao[i].r = reg_color.r * 255;
        dao[i].g = reg_color.g * 255;
        dao[i].b = reg_color.b * 255;
        dao[i].a = 255;

      }
      else
      {
        dao[i].r = dao3[i * 3 + 2];
        dao[i].g = dao3[i * 3 + 1];
        dao[i].b = dao3[i * 3];
        dao[i].a = 255;
      }
    }

    D3D11_MAPPED_SUBRESOURCE resource;
    renderer->context->Map(renderer->textures[texid].texture,
      0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
    u8* da = reinterpret_cast<u8*>(resource.pData);
    u8* dao_c = (u8*)dao;
    //GPU有自己的内存布局，不要整块拷贝
    for (int a = 0; a < pre_r->cam->film_->yres; ++a)
    {
      memcpy(da, dao_c, sizeof(RBColor32)*pre_r->cam->film_->xres);
      da += resource.RowPitch;
      dao_c += pre_r->cam->film_->xres*sizeof(RBColor32);
    }
    renderer->context->Unmap(renderer->textures[texid].texture, 0);
    delete[] dao;

  }
  //need shader_resource_view
  ImGui::Image(renderer->textures[texid].srv, ImVec2(pre_r->cam->film_->xres, pre_r->cam->film_->yres));
}

void MaterialMetaData::on_change_color_gui()
{
  ImGui::SliderFloat3("Color", (f32*)&reg_color, 0, 1);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&reg_color));
}

void MaterialMetaData::select_a_image(std::string& name, const char* filter)
{
  ImGui::PushID(&name);
  if (ImGui::Button("+"))
  {
    RBPathTool::open_file(name, filter);

  }
  ImGui::SameLine();
  if (ImGui::Button("-"))
  {
    name = "";
  }
  ImGui::SameLine();
  if (ImGui::Button("clear"))
  {
    if (name != "")
    {
      
    }
  }
  ImGui::Text(name.data());
  ImGui::PopID();
}

bool MatteMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kd", (float*)&kd, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kd));
  select_a_image(kd_t, "Image Flie\0*.png;*.jpg\0\0");
	change |= ImGui::SliderFloat("Sigma", (float*)&sigma, 0.f, 360.f);

	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

  static bool c = true;
  if (ImGui::Button("check"))
  {
    auto* m = create_material();
    c = m->check_convergence();
  }
  ImGui::Checkbox("check", &c);

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* MatteMaterialMetaData::create_material()
{
  OLPBRTexture<RBColorf>* kdt;
  if (kd_t != "")
  {
    kdt = new OLPBRImageTexture(kd_t.data());
  }
  else
  { 
    kdt = new OLPBRConstantTexture<RBColorf>(kd);
  }
  auto sigmat = new OLPBRConstantTexture<f32>(sigma);
  return new OLPBRMatte(kdt, sigmat, nullptr);
}


bool BlankMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
  on_change_color_gui();
  bool change = false;


  ImGui::SliderInt("SPP", &pre_spp, 1, 128);
  preview(pre_r, texid, renderer);



  return change;
}

OLPBRMaterial* BlankMaterialMetaData::create_material()
{
  return new BlankMaterial();
}


bool SimpleGlassMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kr", (float*)&kr, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kr));
  select_a_image(kr_t, "Image Flie\0*.png;*.jpg\0\0");

	change |= ImGui::SliderFloat3("Kt", (float*)&kt, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kt));
  select_a_image(kt_t, "Image Flie\0*.png;*.jpg\0\0");

	change |= ImGui::InputFloat("eta", &eta);
	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* SimpleGlassMaterialMetaData::create_material()
{
  OLPBRTexture<RBColorf>* krt;
  OLPBRTexture<RBColorf>* ktt;

  if (kr_t == "")
    krt = new OLPBRConstantTexture<RBColorf>(kr);
  else
    krt = new OLPBRImageTexture(kr_t.data());
  if (kt_t == "")
    ktt = new OLPBRConstantTexture<RBColorf>(kt);
  else
    ktt = new OLPBRImageTexture(kt_t.data());
  auto etat = new OLPBRConstantTexture<f32>(eta);
  return new SimpleGlassMaterial(krt, ktt, etat);
}

bool FresnelBlendMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Rd", (float*)&rd, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&rd));


	change |= ImGui::SliderFloat3("Rs", (float*)&rs, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&rs));
	change |= ImGui::SliderFloat("roughness", (float*)&roughness, 0.f, 1.f);
	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* FresnelBlendMaterialMetaData::create_material()
{
  auto rt = new OLPBRConstantTexture<f32>(roughness);
  return new OLPBRFresnelBlendMaterial(rd, rs, rt,true);
}

bool PlasticMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kd", (float*)&kd, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kd));
  select_a_image(kd_t, "Image Flie\0*.png;*.jpg\0\0");


	change |= ImGui::SliderFloat3("Ks", (float*)&ks, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&ks));
  select_a_image(ks_t, "Image Flie\0*.png;*.jpg\0\0");


	change |= ImGui::SliderFloat("roughness", (float*)&roughness, 0.f, 1.f);
  select_a_image(roughness_t, "Image Flie\0*.png;*.jpg\0\0");

	if (change)
		this->change = change;
	if (texid == -1)
		return change;


	ImGui::Spacing();

  static bool c = true;
  if (ImGui::Button("check"))
  {
    auto* m = create_material();
    c = m->check_convergence();
    delete m;
  }
  ImGui::Checkbox("check", &c);

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);

	return change;
}

OLPBRMaterial* PlasticMaterialMetaData::create_material()
{
  OLPBRTexture<RBColorf>* kdt;
  OLPBRTexture<RBColorf>* kst;
  OLPBRTexture<f32>* rt;

  if (kd_t == "")
    kdt = new OLPBRConstantTexture<RBColorf>(kd);
  else
    kdt = new OLPBRImageTexture(kd_t.data());
  if (ks_t == "")
    kst = new OLPBRConstantTexture<RBColorf>(ks);
  else
    kst = new OLPBRImageTexture(ks_t.data());
  if (roughness_t == "")
    rt = new OLPBRConstantTexture<f32>(roughness);
  else
    rt = new OLPBRImageTextureRounghness(roughness_t.data());

  return new OLPBRPlasticMaterial(kdt, kst, rt, true);

}

bool MirrorMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kr", (float*)&kr, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kr));
	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* MirrorMaterialMetaData::create_material()
{
  auto krt = new OLPBRConstantTexture<RBColorf>(kr);
  return new OLPBRMirrorMaterial(krt);
}

bool HairMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kd", (float*)&kd, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kd));
  select_a_image(kd_t, "Image Flie\0*.png;*.jpg\0\0");

	change |= ImGui::SliderFloat3("Ks", (float*)&ks, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&ks));
  select_a_image(ks_t, "Image Flie\0*.png;*.jpg\0\0");

  change |= ImGui::SliderFloat("roughness", (float*)&roughness, 0.f, 1.f);
	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* HairMaterialMetaData::create_material()
{
  OLPBRTexture<RBColorf>* kdt;
  OLPBRTexture<RBColorf>* kst;

  if (kd_t == "")
    kdt = new OLPBRConstantTexture<RBColorf>(kd);
  else
    kdt = new OLPBRImageTexture(kd_t.data());
  if (ks_t == "")
    kst = new OLPBRConstantTexture<RBColorf>(ks);
  else
    kst = new OLPBRImageTexture(ks_t.data());

  auto rt = new OLPBRConstantTexture<f32>(roughness);
  return new OLPBRHairMaterial(kdt, kst, rt);

}

bool SubstrateMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kd", (float*)&kd, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kd));
	change |= ImGui::SliderFloat3("Ks", (float*)&ks, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&ks));
	change |= ImGui::SliderFloat("nu", (float*)&nu, 0.f, 1.f);
  change |= ImGui::SliderFloat("nv", (float*)&nv, 0.f, 1.f);
	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* SubstrateMaterialMetaData::create_material()
{
  auto kdt = new OLPBRConstantTexture<RBColorf>(kd);
  auto kst = new OLPBRConstantTexture<RBColorf>(ks);
  auto nut = new OLPBRConstantTexture<f32>(nu);
  auto nvt = new OLPBRConstantTexture<f32>(nv);
  return new OLPBRSubstrateMaterial(kdt, kst, nut, nvt, true);

}

bool TranslucentMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kd", (float*)&kd, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kd));
	change |= ImGui::SliderFloat3("Ks", (float*)&ks, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&ks));

  change |= ImGui::SliderFloat3("relectance", (float*)&refl, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&refl));
  change |= ImGui::SliderFloat3("transmission", (float*)&trans, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&trans));

	change |= ImGui::SliderFloat("roughness", (float*)&roughness, 0.f, 1.f);
	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* TranslucentMaterialMetaData::create_material()
{
  auto kdt = new OLPBRConstantTexture<RBColorf>(kd);
  auto kst = new OLPBRConstantTexture<RBColorf>(ks);
  auto rt = new OLPBRConstantTexture<f32>(roughness);
  auto reflt = new OLPBRConstantTexture<RBColorf>(refl);
  auto transt = new OLPBRConstantTexture<RBColorf>(trans);
  return new OLPBRTranslucentMaterial(kdt, kst, rt, reflt, transt, true);
}

bool UberMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
	change |= ImGui::SliderFloat3("Kr", (float*)&kr, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kr));
  select_a_image(kr_t, "Image Flie\0*.png;*.jpg\0\0");

	change |= ImGui::SliderFloat3("Kt", (float*)&kt, 0.f, 1.f);
	ImGui::SameLine();
	ImGui::ColorButton(*((ImVec4*)&kt));
  select_a_image(kt_t, "Image Flie\0*.png;*.jpg\0\0");


  change |= ImGui::SliderFloat3("Kd", (float*)&kd, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&kd));
  select_a_image(kd_t, "Image Flie\0*.png;*.jpg\0\0");


  change |= ImGui::SliderFloat3("Ks", (float*)&ks, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&ks));
  select_a_image(ks_t, "Image Flie\0*.png;*.jpg\0\0");


  change |= ImGui::SliderFloat3("opacity", (float*)&opacity, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&opacity));
  select_a_image(opacity_t, "Image Flie\0*.png;*.jpg\0\0");


	change |= ImGui::SliderFloat("roughness", (float*)&roughness, 0.f, 1.f);
  select_a_image(r_t, "Image Flie\0*.png;*.jpg\0\0");


  change |= ImGui::SliderFloat("ru", (float*)&ru, 0.f, 1.f);
  change |= ImGui::SliderFloat("rv", (float*)&rv, 0.f, 1.f);
  change |= ImGui::SliderFloat("eta", (float*)&eta, 0.f, 3.f);

	if (change)
		this->change = change;
	if (texid == -1)
		return change;

	ImGui::Spacing();

	ImGui::SliderInt("SPP", &pre_spp, 1, 128);
	preview(pre_r, texid, renderer);



	return change;
}

OLPBRMaterial* UberMaterialMetaData::create_material()
{

  OLPBRTexture<RBColorf>* kdt;
  OLPBRTexture<RBColorf>* kst;
  OLPBRTexture<RBColorf>* krt;
  OLPBRTexture<RBColorf>* ktt;
  OLPBRTexture<RBColorf>* opt;
  OLPBRTexture<f32>* rt;
  OLPBRTexture<f32>* rut = nullptr;
  OLPBRTexture<f32>* rvt = nullptr;


  if (kd_t == "")
    kdt = new OLPBRConstantTexture<RBColorf>(kd);
  else
    kdt = new OLPBRImageTexture(kd_t.data());
  if (ks_t == "")
    kst = new OLPBRConstantTexture<RBColorf>(ks);
  else
    kst = new OLPBRImageTexture(ks_t.data());
  if (r_t == "")
  {
    if (roughness != 0)
      rt = new OLPBRConstantTexture<f32>(roughness);
    else
      rt = nullptr;
  }
  else
    rt = new OLPBRImageTextureRounghness(r_t.data());
  if (ru!=0&&rv!=0)
  { 
    rut = new OLPBRConstantTexture<f32>(ru);
    rvt = new OLPBRConstantTexture<f32>(rv);
    rt = nullptr;
  }
  else
  {
    if (!rt)
      rt = new OLPBRConstantTexture<f32>(1.f);
  }
  if (kr_t == "")
    krt = new OLPBRConstantTexture<RBColorf>(kr);
  else
    krt = new OLPBRImageTexture(kr_t.data());
  if (kt_t == "")
    ktt = new OLPBRConstantTexture<RBColorf>(kt);
  else
    ktt = new OLPBRImageTexture(kt_t.data());
  if (opacity_t == "")
    opt = new OLPBRConstantTexture<RBColorf>(opacity);
  else
    opt = new OLPBRImageTexture(opacity_t.data());

  auto etat = new OLPBRConstantTexture<f32>(eta);

  return new OLPBRUberMaterial(kdt, kst, krt, ktt, rt, rut, rvt, opt, etat, true);

}

MixMaterialMetaData::MixMaterialMetaData(EditorSceneView* editor_view,LibMaterial* mat)
{
  type = E_MM_MIX;
  param = new node_editor_param();
  param->editor_view = editor_view;
  param->meta_data = mat;
  if (node_editor.isInited())
  {
    
    node_editor.registerNodeTypes(NodeTypeStr, 6, nodeFactory, nullptr, -1);
    node_editor.setNodeCallback(nodeCallback);
    node_editor.setLinkCallback(linkCallback);

    node_editor.show_style_editor = false;
    node_editor.show_load_save_buttons = true;
    node_editor.show_connection_names = false;
    node_editor.show_left_pane = true;
    node_editor.user_ptr = param;
  }
}

bool MixMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
	on_change_color_gui();
	bool change = false;
  //node_editor.
  node_editor_param *node_param = (node_editor_param*)node_editor.user_ptr;
  EditorSceneView* scene_view = node_param->editor_view;
  LibMaterial* meta_data = node_param->meta_data;

  if (scene_view->cur_node_editor != &node_editor)
  {
    scene_view->cur_node_editor = &node_editor;
  }
  if (texid!=-1)
    ImGui::Image(renderer->textures[texid].srv, ImVec2(pre_r->cam->film_->xres, pre_r->cam->film_->yres));
	return change;
}

OLPBRMaterial* MixMaterialMetaData::create_material()
{
  return nullptr;
}

void ModelObjectInstance::on_gui(EditorSceneView* editor_view)
{

  //ImGui::Begin("Object Properties");

  RBVector3 pos = get_pos();
  RBVector3 sca = get_scale();
  RBVector3 rot = get_rot();

  ImGui::InputFloat3("Position", (f32*)&pos);
  ImGui::InputFloat3("Rotation", (f32*)&rot);
  ImGui::InputFloat3("Scale", (f32*)&sca);
  
  translate_to(pos);
  rotate_to(rot);
  scale_to(sca);

  ImGui::Separator();
  
  if (-1 != material_id&&editor_view->scene_model->material_lib[material_id] && editor_view->scene_model->material_lib[material_id]->_ui_show_tex != -1)
  {
    ImGui::Image(editor_view->_d3d11_renderer->textures[editor_view->scene_model->material_lib[material_id]->_ui_show_tex].srv, ImVec2(100, 100));
  }
  if (ImGui::Button("Pick Material"))
  {
    material_id = editor_view->selected_material_id;
  }
  if (ImGui::Button("Drop Material"))
  {
    material_id = -1;
  }
  
  ImGui::Checkbox("Light", &is_area_light);
  if (is_area_light)
  {
    ImGui::InputFloat3("Le", (f32*)&le);
    ImGui::InputInt("Sample", &nsamples);
  }


#if 0
  static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::ROTATE);
  static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
  if (ImGui::IsKeyPressed(90))
    mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
  if (ImGui::IsKeyPressed(69))
    mCurrentGizmoOperation = ImGuizmo::ROTATE;
  if (ImGui::IsKeyPressed(82)) // r Key
    mCurrentGizmoOperation = ImGuizmo::SCALE;
  if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
    mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
  ImGui::SameLine();
  if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
    mCurrentGizmoOperation = ImGuizmo::ROTATE;
  ImGui::SameLine();
  if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
    mCurrentGizmoOperation = ImGuizmo::SCALE;
  /*
  float matrixTranslation[3], matrixRotation[3], matrixScale[3];
  memcpy(matrixTranslation, &pos, 3 * sizeof(f32));
  memcpy(matrixRotation, &rot, 3 * sizeof(f32));
  memcpy(matrixScale, &sca, 3 * sizeof(f32));


  //ImGuizmo::DecomposeMatrixToComponents(matrix.m16, matrixTranslation, matrixRotation, matrixScale);
  ImGui::InputFloat3("Tr", matrixTranslation, 3);
  ImGui::InputFloat3("Rt", matrixRotation, 3);
  ImGui::InputFloat3("Sc", matrixScale, 3);

  translate_to(pos);
  rotate_to(rot);
  scale_to(sca);
  
  ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix.m16);
  */

  if (mCurrentGizmoOperation != ImGuizmo::SCALE)
  {
    if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
      mCurrentGizmoMode = ImGuizmo::LOCAL;
    ImGui::SameLine();
    if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
      mCurrentGizmoMode = ImGuizmo::WORLD;
  }
  static bool useSnap(false);
  if (ImGui::IsKeyPressed(83))
    useSnap = !useSnap;
  ImGui::Checkbox("", &useSnap);
  ImGui::SameLine();
  RBVector2 snap;
  switch (mCurrentGizmoOperation)
  {
  case ImGuizmo::TRANSLATE:
    snap = RBVector2(1,1);// config.mSnapTranslation;
    ImGui::InputFloat3("Snap", &snap.x);
    break;
  case ImGuizmo::ROTATE:
    snap = RBVector2(1, 1);// config.mSnapRotation;
    ImGui::InputFloat("Angle Snap", &snap.x);
    break;
  case ImGuizmo::SCALE:
    snap = RBVector2(1, 1);
    ImGui::InputFloat("Scale Snap", &snap.x);
    break;
  }
  ImGuiIO& io = ImGui::GetIO();
  ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
  RBMatrix view;
  editor_view->scene_camera->get_view_matrix(view);
  RBMatrix proj;
  editor_view->scene_camera->get_perspective_matrix(proj);

  RBMatrix mvp = localtox*view*proj;

  ImGuizmo::Manipulate((f32*)&view, (f32*)&proj, mCurrentGizmoOperation, mCurrentGizmoMode, (f32*)&localtox, NULL, useSnap ? &snap.x : NULL);
  //ImGui::End();
#endif

}

void PointLightObjectInstance::on_gui(class EditorSceneView* editor_view)
{
  ImGui::SliderFloat3("Le", (f32*)&intensity, 0.f, 10.f);
}

void MicrofacetReflectionBrdf::on_gui()
{
  ImGui::SliderFloat3("eta", (f32*)&eta,0.f,2.f);
  ImGui::SliderFloat3("K 1~10 ", (f32*)&k, 0.f, 10.f);
  ImGui::SliderFloat2("uv_roughness", (f32*)&uv_roughness, 0.f, 1.f);
  static int spp = 10000;
  static bool checkc = true;
  ImGui::InputInt("spp", &spp);
  f32 ur, vr;
  if (ImGui::Button("Check"))
  {
    ur = OLPBRTrowbridgeReitzDistribution::roughness2alpha(uv_roughness.x);
    vr = OLPBRTrowbridgeReitzDistribution::roughness2alpha(uv_roughness.y);
    OLPBRFresnel* fr = new OLPBRFresnelConductor(1.f, eta, k);
    OLPBRMicrofacetDistribution* d = new OLPBRTrowbridgeReitzDistribution(ur, vr);
    OLPBRMicrofacetReflection* brdf = new OLPBRMicrofacetReflection(1.f, d, fr);

    checkc = check(brdf,spp);

    delete brdf;
    delete d;
    delete fr;
  }
  ImGui::Checkbox("check?", &checkc);
}

bool EditorBrdf::check(class OLPBRBXDF* bxdf, int spp /*= 1000*/)
{
  {
    auto* sender = OLNetDebugger::get();

    RBVector3 wi;
    RBVector3 wo;
    f32 pdf=0.f;
    bool ret = true;
    RBColorf l = RBColorf::black;
    for (f32 j = 0; j < 2 * PI; j += 2 * PI*0.01)
      for (f32 k = 0; k < 2 * PI; k += 2 * PI*0.01)
      {
        wo = RBVector3(cos(k)*cos(j), cos(k)*sin(j), sin(k));
        sender->draw_line(RBVector3::zero_vector, wo);
        for (int i = 0; i < spp; ++i)
        {
          BxdfType bxdftp;
          RBColorf c = bxdf->sample_f(wo, wi, pdf, &bxdftp);

          if (!RBMath::is_nearly_zero(pdf,0.001f))
          {
            RBColorf ll = c / pdf*RBVector3::abs_dot(wi.get_normalized(), RBVector3(0, 0, 1));
            if (ll.r > 1e18)
            {
              RBColorf cc = bxdf->f(wo, wi);
              cc.r += 0;
            }
            l += ll;
          }
        }
        //l.out();
        l /= spp;

        if (l.r <= 1.01 && l.g <= 1.01 && l.b <= 1.01)
        {
          ret &= true;
        }
        else
        {
          l.out();
          return false;
        }
      }
  }
  return true;
}

bool DisneyMaterialMetaData::on_gui(Direct3D11Renderer* renderer /*= nullptr*/, PreViewRenderer* pre_r /*= nullptr*/, int texid /*= -1*/)
{
  if (brdf_uv_tex_id==-1)
  {
    D3D11_TEXTURE2D_DESC desc;
    rb_memzero(&desc, sizeof(desc));
    desc.Width = 200;
    desc.Height = 200;
    desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;
    desc.Usage = D3D11_USAGE_DYNAMIC;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.ArraySize = 1;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    HRESULT res = renderer->device->CreateTexture2D(&desc, 0, &brdf_uv_tex);
    if (res != S_OK)
    {
      g_logger->debug(WIP_ERROR, "Couldn't create dynamic texture!");
    }

    brdf_uv_tex_id = renderer->addTexture(brdf_uv_tex, 0);
  }

  on_change_color_gui();
  bool change = false;
  change |= ImGui::SliderFloat3("BaseColor", (float*)&base_color, 0.f, 1.f);
  ImGui::SameLine();
  ImGui::ColorButton(*((ImVec4*)&base_color));
  select_a_image(base_colort, "Image Flie\0*.png;*.jpg\0\0");

  char* p[] = { "subsurface", "metallic", "specular", "specular_tint", "roughness", "anisotropic", "sheen", "sheen_tint",
    "clearcoat", "clearcoat_gloss" };

  for (int i = 0; i < 10;++i)
  { 
    f32 tm = 1.f;
    if (i == 6)
      tm = 10.f;
    change |= ImGui::SliderFloat(p[i], (float*)&param[i], 0.f, tm);
    select_a_image(paramt[i], "Image Flie\0*.png;*.jpg\0\0");
  }

  if (change)
    this->change = change;
  if (texid == -1)
    return change;

  if (alpha)
  {
    select_a_image(alpha_t, "Image Flie\0*.png;*.jpg\0\0");
  }

  ImGui::Spacing();

  ImGui::SliderInt("SPP", &pre_spp, 1, 128);
  preview(pre_r, texid, renderer);

  ImGui::Separator();

  ImGui::SliderInt("Sample SPP", &brdf_sample_spp, 1, 10000);
  ImGui::SliderAngle("u-", &u, 0, 360);
  ImGui::SliderAngle("v|", &v, 1, 90);
  ImGui::SliderFloat("brdf scale",&brdf_scale,0,5);

  //单独draw pdf
  if (ImGui::Button("Draw Sample(up sphere)"))
  {
    do
    { 
    SrWriteTexture valmap(200,200);
    
    OLPBRDisneyMaterial* m = (OLPBRDisneyMaterial*)create_material();
    OLPBRBXDF* bxdf = m->get_bxdf();
    RBVector3 wo = RBVector3(cos(v)*cos(u), cos(v)*sin(u), sin(v));
    RBVector3 wi = RBVector3::zero_vector;
    f32 pdf = 0;
    for (int i = 0; i < 1000000; ++i)
    {
      BxdfType bxdftp;
      RBColorf c = bxdf->sample_f(wo, wi, pdf, &bxdftp);

      CHECK(c.is_avaliable());

      ft thetay = spherical_theta(wi), phix = spherical_phi(wi);
      if (pdf!=0)
      {
        RBColorf ll = c *brdf_scale;
        //printf("%f\n", brdf_scale);
        //ll.out();
        //draw brdf
        int x = 200 * phix*INV_2PI;
        int y = 100 * thetay * 2 * INV_PI;
        //CHECK(y<=100);
        valmap.set_pix(x, y, ll);
        //draw sample point
        

        RBColorf lll = pdf*brdf_scale;
        valmap.set_pix(x, y+100, lll);
      }
      /*
      else
      { 
      
      int xs = 200 * phix*INV_2PI;
      int ys = 100 * thetay * 2 * INV_PI;
      valmap.set_pix(xs, ys, RBColorf::green);
      xs = 200 * phix*INV_2PI;
      ys = 100 * thetay * 2 * INV_PI+100;
      valmap.set_pix(xs, ys, RBColorf::green);
      }
      */
    }

    
    for (int i = 0; i < brdf_sample_spp; ++i)
    {
      BxdfType bxdftp;
      RBColorf c = bxdf->sample_f(wo, wi, pdf, &bxdftp);

      CHECK(c.is_avaliable());

      ft thetay = spherical_theta(wi), phix = spherical_phi(wi);

      if (pdf!=0)
      {

        int xs = 200 * phix*INV_2PI;
        int ys = 100 * thetay * 2 * INV_PI;
        valmap.set_pix(xs, ys, RBColorf::red);
      }
    }
    
    
    {
      ft thetay = spherical_theta(wo), phix = spherical_phi(wo);
      int xs = 200 * phix*INV_2PI;
      int ys = 100 * thetay * 2 * INV_PI;

      
      valmap.set_pix(xs-1, ys-1, RBColorf::yellow);
      valmap.set_pix(xs - 1, ys, RBColorf::yellow);
      valmap.set_pix(xs - 1, ys + 1, RBColorf::yellow);
      valmap.set_pix(xs, ys - 1, RBColorf::yellow);
      valmap.set_pix(xs, ys, RBColorf::yellow);
      valmap.set_pix(xs, ys + 1, RBColorf::yellow);
      valmap.set_pix(xs + 1, ys - 1, RBColorf::yellow);
      valmap.set_pix(xs + 1, ys, RBColorf::yellow);
      valmap.set_pix(xs + 1, ys + 1, RBColorf::yellow);
      
    }
    
    {
      f32* dao3 = valmap.pix;
      RBColor32* dao = new RBColor32[200*200];
      for (int i = 0; i < 40000; ++i)
      {
        int y = i / 200;
        int x = i % 200;
        if (y == 100)
          dao[i] = RBColor32(RBColorf::red);
        else
        { 
          dao[i].r = (u8)(
            RBMath::clamp(RBMath::abs( dao3[i * 3 + 2]),0.f,1.f)*255);
          dao[i].g = (u8)(
            RBMath::clamp(RBMath::abs( dao3[i * 3 + 1]), 0.f, 1.f) * 255);
          dao[i].b = (u8)(
            RBMath::clamp(RBMath::abs(dao3[i * 3 ]), 0.f, 1.f) * 255);
          dao[i].a = 255;
        }
        
      }
      D3D11_MAPPED_SUBRESOURCE resource;
      renderer->context->Map(renderer->textures[brdf_uv_tex_id].texture,
        0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
      u8* da = reinterpret_cast<u8*>(resource.pData);
      u8* dao_c = (u8*)dao;
      //GPU有自己的内存布局，不要整块拷贝
      for (int a = 0; a < 200; ++a)
      {
        memcpy(da, dao_c, sizeof(RBColor32)*200);
        da += resource.RowPitch;
        dao_c += 200*sizeof(RBColor32);
      }
      renderer->context->Unmap(renderer->textures[brdf_uv_tex_id].texture, 0);
      delete dao;
    }
    delete bxdf;
    delete m;
    } while (0);
  }
  ImGui::Image(renderer->textures[brdf_uv_tex_id].srv, ImVec2(200,200));
  ImGui::Separator();

  return change;
}

class OLPBRMaterial* DisneyMaterialMetaData::create_material()
{
  OLPBRTexture<RBColorf>* bc = nullptr;
  if (base_colort == "")
    bc = new OLPBRConstantTexture<RBColorf>(base_color);
  else
    bc = new OLPBRImageTexture(base_colort.data());

  OLPBRTexture<f32>* p[10];
  for (int i = 0; i < 10; ++i)
  {
    if (paramt[i] == "")
      p[i] = new OLPBRConstantTexture<f32>(param[i]);
    else
      p[i] = new OLPBRImageTextureRounghness(paramt[i].data());
  }

  if (alpha_t == "")
    return new OLPBRDisneyMaterial(bc, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8],p[9]);
  else
  {
    auto* s = new OLPBRImageTexture(alpha_t.data());
    return new OLPBRAlphaDisneyMaterial(bc, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9],s);
  }

}
