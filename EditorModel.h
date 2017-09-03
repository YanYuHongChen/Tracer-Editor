#pragma once
#include <vector>
#include "Vector3.h"
#include "Vector2.h"
#include "Matrix.h"
#include "RHID3D11.h"
#include "D3DCompiler.h"
#include "D3D11FastRenderer.h"
#include "TracerBase.h"
#include "RBCamera.h"
#include <map>
#include "../base/Assertion.h"
#include "./thirdpart/imgui/addon/imguinodegrapheditor.h"
#include "./thirdpart/imgui/addon/ImGuiDock.h"
#include "./thirdpart/imgui/addon/ImGuizmo.h"

class PreViewRenderer;
using std::vector;

struct HitPoint
{

};

struct Vertex_PNT
{
  RBVector3 pos;
  RBVector3 normal;
  RBVector2 texcoord;
};

struct D3D11VertexBuffer
{
  VertexBufferID vb;
  IndexBufferID ib;
  VertexFormatID vf;
  u32 ib_size;
};

class MetaData
{
public:
  virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr,  int texid=-1) = 0;
  virtual size_t get_serialize_size(){ return 0; }
  virtual void serialize(char* mem){}
  virtual void deserialize(const char* mem, size_t size){}
};

class CameraMetaData : public MetaData
{
public:

  virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid=-1)
  {

  }
};

class MaterialMetaData : public MetaData
{
public:
	enum MatrialType
	{
		E_MM_METAL,
		E_MM_GLASS,
		E_MM_MATTE,
		E_MM_SIMPLEGLASS,
		E_MM_FRESNEL_BLEND,
		E_MM_PLASTIC,
		E_MM_MIRROR,
		E_MM_HAIR,
		E_MM_SUBSTRATE,
		E_MM_TRANSLUCENT,
		E_MM_UBER,
		E_MM_MIX,
    E_MM_DISNEY,
    E_MM_BLANK,
    E_MM_ALPHA_DISNEY,
	};
  MaterialMetaData() :pre_spp(1),change(false),reg_color(RBColorf::magenta){}
  virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid=-1) = 0;
  virtual class OLPBRMaterial* create_material() = 0;
  void preview(PreViewRenderer* pre_r, int texid, Direct3D11Renderer* _d3d11_renderer);

  void select_a_image(std::string& name,const char* filter);
  //辨别颜色
  RBColorf reg_color;
  int pre_spp;
  int type;
  bool change;
  void change_applied()
  {
    change = false;
  }
  void on_change_color_gui();

  virtual size_t get_serialize_size()
  { 
	  return sizeof(int)+sizeof(RBColorf);
  }
  virtual void serialize(char*& mem)
  {
	  write_data(mem, type);
	  write_data(mem,reg_color);
  }
  virtual void deserialize(const char*& mem, size_t size)
  {
	  read_data(mem, type);
	  read_data(mem, reg_color);
  }

  template<class T>
  static void write_data(char*& mem,const T& data)
  {
	  *((T*)mem) = data;
	  mem += sizeof(T);
  }

  template<>
  static void write_data<bool>(char*& mem, const bool& data)
  {
	  *((int*)mem) = data? 1 : 0;
	  mem += sizeof(int);
  }

  template<class T>
  static void read_data(const char*& mem, T& data)
  {
	  data = *((T*)mem);
	  mem += sizeof(T);
  }

  template<>
  static void read_data<bool>(const char*& mem, bool& data)
  {
	  data = *((int*)mem) == 0 ? false : true;
	  mem += sizeof(int);
  }

};

class MetalMaterialMetaData : public MaterialMetaData
{
public:
  MetalMaterialMetaData()
  {
    eta = RBColorf::white;
    k = RBColorf::white;
    k_t = "";
    uv_roughness = RBVector2(0.1,0.1);
    roughness_t = "";
    type = 0;
  }
  ~MetalMaterialMetaData()
  {

  }

  void release(Direct3D11Renderer* renderer)
  {

  }
  virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr , int texid=-1);
  virtual class OLPBRMaterial* create_material();

  RBColorf eta;
  RBColorf k;
  std::string k_t;
  RBVector2 uv_roughness;
  std::string roughness_t;


  virtual size_t get_serialize_size()
  {
    //继承类有虚表指针
    size_t size = sizeof(RBColorf) * 2 + sizeof(RBVector2) + MaterialMetaData::get_serialize_size();
    return size;
  }
  virtual void serialize(char*& mem)
  {
	  MaterialMetaData::serialize(mem);
	  write_data(mem,eta);
	  write_data(mem, k);
	  write_data(mem, uv_roughness);
  }

  virtual void deserialize(const char*& mem, size_t size)
  {
	  const char* p = mem;
	  MaterialMetaData::deserialize(mem, size);
	  read_data(mem, eta);
	  read_data(mem, k);
	  read_data(mem, uv_roughness);
    CHECK((mem-p)<=size);
  }

};

class GlassMaterialMetaData : public MaterialMetaData
{
public:
  GlassMaterialMetaData()
  {
    kr = RBColorf::white;
    kr_t = "";
    kt = RBColorf::white;
    kt_t = "";
    uv_roughness = RBVector2(0.1,0.1);
    remap_roghness = true;
    eta = 1.2;
    type = 1;

  }
  ~GlassMaterialMetaData()
  {

  }
  void release(Direct3D11Renderer* renderer)
  {

  }
  virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid=-1);
  virtual class OLPBRMaterial* create_material();

  RBColorf kr;
  std::string kr_t;
  RBColorf kt;
  std::string kt_t;
  RBVector2 uv_roughness;
  f32 eta;
  bool remap_roghness;


  virtual size_t get_serialize_size()
  {
    return sizeof(RBColorf) * 2 + sizeof(RBVector2) + sizeof(f32) + sizeof(int) + MaterialMetaData::get_serialize_size();
  }

  virtual void serialize(char*& mem)
  {
	  MaterialMetaData::serialize(mem);
	  write_data(mem, kr);
	  write_data(mem, kt);
	  write_data(mem, uv_roughness);
	  write_data(mem, eta);
	  write_data(mem, remap_roghness);
  }

  virtual void deserialize(const char*& mem, size_t size)
  {
	  const char* p = mem;
	  MaterialMetaData::deserialize(mem, size);
	  read_data(mem, kr);
	  read_data(mem, kt);
	  read_data(mem, uv_roughness);
	  read_data(mem, eta);
	  read_data(mem, remap_roghness);
    CHECK((mem-p)<=size);
  }
};

class MatteMaterialMetaData : public MaterialMetaData
{
public:
	RBColorf kd;
	f32 sigma;

  std::string kd_t;


	MatteMaterialMetaData()
	{
		kd = RBColorf::white;
		sigma = 0;
    kd_t = "";

		type = E_MM_MATTE;
	}
	~MatteMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		size_t size = sizeof(RBColorf) + sizeof(f32) + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem, kd);
		write_data(mem, sigma);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem, size);
		read_data(mem, kd);
		read_data(mem, sigma);
		CHECK((mem-p) <= size);
	}
};

class SimpleGlassMaterialMetaData : public MaterialMetaData
{
public:
	RBColorf kr;
  std::string kr_t;
	RBColorf kt;
  std::string kt_t;
	f32 eta;
	SimpleGlassMaterialMetaData()
	{
		kr = RBColorf::white;
    kr_t = "";
		kt = RBColorf::white;
    kt_t = "";
		eta = 1.2;
		type = E_MM_SIMPLEGLASS;
	}
	~SimpleGlassMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		//继承类有虚表指针
		size_t size = sizeof(f32) + sizeof(RBColorf) * 2 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem, kr);
		write_data(mem, kt);
		write_data(mem, eta);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem, size);
		read_data(mem, kr);
		read_data(mem, kt);
		read_data(mem, eta);
		CHECK((mem-p) <= size);
	}
};

class FresnelBlendMaterialMetaData :public MaterialMetaData
{
public:
	RBColorf rd, rs;
	f32 roughness;
	FresnelBlendMaterialMetaData()
	{
		rd = RBColorf::white;
		rs = RBColorf::white;
		roughness = 1.f;
		type = E_MM_FRESNEL_BLEND;
	}
	~FresnelBlendMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		//继承类有虚表指针
		size_t size = sizeof(f32) + sizeof(RBColorf) * 2 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem,rd);
		write_data(mem, rs);
		write_data(mem, roughness);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem, size);
		read_data(mem, rd);
		read_data(mem, rs);
		read_data(mem, roughness);
		CHECK((mem-p) <= size);
	}
};

class PlasticMaterialMetaData : public MaterialMetaData
{
public:
	RBColorf kd, ks;
  std::string kd_t, ks_t;
	f32 roughness;
  std::string roughness_t;
	PlasticMaterialMetaData()
	{
		kd = RBColorf::white;
    kd_t = "";
		ks = RBColorf::white;
    ks_t = "";
		roughness = 1.f;
    roughness_t = "";
		type = E_MM_PLASTIC;
	}
	~PlasticMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		//type
		//继承类有虚表指针
		size_t size = sizeof(f32) + sizeof(RBColorf) * 2 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem, kd);
		write_data(mem, ks);
		write_data(mem,roughness);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem, size);
		read_data(mem, kd);
		read_data(mem, ks);
		read_data(mem, roughness);
		CHECK((mem-p) <= size);
	}
};

class MirrorMaterialMetaData :public MaterialMetaData
{
public:
	RBColorf kr;
	MirrorMaterialMetaData()
	{
		kr = RBColorf::white;
		type = E_MM_MIRROR;
	}
	~MirrorMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		//继承类有虚表指针
		size_t size = sizeof(RBColorf) + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem, kr);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem,size);
		read_data(mem,kr);
		CHECK((mem-p) <= size);
	}
};

class HairMaterialMetaData : public MaterialMetaData
{
public:
	RBColorf kd;
  std::string kd_t;

	RBColorf ks;
  std::string ks_t;

	f32 roughness;
	HairMaterialMetaData()
	{
		kd = RBColorf::white;
    ks_t = "";
		ks = RBColorf::white;
    kd_t == "";
		roughness = 1.f;
		type = E_MM_HAIR;
	}
	~HairMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		//继承类有虚表指针
		size_t size = sizeof(f32) + sizeof(RBColorf) * 2 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem, kd);
		write_data(mem, ks);
		write_data(mem, roughness);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem, size);
		read_data(mem, kd);
		read_data(mem, ks);
		read_data(mem, roughness);
		CHECK((mem-p) <= size);
	}
};

class SubstrateMaterialMetaData : public MaterialMetaData
{
public:
	RBColorf kd;
	RBColorf ks;
	f32 nu, nv;
	SubstrateMaterialMetaData()
	{
		kd = RBColorf::white;
		ks = RBColorf::white;
		nu = 1.f;
		nv = 1.f;
		type = E_MM_SUBSTRATE;
	}
	~SubstrateMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		//继承类有虚表指针
		size_t size = sizeof(f32)*2 + sizeof(RBColorf) * 2 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem, kd);
		write_data(mem, ks);
		write_data(mem, nu);
		write_data(mem, nv);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem,size);
		read_data(mem, kd);
		read_data(mem, ks);
		read_data(mem, nu);
		read_data(mem, nv);
		CHECK((mem-p) <= size);
	}
};

class TranslucentMaterialMetaData : public MaterialMetaData
{
public:
	RBColorf kd, ks;
	f32 roughness;
	RBColorf refl, trans;
	TranslucentMaterialMetaData()
	{
		kd = RBColorf::white;
		ks = RBColorf::white;
		roughness = 1.f;
		refl = RBColorf::white;
		trans = RBColorf::white;
		type = E_MM_TRANSLUCENT;
	}
	~TranslucentMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		size_t size = sizeof(f32) + sizeof(RBColorf) * 4 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem, kd);
		write_data(mem, ks);
		write_data(mem, roughness);
		write_data(mem, refl);
		write_data(mem, trans);
	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
		MaterialMetaData::deserialize(mem, size);
		read_data(mem, kd);
		read_data(mem, ks);
		read_data(mem, roughness);
		read_data(mem, refl);
		read_data(mem, trans);
		CHECK((mem-p) <= size);
	}
};

class UberMaterialMetaData :public MaterialMetaData
{
public:
	RBColorf kd, ks, kr, kt;
  std::string kd_t, ks_t, kr_t, kt_t;
	f32 roughness, ru, rv;
  std::string r_t;
	RBColorf opacity;
  std::string opacity_t;
	f32 eta;

	UberMaterialMetaData()
	{
    kd = RBColorf::white; kd_t = "";
    ks = RBColorf::white; ks_t = "";
    kr = RBColorf::white; kr_t = "";
    kt = RBColorf::white; kt_t = "";
    roughness = 1.f; r_t = "";
		ru = 1.f;
		rv = 1.f;
    opacity = RBColorf::white; opacity_t = "";
		eta = 1.2;
		type = E_MM_UBER;
	}
	~UberMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		size_t size = sizeof(f32) *4+ sizeof(RBColorf) * 5 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{
		MaterialMetaData::serialize(mem);
		write_data(mem,kd);
		write_data(mem, ks);
		write_data(mem, kr);
		write_data(mem, kt);
		write_data(mem, roughness);
		write_data(mem, ru);
		write_data(mem, rv);
		write_data(mem, opacity);
		write_data(mem, eta);

	}

	virtual void deserialize(const char*& mem, size_t size)
	{
		const char* p = mem;
    MaterialMetaData::deserialize(mem, size);
		read_data(mem, kd);
		read_data(mem, ks);
		read_data(mem, kr);
		read_data(mem, kt);
		read_data(mem, roughness);
		read_data(mem, ru);
		read_data(mem, rv);
		read_data(mem, opacity);
		read_data(mem, eta);
		CHECK((mem-p) <= size);
	}
};

class DisneyMaterialMetaData :public MaterialMetaData
{
public:
  RBColorf base_color;
  std::string base_colort;
  f32 param[10];
  std::string paramt[10];

  int brdf_sample_spp;
  f32 u, v;
  f32 brdf_scale = 1.f;

  bool alpha;
  std::string alpha_t;

  DisneyMaterialMetaData(bool alpha)
  {
    base_colort = "";
    base_color = RBColorf::white;
    for (int i = 0; i < 10;++i)
    {
      param[i] = 0.f;
      paramt[i] = "";
    }
    type = E_MM_DISNEY;
    brdf_uv_tex = nullptr;
    brdf_uv_tex_id = -1;

    brdf_sample_spp = 1000;
    u = 3.14*0.25;
    v = u;

    this->alpha = alpha;
    alpha_t = "";
  }
  ~DisneyMaterialMetaData()
  {

  }
  void release(Direct3D11Renderer* renderer)
  {

  }
  virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
  virtual class OLPBRMaterial* create_material();
  virtual size_t get_serialize_size()
  {
    size_t size = sizeof(f32) * 10 + sizeof(RBColorf)  + MaterialMetaData::get_serialize_size();
    return size;
  }
  virtual void serialize(char*& mem)
  {
    MaterialMetaData::serialize(mem);
    write_data(mem, base_color);
    for (int i = 0; i < 10; ++i)
      write_data(mem, param[i]);

  }

  virtual void deserialize(const char*& mem, size_t size)
  {
    const char* p = mem;
    MaterialMetaData::deserialize(mem, size);
    read_data(mem, base_color);
    for (int i = 0; i < 10; ++i)
      read_data(mem, param[i]);
    CHECK((mem - p) <= size);
  }
  ID3D11Texture2D* brdf_uv_tex;
  TextureID brdf_uv_tex_id;


};


class BlankMaterialMetaData :public MaterialMetaData
{
public:

  BlankMaterialMetaData()
  {
  }
  ~BlankMaterialMetaData()
  {

  }
  void release(Direct3D11Renderer* renderer)
  {}
  virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
  virtual class OLPBRMaterial* create_material();
  virtual size_t get_serialize_size()
  {
    size_t size = MaterialMetaData::get_serialize_size();
    return size;
  }
  virtual void serialize(char*& mem)
  {
    MaterialMetaData::serialize(mem);
  }

  virtual void deserialize(const char*& mem, size_t size)
  {
    const char* p = mem;
    MaterialMetaData::deserialize(mem, size);
    CHECK((mem - p) <= size);
  }
  ID3D11Texture2D* brdf_uv_tex;
  TextureID brdf_uv_tex_id;


};


enum class NodeType {
  ConstantColor,
  MatrialRef,
  Constant,
  Mix,
  MixMask,
  Mul
};

static const char* NodeTypeStr[] = {
  "Color",
  "Material",
  "Constant",
  "MixMat",
  "MixMask",
  "Mul"
};

static const char* NodeTooltipStr[] = {
  "1",
  "Convert the noise input data into RGBA image",
  "Provide a constant as input parameter for other nodes",
  "2",
  "MixMaskNode",
  "Mul"
};
void nodeCallback(ImGui::Node*& node, ImGui::NodeGraphEditor::NodeState state, ImGui::NodeGraphEditor& editor);
void linkCallback(const ImGui::NodeLink& link, ImGui::NodeGraphEditor::LinkState state, ImGui::NodeGraphEditor& editor);
ImGui::Node* nodeFactory(int nodeType, const ImVec2& pos);

struct node_editor_param
{
  class LibMaterial* meta_data;
  class EditorSceneView* editor_view;
};

class MixMaterialMetaData :public MaterialMetaData
{
public:
  node_editor_param* param;
  MixMaterialMetaData(class EditorSceneView* editor_view,class  LibMaterial* mat);
	~MixMaterialMetaData()
	{

	}
	void release(Direct3D11Renderer* renderer)
	{

	}
	virtual bool on_gui(Direct3D11Renderer* renderer = nullptr, PreViewRenderer* pre_r = nullptr, int texid = -1);
	virtual class OLPBRMaterial* create_material();
	virtual size_t get_serialize_size()
	{
		size_t size = 0 + MaterialMetaData::get_serialize_size();
		return size;
	}
	virtual void serialize(char*& mem)
	{

	}

	virtual void deserialize(const char*& mem, size_t size)
	{

	}

  ImGui::NodeGraphEditor node_editor;

};

class Mesh
{
public:
	~Mesh();
  Mesh() : kdtree(nullptr){}
  void release_d3d11_buffer(Direct3D11Renderer* d3d11_renderer);
  void release_data();
  bool intersect(const OLPBRRay_& ray, OLPBRItersc_& isc) const
  {
    if (kdtree)
    { 
      bool res = kdtree->intersection_res(ray, kdtree->root, &isc, triangles);
      return res;
    }
    else
    {
      bool res = false;
      for (int i = 0; i < triangles.size();++i)
      {
        bool res1 = triangles[i]->intersect(ray,isc);
        res |= res1;
      }
      return res;
    }
  }

  //obj索引
  vector<int> vertex_idx;
  vector<int> tex_idx;
  vector<int> nomral_idx;
  //obj纹理坐标
  vector<RBVector2> tex;
  //obj法线
  vector<RBVector3> normals;
  //obj顶点位置
  vector<RBVector3> vertices;
  vector<OLPBRGeometry_*> triangles;
  RBAABB bound;
  //加速结构
  OLPBRKDTree_* kdtree;
  //渲染顶点buffer
  D3D11VertexBuffer d3d11_buffer;
  std::string name;
  
};

class Object
{
public:

};

//基本实例
class MeshInstance
{
public:
  MeshInstance() :selected(false), hide(false),pos(RBVector3::zero_vector)
  ,rot(RBVector3::zero_vector),sca(RBVector3(1.f,1.f,1.f)){}
  const Mesh* object_ref;
  RBMatrix xtolocal;
  RBMatrix localtox;
  bool hide;
  bool selected;

  virtual void on_gui(class EditorSceneView* editor_view){}

  void translate(const RBVector3& v)
  {
    pos += v;
    localtox.traslate(v.x, v.y, v.z);
    xtolocal = localtox.get_inverse_slow();
  }
  void scale(const RBVector3& v)
  {
    sca += v;
    localtox.scale(v.x, v.y, v.z);
    xtolocal = localtox.get_inverse_slow();
  }
  void roatate(const RBVector3& v)
  {
    rot += v;
    localtox.rotate(v.x, v.y, v.z);
    xtolocal = localtox.get_inverse_slow();
  }
  void translate_to(const RBVector3& v)
  {
    pos = v;
    localtox.set_translation(v.x, v.y, v.z);
    xtolocal = localtox.get_inverse_slow();
  }
  void scale_to(const RBVector3& v)
  {
    sca = v;
    localtox.set_scale(v.x, v.y, v.z);
    xtolocal = localtox.get_inverse_slow();
  }
  void rotate_to(const RBVector3& v)
  {
    rot = v;
    localtox.set_rotation(v.x, v.y, v.z);
    xtolocal = localtox.get_inverse_slow();
  }
  inline const RBVector3& get_pos()
  {
    return pos;
  }
  inline const RBVector3& get_rot()
  {
    return rot;
  }
  inline const RBVector3& get_scale()
  {
    return sca;
  }

private:
  RBVector3 pos;
  RBVector3 rot;
  RBVector3 sca;

};

class ModelObjectInstance : public MeshInstance
{
public:
  ModelObjectInstance() :material_id(-1), is_area_light(false),le(RBColorf::white){}
  int material_id;
  bool is_area_light;
  RBColorf le;
  int nsamples = 1;
  virtual void on_gui(class EditorSceneView* editor_view) override;
};

class CameraObjectInstance : public MeshInstance
{
public:
  //camera data
};

//平移，旋转，缩放标志
class UtilitiesObjectInstance : public MeshInstance
{
public:

};

class PointLightObjectInstance : public MeshInstance
{
public:
  //light data
  PointLightObjectInstance(Mesh* mesh)
  {
    object_ref = mesh;
    intensity = RBColorf::white;
  }
  RBColorf intensity;

  virtual void on_gui(class EditorSceneView* editor_view) override;
};

class AreaLightObjectInstance : public MeshInstance
{
public:

};



class ObjectInstanceSet
{
public:
  //层次object
  vector<MeshInstance*> sub_object;
  RBMatrix world_2_local;
  RBMatrix local_2_world;
};

struct LibMaterial
{
	~LibMaterial();

  LibMaterial(Direct3D11Renderer* _d3d11_renderer, PreViewRenderer* pre_render);


  void release(Direct3D11Renderer* _d3d11_renderer)
  {
	  if (_ui_show_tex == -1)
		  return;
    _d3d11_renderer->removeTexture(_ui_show_tex);
    _ui_show_tex = -1;
    tex = 0;
  }
  class OLPBRMaterial* offline_material;
  MaterialMetaData* material_data;
  TextureID _ui_show_tex;
  ID3D11Texture2D* tex;
  //Direct3D11Renderer* _d3d11_renderer=nullptr;
  std::string name="";
};

class EditorScene
{
public:
  EditorScene()
  {
    hold_scene = "";
	scene_kdtree = nullptr;
  }
  void init(int matn, int meshn=0)
  {
    material_lib.resize(matn);
    memset(material_lib.data(), 0, material_lib.size()*sizeof(void*));
  }
  MeshInstance* cast_ray(const OLPBRRay_& ray, OLPBRItersc_& isc)
  {
    f32 nearest = MAX_F32;
    MeshInstance* mesh_seleceted = nullptr;

    for (auto& it : objects)
    {
      for (int i = 0; i < it->sub_object.size();++i)
      {
        MeshInstance* its = it->sub_object[i];
        RBMatrix im = it->world_2_local*its->xtolocal;
        OLPBRRay_ ray_inter = ray;
        RBVector4 op = RBVector4(ray_inter.o, 1)*im;
        op /= op.w;
        ray_inter.d = im.transform_vector3(ray_inter.d);
        ray_inter.o = op;
        bool s = its->object_ref->intersect(ray_inter,isc);
        if (s)
        {
          if(isc.dist_ < nearest)
          {
            nearest = isc.dist_;
            mesh_seleceted = its;
          }
        }
      }
    }
    return mesh_seleceted;
  }

  std::vector<Mesh*>& read_model(const char* filename, int kd_threshold, Direct3D11Renderer* _d3d11_renderer, ShaderID bound_shader, bool release_data = true);
  ModelObjectInstance* create_instance_from_filename(const char* filename);

  int create_material();

  void save_scene(const char* filename);
  bool load_scene(const char* filename, Direct3D11Renderer* d3d11_render,PreViewRenderer* pre_r,ShaderID bound_shader);

  //预处理obj，构建kdtree，储存三角形，d3dbuffer,name,bound
  void process_obj(const char* filename);

  void clear_scene(Direct3D11Renderer* d3d11_rendere);

  vector<ObjectInstanceSet*> objects;
  vector<ObjectInstanceSet*> assistant_objects;
  vector<ObjectInstanceSet*> lights;

  //可选构建
  OLPBRKDTree_* scene_kdtree;

  std::map<std::string, std::vector< Mesh*> > loaded_meshes;
  std::map<std::string, std::vector< Mesh*> > loaded_assistant_meshes;
  std::vector< LibMaterial* > material_lib;

  std::vector<std::string> imported_file_names;
  std::string hold_scene;
};

struct EditorBrdf 
{
  bool check(class OLPBRBXDF* bxdf, int spp = 1000);

  virtual void on_gui(){}
};

struct MicrofacetTransmissionBrdf : public EditorBrdf
{
  virtual void on_gui();
  
};

struct MicrofacetReflectionBrdf : public EditorBrdf
{
  MicrofacetReflectionBrdf()
  {
    uv_roughness = RBVector2::zero_vector;
    eta = RBColorf::white;
    k = RBColorf::gray;
  }
  virtual void on_gui();
  RBColorf eta;
  RBColorf k;
  RBVector2 uv_roughness;
};

struct LambertianReflectionBrdf : public EditorBrdf
{
  virtual void on_gui();

};

struct LambertianTransmissionBrdf : public EditorBrdf
{
  virtual void on_gui();

};

struct OrenNayarBrdf : public EditorBrdf
{
  virtual void on_gui();

};

struct FresnelBlendBrdf : public EditorBrdf
{
  virtual void on_gui();

};

struct KajiyaKayBrdf : public EditorBrdf
{
  virtual void on_gui();

};




//只管显示和交互
class EditorSceneView
{
public:
  void init(Direct3D11Renderer* d3d11_renderer, ID3D11RenderTargetView* b);
  void load_sys_assets();
  void create_rhires(int ww, int wh);
  void init_assistant_object();
  void init_d3d11(int ww, int wh);
  void update(f32 dt);
  bool gui_update(f32 dt);
  void render();
  void reset();
  void init_hit_sphere()
  {
    std::vector<Mesh*>& itk = scene_model->read_model("./Res/editor/hit_sphere.obj", 50000, _d3d11_renderer, mode_render_shader);
    ObjectInstanceSet*  a = new ObjectInstanceSet();
    a->local_2_world.set_identity();
    a->world_2_local = a->local_2_world.get_inverse_slow();
    for (int i = 0; i < itk.size(); ++i)
    {
      UtilitiesObjectInstance* meshi = new UtilitiesObjectInstance();
      meshi->hide = false;
      meshi->object_ref = itk[i];
      meshi->localtox.set_identity();
      meshi->xtolocal = meshi->localtox.get_inverse_slow();

      a->sub_object.push_back(meshi);
    }

    scene_model->assistant_objects.push_back(a);
    hit_sphere = a;
  }

  bool on_create_material_gui();
  bool on_material_edit_gui();
  bool on_render_gui();
  void on_show_result_gui();
  void on_save_read_mat_gui();
  void on_set_camera_gui();
  void on_check_brdf_gui();
  void on_dock_gui();

  ~EditorSceneView();

  int ww;
  int wh;

  vector<MetaData*> cur_metadata;
  RBCamera* scene_camera;
  RBVector2 _cam_move_speed;
  RBVector2 _cam_rotate_speed;

  MeshInstance* selected_mesh_instance;
  int selected_material_id;

  ObjectInstanceSet* hit_sphere;

  EditorScene* scene_model;
  Direct3D11Renderer* _d3d11_renderer;
  ID3D11RenderTargetView* _back_buffer_view;

  class PreViewRenderer* pre_renderer;
  class ProductionRenderer* prod_renderer;

  std::vector< EditorBrdf* >brdf;

  bool is_use_ibl;

  //d3d11
  ShaderID mode_render_shader;
  BlendStateID _bs_enable;
  DepthStateID _ds_enable;
  RasterizerStateID _rs;
  TextureID _buffer_depth_stencil;
  TextureID _ui_show_tex;
  ID3D11Texture2D* tex;

  RBVector2 ass_scale;

  bool show_help;
  bool show_material_editor;
  bool show_render;
  bool show_camera_setting;
  bool show_debug_tool;


  ImGui::NodeGraphEditor node_editor;
  ImGui::NodeGraphEditor* cur_node_editor = nullptr;

  ImGuiDock::Dock node_editor_dock;
  ImGuiDock::Dock post_dock;


  ImGuiDock::Dock create_material_editor;

  ImGuiDock::Dock debug_dock;



  ImGuiDock::Dock render_dock;
  ImGuiDock::Dock camera_dock;
  ImGuiDock::Dock material_editor;
  ImGuiDock::Dock object_editor;


  ImGuiDock::Dockspace dockspace;
  ImGuiDock::Dockspace right_window;
  f32 focus_distance;
  bool open_left_editors = false;
};