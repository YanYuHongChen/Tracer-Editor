#pragma once
#include "Vector3.h"
#include "AABB.h"
#include "MemoryFrame.h"
#include <vector>
#include "Assertion.h"

#define ft f32

class OLPBRRay_
{
public:
  //最小t不与原点重合
  OLPBRRay_() :t(MAX_F32), min_t(MIN_F32), max_t(MAX_F32), depth_(0)/*,pix_x(0.f),pix_y(0.f)*/{}
  OLPBRRay_(const RBVector3& to, const RBVector3& td) :o(to), d(td), t(0), min_t(MIN_F32), max_t(MAX_F32), depth_(0)/*,pix_x(0.f),pix_y(0.f)*/{}
  OLPBRRay_(const RBVector3& to, const RBVector3& td, ft tmin) :o(to), d(td), t(0), min_t(tmin), max_t(MAX_F32), depth_(0)/*,pix_x(0.f),pix_y(0.f)*/{}
  OLPBRRay_(const RBVector3& to, const RBVector3& td, ft tmin, ft tmax) :o(to), d(td), t(0), min_t(tmin), max_t(tmax), depth_(0)/*,pix_x(0.f),pix_y(0.f)*/{}
  RBVector3 o;
  RBVector3 d;
  ft t;
  ft min_t;
  ft max_t;
  int depth_;

  std::vector<OLPBRRay_> history;
};

class OLPBRItersc_
{
public:
  ~OLPBRItersc_(){}
  OLPBRItersc_() : dist_(MAX_F32), g_(nullptr), epsilon_(MIN_F32){}
  OLPBRItersc_(ft mdst) : dist_(mdst), g_(nullptr), epsilon_(MIN_F32){}

  void reset()
  {
    dist_ = (MAX_F32);
    g_ = (nullptr);
    epsilon_ = (MIN_F32);
  }


  RBVector3 normal_;
  RBVector3 shading_normal_;
  RBVector3 p_;
  ft dist_;
  const class OLPBRGeometry_* g_;
  ft epsilon_;
  RBVector2 uv;
};


class OLPBRGeometry_
{
public:
  OLPBRGeometry_() :type(-1),  debug_tag(0){}
  virtual ~OLPBRGeometry_(){};
  //自动更新isec到最近点
  virtual bool intersect(const OLPBRRay_& ray, OLPBRItersc_& isec) const = 0;
  virtual bool intersectP(const OLPBRRay_& ray) const = 0;
  virtual RBAABB bound() = 0;
  RBColorf c;
  int tag;
  ft area;
  RBAABB bound_;
  int type;
  int debug_tag;
  virtual void calculate_area() = 0;
};

//三角形bound消除了无厚度的情形
class OLPBRTriangle_ : public OLPBRGeometry_
{
public:
  ~OLPBRTriangle_(){}

  OLPBRTriangle_() = delete;
  OLPBRTriangle_(const RBVector3& p0, const RBVector3& p1, const RBVector3& p2)
  {
    v_[0] = p0;
    v_[1] = p1;
    v_[2] = p2;
    type = 1;
    rebuild_normal();
    sn_[0] = sn_[1] = sn_[2] = normal_;
    uv_[0] = RBVector2(0, 0);
    uv_[1] = RBVector2(1, 0);
    uv_[2] = RBVector2(1, 1);


  }
  OLPBRTriangle_(const RBVector3& p0, const RBVector3& p1, const RBVector3& p2, const RBVector3& n0, const RBVector3& n1, const RBVector3& n2)
  {
    v_[0] = p0;
    v_[1] = p1;
    v_[2] = p2;
    type = 1;
    rebuild_normal();
    sn_[0] = n0;
    sn_[1] = n1;
    sn_[2] = n2;
    uv_[0] = RBVector2(0, 0);
    uv_[1] = RBVector2(1, 0);
    uv_[2] = RBVector2(1, 1);
  }
  OLPBRTriangle_(const RBVector3& p0, const RBVector3& p1, const RBVector3& p2, const RBVector3& n0, const RBVector3& n1, const RBVector3& n2, const RBVector2& uv0, const RBVector2& uv1, const RBVector2& uv2)
  {
    v_[0] = p0;
    v_[1] = p1;
    v_[2] = p2;
    type = 1;
    rebuild_normal();
    sn_[0] = n0;
    sn_[1] = n1;
    sn_[2] = n2;
    uv_[0] = uv0;
    uv_[1] = uv1;
    uv_[2] = uv2;
  }
  OLPBRTriangle_(const RBVector3& p0, const RBVector3& p1, const RBVector3& p2, const RBVector2& uv0, const RBVector2& uv1, const RBVector2& uv2)
  {
    v_[0] = p0;
    v_[1] = p1;
    v_[2] = p2;
    type = 1;
    rebuild_normal();
    sn_[0] = sn_[1] = sn_[2] = normal_;
    uv_[0] = uv0;
    uv_[1] = uv1;
    uv_[2] = uv2;
  }
  void update_triangle(const RBVector3& p0, const RBVector3& p1, const RBVector3& p2, const RBVector3& n0, const RBVector3& n1, const RBVector3& n2)
  {
    v_[0] = p0;
    v_[1] = p1;
    v_[2] = p2;
    type = 1;
    rebuild_normal();
    sn_[0] = n0;
    sn_[1] = n1;
    sn_[2] = n2;
    bound_.reset();
  }
  void update_triangle(const RBVector3& p0, const RBVector3& p1, const RBVector3& p2)
  {
    v_[0] = p0;
    v_[1] = p1;
    v_[2] = p2;
    type = 1;
    rebuild_normal();
    sn_[0] = sn_[1] = sn_[2] = normal_;
    bound_.reset();

  }
  void rebuild_normal()
  {
    normal_ = RBVector3::cross_product(v_[1] - v_[0], v_[2] - v_[0]).get_normalized();
  }
  virtual bool intersect(const OLPBRRay_& ray, OLPBRItersc_& isec) const
  {
    //todo:use double
    RBVector3 oa = v_[0] - ray.o;
    RBVector3 ob = v_[1] - ray.o;
    RBVector3 oc = v_[2] - ray.o;

    RBVector3 v0 = RBVector3::cross_product(oc, ob);
    RBVector3 v1 = RBVector3::cross_product(ob, oa);
    RBVector3 v2 = RBVector3::cross_product(oa, oc);

    ft v0d = RBVector3::dot_product(v0, ray.d);
    ft v1d = RBVector3::dot_product(v1, ray.d);
    ft v2d = RBVector3::dot_product(v2, ray.d);

    if (((v0d < 0.f) && (v1d < 0.f) && (v2d < 0.f)) ||
      ((v0d >= 0.f) && (v1d >= 0.f) && (v2d >= 0.f)))
    {
      float dis = RBVector3::dot_product(normal_, oa) / RBVector3::dot_product(normal_, ray.d);
      if ((dis>ray.min_t) && (dis<isec.dist_) && (dis<ray.max_t) && (dis>isec.epsilon_))
      {
        isec.normal_ = normal_;
        isec.p_ = ray.o + dis*ray.d;
        /*
        if (isec.dist_ > 1500)
        //太远不插值
        isec.shading_normal_ = normal_;
        else
        */
        isec.shading_normal_ = lerp_normal(isec.p_);
        isec.dist_ = dis;
        isec.g_ = this;

        isec.uv = lerp_uv(isec.p_);
        return true;
      }
    }

    return false;
  }

  virtual bool intersectP(const OLPBRRay_& ray) const
  {
    RBVector3 oa = v_[0] - ray.o;
    RBVector3 ob = v_[1] - ray.o;
    RBVector3 oc = v_[2] - ray.o;

    RBVector3 v0 = RBVector3::cross_product(oc, ob);
    RBVector3 v1 = RBVector3::cross_product(ob, oa);
    RBVector3 v2 = RBVector3::cross_product(oa, oc);

    ft v0d = RBVector3::dot_product(v0, ray.d);
    ft v1d = RBVector3::dot_product(v1, ray.d);
    ft v2d = RBVector3::dot_product(v2, ray.d);

    if (((v0d < 0.f) && (v1d < 0.f) && (v2d < 0.f)) ||
      ((v0d >= 0.f) && (v1d >= 0.f) && (v2d >= 0.f)))
    {
      float dis = RBVector3::dot_product(normal_, oa) / RBVector3::dot_product(normal_, ray.d);
      if ((dis>ray.min_t) && (dis<ray.max_t))
      {
        return true;
      }
    }

    return false;
  }

  
  virtual void calculate_area()
  {
    RBVector3 v = RBVector3::cross_product(v_[1] - v_[0], v_[2] - v_[0]);
    area = 0.5*v.size();
  }

  virtual RBAABB bound()
  {
    if (!bound_.is_available())
    {
      bound_.include(v_[0]);
      bound_.include(v_[1]);
      bound_.include(v_[2]);

      bound_.fix_thin();
    }

    return bound_;
  }

  RBVector2 lerp_uv(const RBVector3& p) const
  {
    RBVector3 v = RBVector3::cross_product(v_[1] - v_[0], v_[2] - v_[0]);
    f32 area_all = 0.5*v.size();
    v = RBVector3::cross_product(v_[0] - p, v_[2] - p);
    f32 area1 = 0.f;
    area1 = 0.5*v.size();
    v = RBVector3::cross_product(v_[2] - p, v_[1] - p);
    f32 area2 = 0.5*v.size();
    v = RBVector3::cross_product(v_[1] - p, v_[0] - p);
    f32 area3 = 0.5*v.size();
    f32 t2 = area1 / area_all;
    f32 t1 = area2 / area_all;
    f32 t3 = area3 / area_all;

    return ((t1)*uv_[0] + t2*uv_[1] + t3*uv_[2]);

  }

  RBVector3 lerp_normal(const RBVector3& p) const
  {

    RBVector3 v = RBVector3::cross_product(v_[1] - v_[0], v_[2] - v_[0]);
    f32 area_all = 0.5*v.size();
    v = RBVector3::cross_product(v_[0] - p, v_[2] - p);
    f32 area1 = 0.f;
    area1 = 0.5*v.size();
    v = RBVector3::cross_product(v_[2] - p, v_[1] - p);
    f32 area2 = 0.5*v.size();
    v = RBVector3::cross_product(v_[1] - p, v_[0] - p);
    f32 area3 = 0.5*v.size();

    f32 t2 = area1 / area_all;
    f32 t1 = area2 / area_all;
    f32 t3 = area3 / area_all;

    return ((t1)*sn_[0] + t2*sn_[1] + t3*sn_[2]).get_normalized();

  }

  RBVector3 v_[3];
  //shading normal
  RBVector3 sn_[3];
  //geometry normal
  RBVector3 normal_;
  //for mesh
  RBVector2 uv_[3];
};

class OLPBRSphere_ : public OLPBRGeometry_
{
public:
  OLPBRSphere_(){ type = 0; }
  OLPBRSphere_(const RBVector3& o, ft r) :o_(o), r_(r){ type = 0; }

  virtual bool intersect(const OLPBRRay_& ray, OLPBRItersc_& isec) const
  {
    // we transform ray origin into object space (center == origin)
    RBVector3 transformedOrigin = ray.o - o_;

    f64 A = RBVector3::dot_product(ray.d, ray.d);
    f64 B = 2 * RBVector3::dot_product(ray.d, transformedOrigin);
    f64 C = RBVector3::dot_product(transformedOrigin, transformedOrigin) - (r_ * r_);

    // Must use doubles, because when B ~ sqrt(B*B - 4*A*C)
    // the resulting t is imprecise enough to get around ray epsilons
    const double disc = B*B - 4 * A*C;

    if (disc < 0)
      return false;

    double discSqrt = RBMath::sqrt(disc);
    double q = (B < 0) ? ((-B - discSqrt) / 2.f) : ((-B + discSqrt) / 2.f);

    double t0 = q / A;
    f64 t1 = -B / A - t0;

    if (t0 > t1)
    {
      double temp = t0;
      t0 = t1;
      t1 = temp;
    }

    float resT;
    float flipn = 1.f;

    if (t0 > ray.min_t && t0 < isec.dist_&&t0<ray.max_t&&t0>isec.epsilon_)
      resT = float(t0);
    else if (t1 > ray.min_t && t1 < isec.dist_&&t1<ray.max_t&&t1>isec.epsilon_)
    {
      flipn = -1.f;
      resT = float(t1);
    }
    else
      return false;

    isec.dist_ = resT;
    isec.normal_ = (transformedOrigin + RBVector3(resT) * ray.d).get_normalized();
    isec.shading_normal_ = (transformedOrigin + RBVector3(resT) * ray.d).get_normalized();
    isec.g_ = this;
    isec.p_ = ray.o + resT*ray.d;
    //ad-hoc
    RBVector3 local_p = isec.p_ - o_;
    f32 phi = RBMath::atant2(local_p.x, local_p.y);
    if (phi < 0)
      phi += PI * 2;
    isec.uv.x = phi*INV_2PI;
    f32 theta = RBMath::acos(RBMath::clamp(local_p.z / r_, -1.f, 1.f));
    isec.uv.y = theta*INV_PI;
    ///

    return true;
  }

  virtual bool intersectP(const OLPBRRay_& ray) const
  {
    // we transform ray origin into object space (center == origin)
    RBVector3 transformedOrigin = ray.o - o_;

    ft A = RBVector3::dot_product(ray.d, ray.d);
    ft B = 2 * RBVector3::dot_product(ray.d, transformedOrigin);
    ft C = RBVector3::dot_product(transformedOrigin, transformedOrigin) - (r_ * r_);

    // Must use doubles, because when B ~ sqrt(B*B - 4*A*C)
    // the resulting t is imprecise enough to get around ray epsilons
    const double disc = B*B - 4 * A*C;

    if (disc < 0)
      return false;

    double discSqrt = RBMath::sqrt(disc);
    double q = (B < 0) ? ((-B - discSqrt) / 2.f) : ((-B + discSqrt) / 2.f);

    double t0 = q / A;
    f64 t1 = -B / A - t0;

    if (t0 > t1)
    {
      double temp = t0;
      t0 = t1;
      t1 = temp;
    }



    if (t0 > ray.min_t && t0 < ray.max_t)
      return true;
    if (t1 > ray.min_t && t1 < ray.max_t)
      return true;


    return false;
  }
  virtual void calculate_area()
  {
    area = 4 * PI*r_*r_;
  }
  virtual RBAABB bound()
  {
    if (!bound_.is_available())
    {
      RBVector3 minv = o_ - RBVector3(r_, r_, r_);
      RBVector3 maxv = o_ + RBVector3(r_, r_, r_);
      bound_.include(maxv);
      bound_.include(minv);
      bound_.fix_thin();
    }
    return bound_;

  }

  RBVector3 o_;
  ft r_;
};

#include <algorithm>
#include <fstream>

#define USE_LINEAR_MEM


class OLPBRKDTreeNode_
{
public:
  OLPBRKDTreeNode_(bool val) :leaf(val), left(nullptr), right(nullptr), root(false), state(-1){}
  void init_as_leaf(std::vector<int>& index);

  std::vector<int> index;
  OLPBRKDTreeNode_* left;
  OLPBRKDTreeNode_* right;
  RBAABB bound;
  int split_axis;
  float split_pos;

  //debug
  int depth;
  int state;
  //序列化根节点标记
  bool root;
  bool leaf;
};

class OLPBRKDTree_
{
public:
  OLPBRKDTree_(int smxn=8):
    isc_t(80), trav_t(1), eb(0.5), leaf_n(0), single_max_n(8), root(nullptr)
  {
    
  }
  //节点内存
  ThreadMem linear_mem;
  OLPBRKDTreeNode_* root;
  class OLPBRKDTreeNodeWrite
  {
  public:
    //std::vector<int> index;,64bit release vector is 24 bytes.
    //but cpp does not specilize the size of a vector! 
    //this design is highly dangerous!
    struct
    {
      u64 size;
      //相对于vec_mem的偏移
      u64 offset;
      u64 pad;
#ifdef _DEBUG
      u64 pad1;
#endif

    } index;
    u64 left_offset;
    u64 right_offset;
    RBAABB bound;
    int split_axis;
    float split_pos;

    //debug
    int depth;
    int state;
    //序列化根节点标记
    bool root;
    bool leaf;

  };
  struct HeadWrite
  {
    //总尺寸
    u64 total_size;
    //单个节点大小
    u64 single_block_size;
    //vector偏移量
    u64 vector_offset;
    //节点个数
    u64 node_num;

  };

  //默认100M
  void init_mem(size_t size = (1 << 20) * 100)
  {
    linear_mem.frame.shutdown();
    char s[100];
    sprintf(s,"kd tree allcator for class [%d]",(size_t)this);
    linear_mem.frame.init(size, s);
    linear_mem.frame.getframe(linear_mem.mf, false);
  }

  OLPBRKDTreeNode_* alloc_node(bool leaf)
  {
    void* p = linear_mem.frame.alloc(sizeof(OLPBRKDTreeNode_), false);
    return new(p)OLPBRKDTreeNode_(leaf);
  }

  //释放帧，并没有真的释放内存
  //!!不会调用结点析构函数!!
  void release_frame()
  {
    release_index();
    //!!不会调用析构函数!!
    linear_mem.frame.release(linear_mem.mf);
  }

  //彻底释放内存
  //todo：当前只支持一颗kdtree
  void free_mem()
  {
    release_index();
    linear_mem.frame.shutdown();
  }

  //释放节点之前需要先释放叶节点的vector
  //可能需要自行实现vector替代是个更好的方案
  void release_index()
  {
    size_t block_size = linear_mem.frame.get_alloc_size(sizeof(OLPBRKDTreeNode_));
    f32 tt = (f32)linear_mem.frame.get_allocated_memory() / block_size;
    int block_num = (int)tt;
    for (int k = 0; k < block_num; ++k)
    {
      OLPBRKDTreeNode_* node = ((OLPBRKDTreeNode_*)&linear_mem.mf.memory_ptr[k*block_size]);
      if (node->leaf)
      {
        node->index.swap(std::vector<int>());
      }
    }
  }

  //序列化kdtree
  void serialize(const char* filename)
  {
    size_t block_size = linear_mem.frame.get_alloc_size(sizeof(OLPBRKDTreeNode_));
    f32 tt = (f32)linear_mem.frame.get_allocated_memory() / block_size;
    int block_num = (int)tt;
    printf("serialize %f nodes!\n node size:%d\ntotal size:%d\n", tt, block_size, linear_mem.frame.get_allocated_memory());

    size_t node_size = linear_mem.frame.get_allocated_memory();
    size_t total_size = 0;
    for (int k = 0; k < block_num; ++k)
    {
      OLPBRKDTreeNode_* node = ((OLPBRKDTreeNode_*)&linear_mem.mf.memory_ptr[k*block_size]);
      total_size += node->index.size()*sizeof(int);
    }

    size_t head_size = sizeof(HeadWrite);

    char* mem = new char[head_size + node_size + total_size];
    HeadWrite* head_mem = (HeadWrite*)mem;
    char* node_mem = &mem[head_size];
    char* vec_mem = &mem[node_size + head_size];

    head_mem->node_num = block_num;
    head_mem->single_block_size = block_size;
    head_mem->total_size = head_size + node_size + total_size;
    head_mem->vector_offset = node_size + head_size;

    if (sizeof(OLPBRKDTreeNodeWrite) != sizeof(OLPBRKDTreeNode_))
      printf("write:%d~origin:%d\n", sizeof(OLPBRKDTreeNodeWrite), sizeof(OLPBRKDTreeNode_));
    CHECK(sizeof(OLPBRKDTreeNodeWrite) == sizeof(OLPBRKDTreeNode_));
    CHECK(sizeof(OLPBRKDTreeNodeWrite) == block_size);

    OLPBRKDTreeNodeWrite* cur_save_node = (OLPBRKDTreeNodeWrite*)node_mem;
    size_t cur_vec_offset = 0;

    for (int k = 0; k < block_num; ++k)
    {
      OLPBRKDTreeNode_* node = ((OLPBRKDTreeNode_*)&linear_mem.mf.memory_ptr[k*block_size]);

      memcpy(cur_save_node, node, block_size);
      size_t vsize = node->index.size()*sizeof(int);
      cur_save_node->index.size = vsize;
      cur_save_node->index.offset = cur_vec_offset;
      cur_save_node->index.pad = 0;
#ifdef _DEBUG
      cur_save_node->index.pad1 = 0;
#endif
      memcpy(vec_mem + cur_vec_offset, node->index.data(), vsize);
      cur_vec_offset += vsize;


      if (node->left)
      {
        cur_save_node->left_offset = (u64)((u8*)node->left - (u8*)linear_mem.mf.memory_ptr);
        //if (!node->left)
        //printf("sss");
      }
      else
        cur_save_node->left_offset = 0xffffffffffffffff;
      if (node->right)
      {
        cur_save_node->right_offset = (u64)((u8*)node->right - (u8*)linear_mem.mf.memory_ptr);
        //if (!node->right)
        //printf("sss");
      }
      else
        cur_save_node->right_offset = 0xffffffffffffffff;

      cur_save_node++;

    }

    //OLPBRKDTreeNodeWrite* o = OLPBRKDTreeNode*

    std::ofstream fout(filename, std::ios::binary);
    fout.write((char*)mem, head_size + node_size + total_size);
    fout.close();
    delete[] mem;

  }

  void* deserialize(const char* filename)
  {
    release_frame();


    std::ifstream fin(filename, std::ios::binary);
    if (!fin)
    {
      printf("read %s failed!\n", filename);
      return nullptr;
    }
    fin.seekg(0, fin.end);
    int read_size = fin.tellg();
    fin.seekg(0, fin.beg);
    char* mem = new char[read_size];
    fin.read(mem, read_size);
    fin.close();

    HeadWrite* head = (HeadWrite*)mem;
    u64 node_num = head->node_num;
    u64 block_size = head->single_block_size;
    u64 total_szie = head->total_size;
    u64 vector_off = head->vector_offset;
    CHECK(total_szie == read_size);
    char* vec_mem = &mem[vector_off];
    OLPBRKDTreeNodeWrite* cur_read_node = (OLPBRKDTreeNodeWrite*)&mem[sizeof(HeadWrite)];

    void* p = linear_mem.frame.alloc(block_size*node_num, false);
    memcpy(p, cur_read_node, block_size*node_num);
    OLPBRKDTreeNode_* node = (OLPBRKDTreeNode_*)p;
    OLPBRKDTreeNode_* ret = nullptr;

    for (int k = 0; k < node_num; ++k)
    {
      u32 icount = cur_read_node->index.size / sizeof(int);
      u32 ioff = cur_read_node->index.offset;
      //std::vector<int> v;
      //OLPBRKDTreeNode* node = (OLPBRKDTreeNode*)p;
      new(&node->index) std::vector<int>();
      //memcpy(&node->index, &v, sizeof(std::vector<int>));
      node->index.resize(icount);
      memcpy((node->index.data()), vec_mem + ioff, icount*sizeof(int));
      ret = node->root ? node : ret;

      if (cur_read_node->left_offset != 0xffffffffffffffff)
        node->left = (OLPBRKDTreeNode_*)((u64)cur_read_node->left_offset + (u64)p);
      else
        node->left = nullptr;
      if (cur_read_node->right_offset != 0xffffffffffffffff)
        node->right = (OLPBRKDTreeNode_*)((u64)cur_read_node->right_offset + (u64)p);
      else
        node->right = nullptr;

      cur_read_node++;
      node++;
    }
    delete[] mem;
    return ret;

  }

  struct split_point_t
  {
    split_point_t(int pi, bool lh, float dis) :prim_index(pi), low(lh), point(dis){}
    int prim_index;
    bool low;
    float point;
    bool operator<(const split_point_t& o)
    {
      if (point == o.point)
        return (int)o.low<(int)(low);
      else
        return point < o.point;
    }
  };

  struct kd_todo_t
  {
    kd_todo_t(){}
    kd_todo_t(OLPBRKDTreeNode_* anode, float mint, float maxt) :node(anode), tmin(mint), tmax(maxt){}
    const OLPBRKDTreeNode_* node;
    float tmin, tmax;
  };

  bool intersection_res(const OLPBRRay_& ray, const OLPBRKDTreeNode_* node, OLPBRItersc_* isc, const std::vector<OLPBRGeometry_*>& prims)
  {

    if (!node)
    {
      return false;
    }

    float tmin, tmax;
    if (!node->bound.intersection(ray.o, ray.d, tmin, tmax))
      return false;
    if (node->leaf)
    {
      bool ret = false;
      for (int i = 0; i < node->index.size(); ++i)
      {
        ret |= prims[node->index[i]]->intersect(ray, *isc);
      }
      return ret;
    }
    int axis = node->split_axis;
    RBVector3 inv_dir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
    OLPBRKDTreeNode_* first, *second;
    float tplane = (node->split_pos - ray.o[axis])*inv_dir[axis];
    if (inv_dir[axis] > 0)
    {
      if (tplane <= 0)
      {
        first = node->right;
        second = nullptr;
      }
      else
      {
        first = node->left;
        second = node->right;
      }
    }
    else
    {
      if (tplane <= 0)
      {
        first = node->left;
        second = nullptr;
      }
      else
      {
        first = node->right;
        second = node->left;
      }
    }


    //blow for debug
    // if (first || second)
    {

      bool a = intersection_res(ray, first, isc, prims);
      //如果包含了交点在aabb中才算，预防相交的交点不再本aabb中的情形
      if (a)
      {
        RBVector3 sp = ray.o + ray.d*(isc->dist_ - isc->epsilon_);
        /*
        |    |   __B_     |
        | A  |   |~~|     |
        |____|___|__|___C_|
        光线首先穿过Bound A与C平面相交，此时返回true，然而这个交点可能不是最近的，甚至可能根本不在Bound A中！
        只要检测到的最近点一定满足：宿主object一定与A有交；一定是最近的点。
        */
        if (first->bound.is_contain(sp))
          return a;
      }
      //todo:if already had the result stop tracing
      bool b = intersection_res(ray, second, isc, prims);
      return a || b;
    }

    //printf("kdtree intersc should not to be here!%d,%d\n",first,second);
    //return false;
  }

  bool intersection(const OLPBRRay_& ray, OLPBRItersc_ *isc, const std::vector<OLPBRGeometry_*>& prims)
  {
    float tmin, tmax;
    if (!root) return false;
    if (!root->bound.intersection(ray.o, ray.d, tmin, tmax))
    {
      return false;
    }
    CHECK(tmin >= 0);
    RBVector3 inv_dir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);

#define MAX_TODON 64
    kd_todo_t todo[MAX_TODON];
    int todo_pos = 0;

    bool hit = false;
    const OLPBRKDTreeNode_*node = this->root;
    while (node != nullptr || todo_pos != 0)
    {

      if (ray.max_t<tmin) break;
      //有可能某个节点是空节点，但是此时栈中依然有节点需要处理
      if (!node)
      {
        --todo_pos;
        node = todo[todo_pos].node;
        tmin = todo[todo_pos].tmin;
        tmax = todo[todo_pos].tmax;
        //sd->draw_sphere(ray.o + ray.d*tmax, 0.5);
      }
      if (!node)
        continue;
      if (!node->leaf)
      {
        int axis = node->split_axis;
        float tplane = (node->split_pos - ray.o[axis])*inv_dir[axis];

        const OLPBRKDTreeNode_* first_child, *second_child;
        int below_first = ((ray.o[axis]<node->split_pos) ||
          (ray.o[axis] == node->split_pos&&ray.d[axis] <= 0));
        if (below_first)
        {
          first_child = node->left;
          second_child = node->right;
        }
        else
        {
          first_child = node->right;
          second_child = node->left;
        }

        if (tplane>tmax || tplane <= 0)
        {
          node = first_child;
        }
        else if (tplane<tmin)
        {
          node = second_child;
        }
        else
        {
          todo[todo_pos].node = second_child;
          todo[todo_pos].tmin = tplane;
          todo[todo_pos].tmax = tmax;
          ++todo_pos;
          node = first_child;
          tmax = tplane;
        }
      }
      else
      {
        //相交测试叶节点
        int nprim = node->index.size();
        if (nprim == 1)
        {
          //叶节点只有一个primitive
          if (prims[node->index[0]]->intersect(ray, *isc))
          {
            hit = true;
          }

        }
        else
        {
          //叶节点不止一个primitive
          //for

          for (auto obj_i : node->index)
          {
            hit |= prims[obj_i]->intersect(ray, *isc);
          }

        }


        if (todo_pos>0)
        {
          --todo_pos;
          node = todo[todo_pos].node;
          tmin = todo[todo_pos].tmin;
          tmax = todo[todo_pos].tmax;
          //sd->draw_sphere(ray.o + ray.d*tmax, 0.5);

        }
        else
          break;
      }
    }

    return hit;
  }

  //bd当前节点的AABB，prim图元储存实际储存，cur_index当前节点所属图元的index
  void build_kdtree(const RBAABB& bd, const std::vector<OLPBRGeometry_*>& prim,
    std::vector<int>& cur_index, int depth, OLPBRKDTreeNode_*& toparent)
  {
    if (cur_index.size()<1)
    {
      toparent = nullptr;
      return;
    }

    if (depth == 0 || cur_index.size()<single_max_n)
    {

#ifdef USE_LINEAR_MEM
      toparent = alloc_node(true);
#else
      toparent = new OLPBRKDTreeNode(true);
#endif

      toparent->init_as_leaf(cur_index);
      for (int k = 0; k < cur_index.size(); k++)
      {
        prim[cur_index[k]]->debug_tag += 1;
      }
      toparent->bound = bd;
      toparent->depth = 0;
      toparent->split_axis = -1;
      toparent->split_pos = -1.f;
      leaf_n++;
      if (!toparent)
        printf("new failed!\n");
      return;
    }

    int axis = 0;

    std::vector<int> oa;
    std::vector<int> ob;
    f32 best_point;
    int best_aixs = -1;
    RBAABB bda, bdb;

    {
      //std::vector<int> flags;
      float old_cost = MAX_F32;

      int best_index = -1;
      float inv_ta = 1.f / bd.get_surface_area();
      auto d = bd.max - bd.min;

#define SAH
      std::vector<split_point_t> tobesplitx;
      {
        //x

        for (int i = 0; i<cur_index.size(); ++i)
        {
          auto bdi = prim[cur_index[i]]->bound();
          tobesplitx.push_back(split_point_t(cur_index[i], true, bdi.min.x - bd.min.x));
          tobesplitx.push_back(split_point_t(cur_index[i], false, bdi.max.x - bd.min.x));
        }
        if (tobesplitx.size()>0)
          std::sort(tobesplitx.begin(), tobesplitx.end());
        f32 aa = 0;
        f32 ab = 0;
        int na = 0, nb = tobesplitx.size()*0.5;
        for (int i = 0; i<tobesplitx.size(); ++i)
        {
          if (tobesplitx[i].low == false) nb--;
          aa = ((d.y*d.z) + tobesplitx[i].point*(d.z + d.y)) * 2;
          ab = ((d.y*d.z) + (d.x - tobesplitx[i].point)*(d.z + d.y)) * 2;
#ifdef SAH
          f32 pa = aa * inv_ta;
          f32 pb = ab * inv_ta;
#else
          f32 pa = tobesplitx[i].point / d.x;// aa * inv_ta;
          f32 pb = 1 - pa;//ab * inv_ta;
#endif
          f32 eb_ = (na == 0 || nb == 0) ? eb : 0.f;
          eb_ = 0;
          f32 cost = trav_t + isc_t*(1.f - eb_)*(pa*na + pb*nb);

          if (cost<old_cost)
          {
            old_cost = cost;
            best_aixs = 0;
            best_index = i;
          }

          if (tobesplitx[i].low == true) na++;
        }
      }
      std::vector<split_point_t> tobesplity;
      {
        //y

        for (int i = 0; i<cur_index.size(); ++i)
        {
          auto bdi = prim[cur_index[i]]->bound();
          tobesplity.push_back(split_point_t(cur_index[i], true, bdi.min.y - bd.min.y));
          tobesplity.push_back(split_point_t(cur_index[i], false, bdi.max.y - bd.min.y));
        }
        if (tobesplity.size()>0)
          std::sort(tobesplity.begin(), tobesplity.end());
        f32 aa = 0;
        f32 ab = 0;
        int na = 0, nb = tobesplity.size()*0.5;
        for (int i = 0; i<tobesplity.size(); ++i)
        {
          if (tobesplity[i].low == false) nb--;
          aa = ((d.x*d.z) + tobesplity[i].point*(d.z + d.x)) * 2;
          ab = ((d.x*d.z) + (d.y - tobesplity[i].point)*(d.z + d.x)) * 2;
#ifdef SAH
          f32 pa = aa * inv_ta;
          f32 pb = ab * inv_ta;
#else
          f32 pa = tobesplity[i].point / d.y;//aa * inv_ta;
          f32 pb = 1 - pa;//ab * inv_ta;
#endif
          f32 eb_ = (na == 0 || nb == 0) ? eb : 0.f;
          eb_ = 0;
          f32 cost = trav_t + isc_t*(1.f - eb_)*(pa*na + pb*nb);

          if (cost<old_cost)
          {
            old_cost = cost;
            best_aixs = 1;
            best_index = i;
          }

          if (tobesplity[i].low == true) na++;
        }
      }
      std::vector<split_point_t> tobesplitz;
      {
        //z

        for (int i = 0; i<cur_index.size(); ++i)
        {
          auto bdi = prim[cur_index[i]]->bound();
          tobesplitz.push_back(split_point_t(cur_index[i], true, bdi.min.z - bd.min.z));
          tobesplitz.push_back(split_point_t(cur_index[i], false, bdi.max.z - bd.min.z));
        }
        if (tobesplitz.size()>0)
          std::sort(tobesplitz.begin(), tobesplitz.end());
        f32 aa = 0;
        f32 ab = 0;
        int na = 0, nb = tobesplitz.size()*0.5;
        for (int i = 0; i<tobesplitz.size(); ++i)
        {
          if (tobesplitz[i].low == false) nb--;
          aa = ((d.y*d.x) + tobesplitz[i].point*(d.x + d.y)) * 2;
          ab = ((d.y*d.x) + (d.z - tobesplitz[i].point)*(d.x + d.y)) * 2;
#ifdef SAH
          f32 pa = aa * inv_ta;
          f32 pb = ab * inv_ta;
#else
          f32 pa = tobesplitz[i].point / d.z;//aa * inv_ta;
          f32 pb = 1 - pa;//ab * inv_ta;
#endif
          f32 eb_ = (na == 0 || nb == 0) ? eb : 0.f;
          eb_ = 0;
          f32 cost = trav_t + isc_t*(1.f - eb_)*(pa*na + pb*nb);

          if (cost<old_cost)
          {
            old_cost = cost;
            best_aixs = 2;
            best_index = i;
          }

          if (tobesplitz[i].low == true) na++;
        }
      }


      f32 t;

      switch (best_aixs)
      {
      case 0:
        for (int i = 0; i <= best_index; ++i)
          if (tobesplitx[i].low)
            oa.push_back(tobesplitx[i].prim_index);
        for (int i = best_index + 1; i<tobesplitx.size(); ++i)
          if (!tobesplitx[i].low)
            ob.push_back(tobesplitx[i].prim_index);
        //
        if ((ob.size() + oa.size()) < tobesplitx.size() / 2)
        {
          oa.clear();
          ob.clear();
          for (int i = 0; i<best_index; ++i)
            if (tobesplitx[i].low)
              oa.push_back(tobesplitx[i].prim_index);
          for (int i = best_index + 1; i<tobesplitx.size(); ++i)
            if (!tobesplitx[i].low)
              ob.push_back(tobesplitx[i].prim_index);
        }
        //
        t = tobesplitx[best_index].point / d.x;
        /*
        if (oa.size() == 0)
        printf("");
        */
        bd.split(0, t, bda, bdb);
        best_point = tobesplitx[best_index].point;
        break;
      case 1:
        for (int i = 0; i <= best_index; ++i)
          if (tobesplity[i].low)
            oa.push_back(tobesplity[i].prim_index);
        for (int i = best_index + 1; i<tobesplity.size(); ++i)
          if (!tobesplity[i].low)
            ob.push_back(tobesplity[i].prim_index);
        //
        if ((ob.size() + oa.size()) < tobesplity.size() / 2)
        {
          oa.clear();
          ob.clear();
          for (int i = 0; i<best_index; ++i)
            if (tobesplity[i].low)
              oa.push_back(tobesplity[i].prim_index);
          for (int i = best_index + 1; i<tobesplity.size(); ++i)
            if (!tobesplity[i].low)
              ob.push_back(tobesplity[i].prim_index);
        }
        //
        t = tobesplity[best_index].point / d.y;
        bd.split(1, t, bda, bdb);
        best_point = tobesplity[best_index].point;
        break;
      case 2:
        for (int i = 0; i <= best_index; ++i)
          if (tobesplitz[i].low)
            oa.push_back(tobesplitz[i].prim_index);
        for (int i = best_index + 1; i<tobesplitz.size(); ++i)
          if (!tobesplitz[i].low)
            ob.push_back(tobesplitz[i].prim_index);
        if ((ob.size() + oa.size()) < tobesplitz.size() / 2)
        {
          oa.clear();
          ob.clear();
          for (int i = 0; i<best_index; ++i)
            if (tobesplitz[i].low)
              oa.push_back(tobesplitz[i].prim_index);
          for (int i = best_index + 1; i<tobesplitz.size(); ++i)
            if (!tobesplitz[i].low)
              ob.push_back(tobesplitz[i].prim_index);
        }

        t = tobesplitz[best_index].point / d.z;
        bd.split(2, t, bda, bdb);
        best_point = tobesplitz[best_index].point;
        break;
      default:

        break;
      }
      tobesplitz.swap(std::vector<split_point_t>());
      tobesplity.swap(std::vector<split_point_t>());
      tobesplitx.swap(std::vector<split_point_t>());

    }


    OLPBRKDTreeNode_* a, *b;

#if 1
    if (bda.max[best_aixs] - bda.min[best_aixs] <= SMALL_F)
    {
      a = nullptr;
#ifdef USE_LINEAR_MEM
      b = alloc_node(true);
#else
      b = new OLPBRKDTreeNode(true);
#endif
      b->init_as_leaf(cur_index);
      for (int k = 0; k < cur_index.size(); k++)
      {
        prim[cur_index[k]]->debug_tag += 1;
      }
      b->bound = bd;
      b->depth = 0;
      b->split_axis = -1;
      b->split_pos = -1.f;
      leaf_n++;
      if (!b)
        printf("new failed!\n");
    }
    else
      if (bdb.max[best_aixs] - bdb.min[best_aixs] <= SMALL_F)
      {
        b = nullptr;
#ifdef USE_LINEAR_MEM
        a = alloc_node(true);
#else
        a = new OLPBRKDTreeNode(true);
#endif
        a->init_as_leaf(cur_index);
        for (int k = 0; k < cur_index.size(); k++)
        {
          prim[cur_index[k]]->debug_tag += 1;
        }
        a->bound = bd;
        a->depth = 0;
        a->split_axis = -1;
        a->split_pos = -1.f;
        leaf_n++;
        if (!a)
          printf("new failed!\n");
      }

      else
#endif
      {
        /*
        if (oa.size() == 0)
        printf("");
        */
        build_kdtree(bda, prim, oa, depth - 1, a);
        build_kdtree(bdb, prim, ob, depth - 1, b);
        oa.swap(std::vector<int>());
        ob.swap(std::vector<int>());

      }

#ifdef USE_LINEAR_MEM
    toparent = alloc_node(false);
#else
    toparent = new OLPBRKDTreeNode(false);
#endif
    if (!toparent)
      printf("new failed!\n");
    if (bda.max[axis] - bda.min[axis] <= SMALL_F)
    {
      toparent->state = 9999;
    }
    toparent->bound = bd;
    toparent->left = a;
    toparent->right = b;
    toparent->depth = depth;
    toparent->split_axis = best_aixs;
    toparent->split_pos = best_point + bd.min[best_aixs];

  }

  //std::vector<int> nodes;
  float isc_t;
  float trav_t;
  float eb;
  int single_max_n;

  int leaf_n;
};




#undef ft