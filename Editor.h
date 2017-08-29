#pragma once
#include "FastApp.h"
#include "RHID3D11.h"
#include "D3D11FastRenderer.h"
#include <vector>

class EditorApp : public FastApp
{
public:
  virtual bool preinit() override;
  virtual bool init() override;
  virtual void update(f32 dt) override;
  virtual void draw() override;
  virtual void ter() override;
  virtual void postter() override;

  EditorApp(class AppBase* app);
  virtual ~EditorApp();
  EditorApp(const EditorApp& o) = delete;
  EditorApp& operator=(const EditorApp& o) = delete;

protected:

  class EditorSceneView* scene_view;



  void load_models_generate_buffer(const char* filename);
  void load_assets();
  void create_rhires();
  void handle_input(f32 dt);
  void proccess_imgui(f32 dt);
  void resize(int w, int h, bool bfullscreen){}

  FD3D11DynamicRHI* _rhi;
  Direct3D11Renderer* _d3d11_renderer;
};
