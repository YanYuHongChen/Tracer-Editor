#include "Editor.h"
#include "EditorModel.h"
#include "App.h"

bool EditorApp::preinit()
{
  return true;
}

bool EditorApp::init()
{
  _rhi = static_cast<FD3D11DynamicRHI*>(base_app->get_rhi());
  _d3d11_renderer = new Direct3D11Renderer(_rhi->GetDevice(), _rhi->GetDeviceContext());


  scene_view = new EditorSceneView();
  scene_view->init(_d3d11_renderer, back_buffer_view);
  scene_view->load_sys_assets();
  scene_view->create_rhires(ww,wh);
  scene_view->init_d3d11(ww,wh);
  scene_view->init_assistant_object();
  return true;

}

void EditorApp::update(f32 dt)
{
  if(scene_view->gui_update(dt))
    return;
  scene_view->update(dt);
}

void EditorApp::draw()
{
  scene_view->render();
}

void EditorApp::ter()
{
  delete _d3d11_renderer;
  delete scene_view;
}

void EditorApp::postter()
{

}

EditorApp::~EditorApp()
{

}

EditorApp::EditorApp(class AppBase* app) :FastApp(app)
{

}
