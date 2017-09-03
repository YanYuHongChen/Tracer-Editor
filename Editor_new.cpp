#include "Editor_new.h"
#include "EditorModel_new.h"
#include "EditorView_new.h"
#include "App.h"
#include "UIWrap.h"
#if 0
bool EditorApp_new::preinit()
{
  return true;
}

bool EditorApp_new::init()
{
  _rhi = static_cast<FD3D11DynamicRHI*>(base_app->get_rhi());
  _d3d11_renderer = new Direct3D11Renderer(_rhi->GetDevice(), _rhi->GetDeviceContext());

  return true;

}

void EditorApp_new::update(f32 dt)
{
  //if(scene_view->gui_update(dt))
  //return;
  //scene_view->update(dt);
}

void EditorApp_new::draw()
{

}

void EditorApp_new::ter()
{
  delete scene_view;
  delete scene_model;
  delete _d3d11_renderer;
}

void EditorApp_new::postter()
{

}

EditorApp_new::~EditorApp_new()
{

}

EditorApp_new::EditorApp_new(class AppBase* app) :FastApp(app)
{

}
#endif