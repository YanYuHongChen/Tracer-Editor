#include "Editor.h"
#include "EditorModel.h"
#include "App.h"
#include "UIWrap.h"

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


  //test event
  imbt = new IMButton("caonim");
  cb = new IMCheckBox("nimei", true);
  a = new A();
  a1 = new A();
  b = new B();
  main_bar = new IMMainMenuBar();
  main_bar->add_menu_item("File", "Open..");
  main_bar->add_menu_item("File", "Save..");
  main_bar->add_menu_item("File", "Import..");
  main_bar->add_menu_item("Tools", "Material Editor");
  main_bar->add_menu_item("Tools", "Rendering Setting");
  main_bar->add_menu_item("Tools", "Camera Setting");
  main_bar->add_menu_item("Help", "Help");
  main_bar->add_menu_item("Debug", "Debug Tools");

  a->subscribe_event(cb, get_string_hash("check_box_change"), WIP_EVENT_HANDLER_OUT(A, check_box, a));
  cb->signal_for_init();
  a->subscribe_event(imbt, get_string_hash("button_push"), WIP_EVENT_HANDLER_OUT(A, push, a), 2);
  a1->subscribe_event(imbt, get_string_hash("button_push"), WIP_EVENT_HANDLER_OUT(A, push, a1), 2);
  b->subscribe_event(imbt, get_string_hash("button_push"), WIP_EVENT_HANDLER_OUT(B, push1, b), 1);

  b->subscribe_event(main_bar, get_string_hash("File.Open.."), WIP_EVENT_HANDLER_OUT(B, handle_menu, b));
  b->subscribe_event(main_bar, get_string_hash("File.Save.."), WIP_EVENT_HANDLER_OUT(B, handle_menu, b));
  b->subscribe_event(main_bar, get_string_hash("File.Import.."), WIP_EVENT_HANDLER_OUT(B, handle_menu, b));
  b->subscribe_event(main_bar, get_string_hash("Tools.Rendering Setting"), WIP_EVENT_HANDLER_OUT(B, handle_menu, b));

  component_update = get_string_hash("MapComponent Update");

  imbt->subscribe_event(this, component_update, WIP_EVENT_HANDLER_OUT(IMButton, update, imbt));
  main_bar->subscribe_event(this, component_update, WIP_EVENT_HANDLER_OUT(IMMainMenuBar, update, main_bar));
  cb->subscribe_event(this, component_update, WIP_EVENT_HANDLER_OUT(IMCheckBox, update, cb));

  return true;

}

void EditorApp::update(f32 dt)
{
  //send_event(component_update);
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
