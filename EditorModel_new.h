#pragma once
#if 0
#include "WIPObject.h"
#include <vector>
#include "Vector3.h"
#include "Vector2.h"
#include "Matrix.h"
#include "RHID3D11.h"

class ObjectInstanceSet
{
public:
  //²ã´Îobject
  vector<MeshInstance*> sub_object;
  RBMatrix world_2_local;
  RBMatrix local_2_world;
};

class EditorModel_new : public WIPObject
{
public:

};
#endif