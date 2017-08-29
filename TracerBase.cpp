#include "TracerBase.h"

void OLPBRKDTreeNode_::init_as_leaf(std::vector<int>& index1)
{
  leaf = true;
  index.clear();
  index.resize(index1.size());
  memcpy(&index[0], &index1[0], index1.size()*sizeof(int));
}
