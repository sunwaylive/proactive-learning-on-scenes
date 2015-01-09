#include "stdafx.h"

#include "VoxelUtilHashSDF.h"


//wei add, 添加id数据域之后， 每个元素变成3个字节
Voxel VoxelUtilHelper::getVoxel(const SDFBlock& sdfBlocks, unsigned int id)
{
	Voxel voxel;

	//float* f = ((float*)(&(sdfBlocks.data[0])))[2*id+0]; voxel.sdf = *f;
	voxel.sdf = ((float*)&(sdfBlocks.data[0]))[3 * id + 0];

	int last = sdfBlocks.data[3 * id + 1];
	voxel.weight = last & 0x000000ff;
	last >>= 0x8;
	voxel.color.x = last & 0x000000ff;
	last >>= 0x8;
	voxel.color.y = last & 0x000000ff;
	last >>= 0x8;
	voxel.color.z = last & 0x000000ff;

	int patch_id = sdfBlocks.data[3 * id + 2];
	voxel.id = patch_id;

	return voxel;
}

//添加了id后，sdfBlock中每个元素由2个字节变成3个字节
void VoxelUtilHelper::setVoxel(SDFBlock& sdfBlocks, unsigned int id, const Voxel& voxel)
{
	unsigned int* f = (unsigned int*)&voxel.sdf; 
	sdfBlocks.data[3*id+0] = *f;

	int last = 0;
	last |= voxel.color.z & 0x000000ff;
	last <<= 8;
	last |= voxel.color.y & 0x000000ff;
	last <<= 8;
	last |= voxel.color.x & 0x000000ff;
	last <<= 8;
	last |= voxel.weight & 0x000000ff;
	sdfBlocks.data[3*id+1] = last;

	int patchi_id = voxel.id;
	sdfBlocks.data[3 * id + 2] = patchi_id;
}
				
vec3i VoxelUtilHelper::SDFBlockToVirtualVoxelPos(vec3i sdfBlock)
{
	return sdfBlock * SDF_BLOCK_SIZE;
}

vec3i VoxelUtilHelper::worldToVirtualVoxelPos(vec3f pos)
{
	const vec3f p = pos/GlobalAppState::getInstance().s_virtualVoxelSize;
	vec3f s;
	s.x = (float)math::sign(p.x);
	s.y = (float)math::sign(p.y);
	s.z = (float)math::sign(p.z);
	return (vec3i)(p+s*0.5f);
}
		
vec3i VoxelUtilHelper::virtualVoxelPosToSDFBlock(vec3i virtualVoxelPos)
{
	if (virtualVoxelPos.x < 0) virtualVoxelPos.x -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.y < 0) virtualVoxelPos.y -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.z < 0) virtualVoxelPos.z -= SDF_BLOCK_SIZE-1;

	return virtualVoxelPos/SDF_BLOCK_SIZE;
}

vec3f VoxelUtilHelper::virtualVoxelPosToWorld(vec3i pos)
{
	return vec3f(pos) * GlobalAppState::getInstance().s_virtualVoxelSize;
}

vec3f VoxelUtilHelper::SDFBlockToWorld(vec3i sdfBlock)
{
	return virtualVoxelPosToWorld(SDFBlockToVirtualVoxelPos(sdfBlock));
}

vec3f VoxelUtilHelper::computeSDFBlockCenter(vec3i sdfBlock)
{
	vec3f posWorld = SDFBlockToWorld(sdfBlock);
	const float centerOffset(0.5f*(SDF_BLOCK_SIZE-1)*GlobalAppState::getInstance().s_virtualVoxelSize);

	posWorld.x += centerOffset;
	posWorld.y += centerOffset;
	posWorld.z += centerOffset;

	return posWorld;
}

vec3i VoxelUtilHelper::worldToSDFBlock(vec3f worldPos)
{
	const vec3i virtualVoxelPos = worldToVirtualVoxelPos(worldPos);
	return virtualVoxelPosToSDFBlock(virtualVoxelPos);
}

vec3i VoxelUtilHelper::delinearlize(unsigned int idx)
{
  unsigned int x = idx % SDF_BLOCK_SIZE;
  unsigned int y = (idx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
  unsigned int z = idx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);	
  return vec3ui(x,y,z);
}

vec3i VoxelUtilHelper::getColorBySDF(float sdf_value, float sdf_min, float sdf_max)
{
  if(abs(sdf_value - 100) < 1e-4){
    return vec3i(0, 0, 0);//如果是默认的sdf值，即里面没有sdf voxel的话，返回黑色
  }

  int tmp = static_cast<int>((sdf_value - sdf_min) / (sdf_max - sdf_min) * 255);
  float r = g_color_table[tmp][0];
  float g = g_color_table[tmp][1];
  float b = g_color_table[tmp][2];
  return vec3i(int(r * 255), int(g * 255), int(b * 255));
}