//static const float rayIncrement = g_Truncation;
//static const float g_thresSampleDist = 1.5f*rayIncrement;
//static const float g_thresDist = 1.5f*rayIncrement;

//颜色表, 不加static const就错。
static const float4 colorTable[16] = {
	float4(0.53f, 0.81f, 1.00f, 0.0f),
    float4(1.00f, 0.08f, 0.58f, 0.0f),
    float4(0.00f, 0.00f, 1.00f, 0.0f),
    float4(0.00f, 1.00f, 0.00f, 0.0f),
    float4(1.00f, 1.00f, 0.00f, 0.0f),
    float4(1.00f, 0.00f, 0.00f, 0.0f),
    float4(0.63f, 0.13f, 0.94f, 0.0f),
    float4(0.00f, 0.00f, 0.00f, 0.0f),
    float4(0.93f, 0.68f, 0.05f, 0.0f),
    float4(1.00f, 0.50f, 0.14f, 0.0f),
    float4(1.00f, 0.71f, 0.77f, 0.0f),
    float4(0.00f, 0.55f, 0.55f, 0.0f),
    float4(0.55f, 0.49f, 0.48f, 0.0f),
    float4(1.00f, 0.87f, 0.68f, 0.0f),
    float4(0.42f, 0.56f, 0.14f, 0.0f),
    float4(0.00f, 1.00f, 1.00f, 0.0f)
};


float3 gradientForPoint(float3 pos)
{
	float3 offset = g_VirtualVoxelSize;

	float distp00; float3 colorp00; trilinearInterpolationSimpleFastFast(pos-float3(0.5f*offset.x, 0.0f, 0.0f), distp00, colorp00);
	float dist0p0; float3 color0p0; trilinearInterpolationSimpleFastFast(pos-float3(0.0f, 0.5f*offset.y, 0.0f), dist0p0, color0p0);
	float dist00p; float3 color00p; trilinearInterpolationSimpleFastFast(pos-float3(0.0f, 0.0f, 0.5f*offset.z), dist00p, color00p);

	float dist100; float3 color100; trilinearInterpolationSimpleFastFast(pos+float3(0.5f*offset.x, 0.0f, 0.0f), dist100, color100);
	float dist010; float3 color010; trilinearInterpolationSimpleFastFast(pos+float3(0.0f, 0.5f*offset.y, 0.0f), dist010, color010);
	float dist001; float3 color001; trilinearInterpolationSimpleFastFast(pos+float3(0.0f, 0.0f, 0.5f*offset.z), dist001, color001);

	float3 grad = float3((distp00-dist100)/offset.x, (dist0p0-dist010)/offset.y, (dist00p-dist001)/offset.z);

	float l = length(grad);
	if(l == 0.0f)
	{
		return float3(0.0f, 0.0f, 0.0f);
	}

	return -grad/l;
}

/*void traverseCoarseGridSimpleSampleAll(float3 worldCamPos, float3 worldDir, float3 camDir, int3 dTid)
{
	// Last Sample
	Sample lastSample; lastSample.sdf = 0.0f; lastSample.alpha = 0.0f; lastSample.weight = 0; lastSample.color = int3(0, 0, 0);
	const float depthToRayLength = 1.0f/camDir.z; // scale factor to convert from depth to ray length

	int2 screenPos = cameraToKinectScreenInt(camDir);
	float rayCurrent = depthToRayLength*max(g_SensorDepthWorldMin, kinectProjZToCamera(g_RayIntervalMin[screenPos])); // Convert depth to raylength // depthToRayLength
	float rayEnd = depthToRayLength*min(g_SensorDepthWorldMax, kinectProjZToCamera(g_RayIntervalMax[screenPos])); // Convert depth to raylength
	//rayCurrent = depthToRayLength*g_SensorDepthWorldMin;
	//rayEnd = depthToRayLength*g_SensorDepthWorldMax;
	//const float rayIncrement = depthToRayLength*g_Truncation/2.0f;

	[allow_uav_condition]
	while(rayCurrent < rayEnd)
	{
		float3 currentPosWorld = worldCamPos+rayCurrent*worldDir;
		HashEntry entry = getHashEntryForSDFBlockPos(g_Hash, worldToSDFBlock(currentPosWorld));
		
		if(entry.ptr != FREE_ENTRY)
		{
			float dist;	float3 color;
			if(trilinearInterpolationSimpleFastFast(currentPosWorld, dist, color))
			{
				if(lastSample.weight > 0 && lastSample.sdf > 0.0f && dist < 0.0f) // current sample is always valid here
				{					
					float alpha = findIntersectionLinear(lastSample.alpha, rayCurrent, lastSample.sdf, dist);
					//findIntersectionBisection(worldCamPos, worldDir, lastSample.sdf, lastSample.alpha, dist, rayCurrent, alpha);
					
					float3 currentIso = worldCamPos+alpha*worldDir;
					if(abs(lastSample.sdf - dist) < g_thresSampleDist)// && trilinearInterpolationSimpleFastFast(currentIso, dist, color)) // sets dist and color to new values
					{
						if(abs(dist) < g_thresDist)
						{
							g_output[dTid.xy] = alpha/depthToRayLength; // Convert ray length to depth depthToRayLength
							g_outputColor[dTid.xy] = float4(color/255.0f, 1.0f);

							if(g_useGradients == 1)
							{
								float3 normal = gradientForPoint(currentIso);
								g_outputNormals[dTid.xy] = float4(mul(float4(normal,0.0f), g_ViewMat).xyz, 1.0f);
							}

							return;
						}
					}
				}

				lastSample.sdf = dist;
				lastSample.alpha = rayCurrent;
				lastSample.color = color;
				lastSample.weight = 1;
			}
			else
			{
				lastSample.weight = 0;
			}
		}

		rayCurrent+=rayIncrement;
	}
}
*/

void traverseCoarseGridSimpleSampleAllMultiLayer(float3 worldCamPos, float3 worldDir, float3 camDir, int3 dTid)
{
	const float depthToRayLength = 1.0f/camDir.z; // scale factor to convert from depth to ray length
	int index1D = dTid.y*g_RenderTargetWidth+dTid.x;

	int startIndex = 0;
	if(index1D > 0) startIndex = g_FragmentPrefixSumBufferSRV[index1D-1];
	int endIndex = g_FragmentPrefixSumBufferSRV[index1D]-1;
	int currentIndex = startIndex;
	
	float rayCurrent = g_SensorDepthWorldMin;

	// Last Sample
	Sample lastSample; lastSample.sdf = 0.0f; lastSample.alpha = 0.0f; lastSample.weight = 0; // lastSample.color = int3(0, 0, 0);

	[allow_uav_condition]
	while(currentIndex <= endIndex)
	{
		float rayStart = depthToRayLength*kinectProjZToCamera(g_FragmentSortedDepthBufferSRV[currentIndex]);
		rayCurrent = max(rayStart, rayCurrent);
				
		[allow_uav_condition]
		while(rayCurrent <= rayStart+1.74f*SDF_BLOCK_SIZE*g_VirtualVoxelSize+rayIncrement) // 1.74f > sqrt(3)
		{
			float3 currentPosWorld = worldCamPos+rayCurrent*worldDir;
			float dist;	float3 color;
			if(trilinearInterpolationSimpleFastFast(currentPosWorld, dist, color))
			{
				if(lastSample.weight > 0 && lastSample.sdf > 0.0f && dist < 0.0f) // current sample is always valid here
				{
					float alpha; // = findIntersectionLinear(lastSample.alpha, rayCurrent, lastSample.sdf, dist);
					findIntersectionBisection(worldCamPos, worldDir, lastSample.sdf, lastSample.alpha, dist, rayCurrent, alpha);
					
					float3 currentIso = worldCamPos+alpha*worldDir;
					if(abs(lastSample.sdf - dist) < g_thresSampleDist)
					{
						if(abs(dist) < g_thresDist)
						{
							g_output[dTid.xy] = alpha/depthToRayLength; // Convert ray length to depth depthToRayLength
							g_outputColor[dTid.xy] = float4(color, 1.0f);

							if(g_useGradients == 1)
							{
								float3 normal = gradientForPoint(currentIso);
								g_outputNormals[dTid.xy] = float4(mul(float4(normal,0.0f), g_ViewMat).xyz, 1.0f);
							}

							return;
						}
					}
				}

				lastSample.sdf = dist;
				lastSample.alpha = rayCurrent;
				// lastSample.color = color;
				lastSample.weight = 1;
			}
			else
			{
				lastSample.weight = 0;
			}
			
			rayCurrent+=rayIncrement;
		}

		currentIndex++;
	}
}

void SetVoxelIDByPointCloud()
{
	//////遍历所有pointcloud中的点，找出距离当前raycasting的点最近的位置
	int nelements_for_each_point = 5;
	int nPoints = g_PCXYZID[0]; //取出第一个字节，得到点云的数量
	/*float nearest_distance = MAXF;
	int nearest_patch_id = -1;*/

	//int *PointBID = new int[nPoints](-1);全局已经分配，在RaycastingHashSF.hlsl中
	//int *BIDArray = new int[nPoints](-1);
	for(int i = 0; i < nPoints; i++){
		float3 pt;
		pt.x = g_PCXYZID[1 + nelements_for_each_point * i + 0];
		pt.y = g_PCXYZID[1 + nelements_for_each_point * i + 1];
		pt.z = g_PCXYZID[1 + nelements_for_each_point * i + 2];
		//PointBID[i] = linearizeIndex(worldToSDFBlock(pt);
		int3 tmp = worldToSDFBlock(pt);
		int tmp2 = linearizeIndex(tmp);
		PointBID[i] = tmp2;
	}

	int nBID = 0;
	for(int i = 0; i < nPoints; i++)
	{
		int j;
		for (j = 0; j < nBID; j++) 
		{
			if (BIDArray[j] == PointBID[i]) //BIDArray默认是-1
			{
				break;
			}
		}

		if (j == nBID) { // not found
			BIDArray[nBID] = PointBID[i];
			nBID++;
		}
	}

	for(int i = 0; i < nBID; i++){
		HashEntry entry = getHashEntryForSDFBlockPos(g_Hash, BIDArray[i]);

		if(entry.ptr != FREE_ENTRY)
		{
			int3 pi_base = SDFBlockToVirtualVoxelPos(entry.pos);
			for (int j = 0; j < 512; j++) {
				int3 pi = pi_base + delinearizeVoxelIndex(j);
				float3 pf = virtualVoxelPosToWorld(pi);
				float min_dist = 100000000.0;
				int cpi = -1;

				for(int k = 0; k < nPoints; k++){
					if (PointBID[k] == BIDArray[i]) {
						float3 pt;
						pt.x = g_PCXYZID[1 + nelements_for_each_point * k + 0];
						pt.y = g_PCXYZID[1 + nelements_for_each_point * k + 1];
						pt.z = g_PCXYZID[1 + nelements_for_each_point * k + 2];
						float dist = (pf.x-pt.x)*(pf.x-pt.x) + (pf.y-pt.y)*(pf.y-pt.y) + (pf.z-pt.z)*(pf.z-pt.z);
						if (dist < min_dist) {
							min_dist = dist;
							cpi = k;
						}
					}
				}

				if (cpi != -1) {
					Voxel	v = getVoxel(g_SDFBlocksSDF, g_SDFBlocksRGBW, j);
					v.color.x = g_PCXYZID[1 + nelements_for_each_point * cpi + 3];
					//setVoxel(g_SDFBlocksSDF, g_SDFBlocksRGBW, j, v);
				}
			}
		}
	}
}

void traverseCoarseGridSimpleSampleAll(float3 worldCamPos, float3 worldDir, float3 camDir, int3 dTid)
{
	// Last Sample
	Sample lastSample; lastSample.sdf = 0.0f; lastSample.alpha = 0.0f; lastSample.weight = 0; // lastSample.color = int3(0, 0, 0);
	const float depthToRayLength = 1.0f/camDir.z; // scale factor to convert from depth to ray length

	float rayCurrent = depthToRayLength*max(g_SensorDepthWorldMin, kinectProjZToCamera(g_RayIntervalMin[dTid.xy])); // Convert depth to raylength
	float rayEnd = depthToRayLength*min(g_SensorDepthWorldMax, kinectProjZToCamera(g_RayIntervalMax[dTid.xy])); // Convert depth to raylength
 
	[allow_uav_condition]
	while(rayCurrent < rayEnd)
	{
		float3 currentPosWorld = worldCamPos+rayCurrent*worldDir;
		HashEntry entry = getHashEntryForSDFBlockPos(g_Hash, worldToSDFBlock(currentPosWorld));
		float dist;	float3 color;
		if(trilinearInterpolationSimpleFastFast(currentPosWorld, dist, color))
		//if(trilinearInterpolation(currentPosWorld, dist, color))
		{
			if(lastSample.weight > 0 && lastSample.sdf > 0.0f && dist < 0.0f) // current sample is always valid here
			{					
				float alpha; // = findIntersectionLinear(lastSample.alpha, rayCurrent, lastSample.sdf, dist);
				findIntersectionBisection(worldCamPos, worldDir, lastSample.sdf, lastSample.alpha, dist, rayCurrent, alpha);
					
				float3 currentIso = worldCamPos+alpha*worldDir;
				if(abs(lastSample.sdf - dist) < g_thresSampleDist)
				{
					if(abs(dist) < g_thresDist)
					{
						////遍历所有pointcloud中的点，找出距离当前raycasting的点最近的位置
						//int nelements_for_each_point = 5;
						//int nPoints = g_PCXYZID[0]; //取出第一个字节，得到点云的数量
						//float nearest_distance = MAXF;
						//int nearest_patch_id = -1;

						//for(int i = 0; i < nPoints; i++){
						//	float3 pt;
						//	pt.x = g_PCXYZID[1 + nelements_for_each_point * i + 0];
						//	pt.y = g_PCXYZID[1 + nelements_for_each_point * i + 1];
						//	pt.z = g_PCXYZID[1 + nelements_for_each_point * i + 2];

						//	int cur_patch_id = g_PCXYZID[1 + nelements_for_each_point * i + 3];
						//	
						//	double cur_dist = distance(pt, currentPosWorld);//distance为内置函数
						//	if(cur_dist < nearest_distance){
						//		nearest_distance = cur_dist;
						//		nearest_patch_id = cur_patch_id;
						//	}
						//}

						//if(nearest_patch_id >=0 && nearest_patch_id < 16){
						//	g_outputColor[dTid.xy] = colorTable[nearest_patch_id];
						//}else{
						//	g_outputColor[dTid.xy] = float4(1.0f, 0.0f, 0.0f, 1.0f);
						//}

						g_outputColor[dTid.xy] = float4(1.0f, 0.0f, 0.0f, 1.0f);

						g_output[dTid.xy] = alpha/depthToRayLength; // Convert ray length to depth depthToRayLength
						

						if(g_useGradients == 1)
						{
							float3 normal = gradientForPoint(currentIso);
							g_outputNormals[dTid.xy] = float4(mul(float4(normal,0.0f), g_ViewMat).xyz, 1.0f);
						}
						return;
					}
				}
			}

			lastSample.sdf = dist;
			lastSample.alpha = rayCurrent;
			// lastSample.color = color;
			lastSample.weight = 1;
			rayCurrent+=rayIncrement;
		}
		else
		{
			lastSample.weight = 0;
			rayCurrent+=rayIncrement;
		}

		//rayCurrent+=rayIncrement; //0.6f*getTruncation(rayCurrent);
	}
}