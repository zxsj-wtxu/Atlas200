/*******
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-19
*/
#include "ImageProcess_1.h"
#include "hiaiengine/log.h"
#include "hiaiengine/data_type_reg.h"
#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS : implementaion of the engine
* @[in]: engine name and the number of input
*/
HIAI_StatusT ImageProcess_1::Init(const hiai::AIConfig &config,
    const std::vector<hiai::AIModelDescription> &model_desc)
{
    //TODO:
    return HIAI_OK;
}

HIAI_IMPL_ENGINE_PROCESS("ImageProcess_1", ImageProcess_1, INPUT_SIZE)
{
    //TODO:
	//user code here
    return HIAI_OK;
}