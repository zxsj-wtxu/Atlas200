/**
* @file ImagePreProcess_1.cpp
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-4-25
*/

#include "ImagePreProcess_1.h"
#include <math.h>
#include <sstream>
#include <ctime>
#include "hiaiengine/log.h"
#include "hiaiengine/ai_types.h"
#include "hiaiengine/c_graph.h"
#include "hiaiengine/ai_memory.h"
#include "custom/toolchain/ide_daemon_api.h"

const static int SEND_DATA_SLEEP_MS = 100000;
const static int DVPP_SUPPORT_MAX_WIDTH = 4096;
const static int DVPP_SUPPORT_MIN_WIDTH = 16;
const static int DVPP_SUPPORT_MAX_HEIGHT = 4096;
const static int DVPP_SUPPORT_MIN_HEIGHT = 16;

#define CHECK_ODD(NUM)      (((NUM) % 2 != 0) ? (NUM) : ((NUM) - 1))
#define CHECK_EVEN(NUM)     (((NUM) % 2 == 0) ? (NUM) : ((NUM) - 1))


ImagePreProcess_1::~ImagePreProcess_1()
{
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1]ImagePreProcess_1 Engine Destory");
}

HIAI_StatusT ImagePreProcess_1::Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& modelDesc)
{
    std::stringstream ss;
    for (int index = 0; index < config.items_size(); ++index)
    {
        const ::hiai::AIConfigItem& item = config.items(index);
        std::string name = item.name();
        ss << item.value();
        if (name == "id") {
           ss >> id;
           std::cout << "id:" <<id <<std::endl;
        }
    }
    return HIAI_OK;
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS: realize ImagePreProcess_1
* @[in]: define a input and output,
*        and register Engine named ImagePreProcess_1
*/
HIAI_IMPL_ENGINE_PROCESS("ImagePreProcess_1", ImagePreProcess_1, INPUT_SIZE)
{
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1] start process!");
    int errCode = HIAI_OK;
    std::shared_ptr<BatchImageParaWithScaleT> errorHandler = NULL;

    if(arg0 != nullptr){
        std::shared_ptr<std::string> ptr = std::static_pointer_cast<std::string>(arg0);
        std::cout << "pre process:" << *ptr << std::endl;
    }
    std::shared_ptr<std::string> temp_ptr = std::make_shared<std::string>("image pre process ");
    //send null to next node to avoid blocking when to encounter abnomal situation.
    SendData(0, "string", std::static_pointer_cast<void>(temp_ptr));
    return HIAI_ERROR;
}

