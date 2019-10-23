/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-19
*/
#include "ImageNet100.h"
#include "hiaiengine/log.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/ai_memory.h"
#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <time.h>
#include <map>

const static std::string SIMULATOR_LOCAL = "Simulator_local";
const static std::string OI = "OI";
const int DVPP_BUFFER_ALIGN_SIZE = 128;


HIAI_StatusT ImageNet100::Init(const hiai::AIConfig& config, const  std::vector<hiai::AIModelDescription>& model_desc)
{
    return HIAI_OK;
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS : Overloading Engine Process processing logic
* @[in]: Define an input port, an output port
*/
HIAI_IMPL_ENGINE_PROCESS("ImageNet100", ImageNet100, INPUT_SIZE)
{
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageNet100] start process!");
    std::shared_ptr<std::string> temp = std::static_pointer_cast<string>(arg0);
    if(temp != nullptr){
        std::cout << "arg0:" <<*temp <<std::endl;
    }
    std::shared_ptr<std::string> temp1 = std::static_pointer_cast<string>(arg1);
    if(temp1 != nullptr){
        std::cout << "arg1:" <<*temp1 <<std::endl;
    }
    std::shared_ptr<std::string> temp2 = std::static_pointer_cast<string>(arg2);
    if(temp2 != nullptr){
        std::cout << "arg2:" <<*temp2 <<std::endl;
    }

    std::shared_ptr<std::string> temp_ptr = std::make_shared<std::string>("image net 100");
    SendData(DEFAULT_DATA_PORT, "string", std::static_pointer_cast<void>(temp_ptr));

    std::cout << "[ImageNet100] send sentinel image!" <<endl;
    std::cout << "[ImageNet100] end process!" <<endl;
    return 0;
}
