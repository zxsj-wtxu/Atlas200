/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-30
*/
#include "ImageClassificationPostProcess_1.h"
#include <hiaiengine/log.h>
#include <vector>
#include <unistd.h>
#include <thread>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>
#include <sstream>
#include <fcntl.h>


HIAI_StatusT ImageClassificationPostProcess_1::Init(const hiai::AIConfig& config, const  std::vector<hiai::AIModelDescription>& model_desc)
{
    return HIAI_OK;
}

HIAI_IMPL_ENGINE_PROCESS("ImageClassificationPostProcess_1", ImageClassificationPostProcess_1, INPUT_SIZE)
{
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] start process!");

    if (nullptr != arg0) {
        std::shared_ptr<std::string> ptr = std::static_pointer_cast<std::string>(arg0);
        std::cout << "arg0 post process:" << *ptr << std::endl;
    }
    if (nullptr != arg1) {
        std::shared_ptr<std::string> ptr = std::static_pointer_cast<std::string>(arg1);
        std::cout << "arg1 post process:" << *ptr << std::endl;
    }
    //add sentinel image for showing this data in dataset are all sended, this is last step.
    std::shared_ptr<std::string> result_data(new std::string);
    SendData(0, "string", std::static_pointer_cast<void>(result_data));
    return HIAI_OK;
}
