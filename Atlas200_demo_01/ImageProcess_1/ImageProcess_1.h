/*******
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-19
*/
#ifndef ImageProcess_1_ENGINE_H_
#define ImageProcess_1_ENGINE_H_
#include "hiaiengine/engine.h"
#include "hiaiengine/multitype_queue.h"
#include <iostream>
#include <string>
#include <dirent.h>
#include <memory>
#include <unistd.h>
#include <vector>
#include <stdint.h>
#define INPUT_SIZE 16
#define OUTPUT_SIZE 1
using hiai::Engine;
using namespace std;
using namespace hiai;
class ImageProcess_1 : public Engine {
public:
    ImageProcess_1() :
        input_que_(INPUT_SIZE) {}
    HIAI_StatusT Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc);
    /**
    * @ingroup hiaiengine
    * @brief HIAI_DEFINE_PROCESS : reload Engine Process
    * @[in]: define the number of input and output
    */
    HIAI_DEFINE_PROCESS(INPUT_SIZE, OUTPUT_SIZE)
private:

    // Private implementation a member variable, which is used to cache the input queue
    hiai::MultiTypeQueue input_que_;
};

#endif