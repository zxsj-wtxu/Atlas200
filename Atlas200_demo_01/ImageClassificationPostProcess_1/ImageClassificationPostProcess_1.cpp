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
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] start init!");
    if (postprocess_config_ == nullptr)
    {
        postprocess_config_ = std::make_shared<PostprocessConfig>();
    }
    for (int index = 0; index < config.items_size(); ++index)
    {
        const ::hiai::AIConfigItem& item = config.items(index);
        std::string name = item.name();
        if (name == "path") {
            postprocess_config_->path = item.value();
        } else if (name == "output_name") {
            postprocess_config_->output_node = item.value();
        }
    }

    std::string info_file_ = GetInfoFilePath(postprocess_config_->path);
    id_img_correlation.clear();
    id_img_correlation = SetImgPredictionCorrelation(info_file_,".txt");
    if (id_img_correlation.empty()) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] scan info file failed!");
        return HIAI_ERROR;
    }

    uint32_t graph_id = Engine::GetGraphId();
    std::shared_ptr<Graph> graph = Graph::GetInstance(graph_id);
    if (nullptr == graph)
    {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Fail to get the graph");
        return HIAI_ERROR;
    }
    std::ostringstream deviceId;
    deviceId << graph->GetDeviceID();
    string device_dir = RESULT_FOLDER + "/" + deviceId.str();
    store_path = device_dir + "/" + ENGINE_NAME;
    if (HIAI_OK != CreateFolder(RESULT_FOLDER, PERMISSION)) {
        return HIAI_ERROR;
    }
    if (HIAI_OK != CreateFolder(device_dir, PERMISSION)) {
        return HIAI_ERROR;
    }
    if (HIAI_OK != CreateFolder(store_path, PERMISSION)) {
        return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] end init!");
    return HIAI_OK;
}

/**
* @brief: send sentinel image to inform the graph to destroy
*/
HIAI_StatusT ImageClassificationPostProcess_1::SendSentinelImage() {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] sentinel image, process success!");
    std::shared_ptr<std::string> result_data(new std::string);
    HIAI_StatusT hiai_ret = HIAI_OK;
    do {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] sentinel image, process success!");
        hiai_ret = SendData(0, "string", std::static_pointer_cast<void>(result_data));
        if (HIAI_OK != hiai_ret) {
            if (HIAI_ENGINE_NULL_POINTER == hiai_ret || HIAI_HDC_SEND_MSG_ERROR == hiai_ret || HIAI_HDC_SEND_ERROR == hiai_ret
                    || HIAI_GRAPH_SRC_PORT_NOT_EXIST == hiai_ret || HIAI_GRAPH_ENGINE_NOT_EXIST == hiai_ret) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] SendData error[%d], break.", hiai_ret);
                break;
            }
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] SendData return value[%d] not OK, sleep 200ms", hiai_ret);
            usleep(SEND_DATA_INTERVAL_MS);
        }
    } while (HIAI_OK != hiai_ret);
    return hiai_ret;
}

HIAI_IMPL_ENGINE_PROCESS("ImageClassificationPostProcess_1", ImageClassificationPostProcess_1, INPUT_SIZE)
{
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] start process!");

    if (nullptr == arg0)
    {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] fail to process invalid message");
        return HIAI_ERROR;
    }
    std::shared_ptr<EngineTransT> tran = std::static_pointer_cast<EngineTransT>(arg0);
    std::vector<OutputT> output_data_vec = tran->output_data_vec;
    //add sentinel image for showing this data in dataset are all sended, this is last step.
    BatchImageParaWithScaleT image_handle = { tran->b_info, tran->v_img };
    if (isSentinelImage(std::make_shared<BatchImageParaWithScaleT>(image_handle)))
    {
        return SendSentinelImage();
    }
    if (!tran->status)
    {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, tran->msg.c_str());
        return HIAI_ERROR;
    }
    bool validate_name = false;
    OutputT out;
    for (unsigned int i = 0; i< output_data_vec.size(); i++) {
        out = output_data_vec[i];
        std::string index;
        std::string cur_name;
        GetLayerName(out.name, index, cur_name);
        std::string match_name = postprocess_config_->output_node;
        if (match_name == cur_name) {
            validate_name = true;
            break; //here assume that the classification network only has one output
        }
    }
    if (validate_name == false) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1]: the output node name doesn't exist");
        return HIAI_ERROR;
    }

    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1]: %s", out.name.c_str());
    int size = out.size / sizeof(float);
    if(size <= 0){
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1]: the OutPutT size less than 0!");
        return HIAI_ERROR;
    }
    float* result = nullptr;
    try{
        result = new float[size];
    }
     catch (const std::bad_alloc& e) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1]:failed to allocate result!");
        return HIAI_ERROR;
    }
    int ret = memcpy_s(result, sizeof(float)*size, out.data.get(), sizeof(float)*size);
    if(ret != 0){
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1]: memcpy_s out data error!");
        delete[] result;
        return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "run to postprocess: size %d, batch_size %d", size, tran->b_info.batch_size);

    std::vector<uint32_t> frame_ID = tran->b_info.frame_ID;
    for (unsigned int ind = 0; ind < tran->b_info.batch_size; ind++)
    {
        if ((int)frame_ID[ind] == -1) {
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] image number %d failed, add empty struct", ind);
            continue;
        }
        ImageInfor img_infor = id_img_correlation[frame_ID[ind]];
        CropInfo crop_info = tran->v_img[ind].crop_info;

        std::string img_infor_tfilename = store_path + "/" + img_infor.tfilename;
        int fd = open(img_infor_tfilename.c_str(), O_CREAT| O_WRONLY|O_APPEND, S_IRUSR|S_IWUSR);
        int ret = 0;
        if(fd == -1){
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] failed to open result file!");
            delete[] result;
            return HIAI_ERROR;
        }
        printf("[ImageClassificationPostProcess_1] result file open successfully\n");
        std::string img_infor0 = "";
        img_infor0 = img_infor0 + to_string(img_infor.format) + " ";
        img_infor0 = img_infor0 + to_string(img_infor.width) + " ";
        img_infor0 = img_infor0 + to_string(img_infor.height) + " ";
        img_infor0 = img_infor0 + to_string(crop_info.point_x) + " ";
        img_infor0 = img_infor0 + to_string(crop_info.point_y) + " ";
        img_infor0 = img_infor0 + to_string(crop_info.crop_width) + " ";
        img_infor0 = img_infor0 + to_string(crop_info.crop_height) + " ";
        ret = write(fd, img_infor0.c_str(), img_infor0.length());
        if(ret == -1){
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] write file error!");
            ret = close(fd);
            if(ret == -1){
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] close file error!");
            }
            delete[] result;
            return HIAI_ERROR;
        }
        for (unsigned int k = 0; k < (size / tran->b_info.max_batch_size); k++){
            std::string img_infor1 = "";
            img_infor1 = img_infor1 + to_string(result[(size / tran->b_info.max_batch_size)*ind + k]) + " ";
            ret = write(fd, img_infor1.c_str(), img_infor1.length());

            if(ret == -1){
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] write file error!");
                ret = close(fd);
                if(ret == -1){
                    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] close file error!");
                }
                delete[] result;
                return HIAI_ERROR;
            }
        }
        ret = write(fd, "\n", 1);
        if(ret == -1){
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] write file error!");
            ret = close(fd);
            if(ret == -1){
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] close file error!");
            }
            delete[] result;
            return HIAI_ERROR;
        }
        ret = close(fd);
        if(ret == -1){
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImageClassificationPostProcess_1] close file error!");
            delete[] result;
            return HIAI_ERROR;
        }
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] end process!");
    delete[] result;
    return HIAI_OK;
}
