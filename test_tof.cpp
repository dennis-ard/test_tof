#include "ArduCamTOFCamera.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <sqlite3.h>
#include <ctime>

#define MAX_DISTANCE 4
sqlite3 *sql = NULL;

float differ=0.2;
cv::Scalar green(83, 168,  52);
cv::Scalar orange(42,147,252);

void execut_statement(sqlite3 *sql,const char *sql_stmt){
    sqlite3_stmt *stmt = NULL;
    int result = sqlite3_prepare_v2(sql, sql_stmt, -1, &stmt, NULL);
 
    if (result == SQLITE_OK) {
        // std::clog<< "statement ok"<<std::endl;
        //执行该语句
        sqlite3_step(stmt);
    }
    else {
        std::clog << "Problem executing statement";
    }
    //清理语句句柄，准备执行下一个语句
    sqlite3_finalize(stmt);
}

std::string convertFloat2String(const float value,const int precision = 2){
    std::stringstream stream{};
    stream<<std::fixed<<std::setprecision(precision)<<value;
    return stream.str();
}

bool readConfigFile(const char *cfgFilePath,const std::string &key,std::string &value){
    std::fstream cfgFile;
    cfgFile.open(cfgFilePath);
    if(!cfgFile.is_open()){
        std::cerr<<"can not open file!"<<std::endl;
        return false;
    }
    char tmp[80];
    while(!cfgFile.eof()){
        cfgFile.getline(tmp,80);
        std::string line(tmp);
        auto pos = line.find('=');
        if(pos== std::string::npos) return false;
        std::string tmpKey = line.substr(0, pos);
        if(key==tmpKey){
            value = line.substr(pos+1);
            return true;
        }
    }
    return false;
}

bool getConfig(const std::string &key,float &value){
    std::string tmpValue;
    if(readConfigFile("./test.ini",key,tmpValue)){
        try{
            value = std::stof(tmpValue);
            return true;
        }catch(...){
            std::cerr<<"配置的值有问题"<<std::endl;
        }
    }else{
        std::cout<<"没找到配置项:"<< key <<std::endl;
    }
    return false;
}

void display_fps(void)
{
    static int count = 0;
    ++count;
    static std::chrono::high_resolution_clock::time_point time_beg = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::ratio<1, 1>> duration_s = time_end - time_beg;
    if (duration_s.count() >= 1)
    {
        std::cout << "fps:" << count << std::endl;
        count = 0;
        time_beg = time_end;
    }
}

void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr)
{
    auto len = 240 * 180;
    for (auto i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(phase_image_ptr + i) / MAX_DISTANCE)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}

void drawResult(cv::Mat &result_frame,float nowValue,float standValue,cv::Rect _rect,cv::Point _point){
    if(std::abs(nowValue - standValue) > differ){
        cv::rectangle(result_frame, _rect, orange, 2);
        cv::putText(result_frame, convertFloat2String(nowValue),_point,cv::FONT_HERSHEY_COMPLEX,0.3,orange);
    }else{
        cv::rectangle(result_frame, _rect, green, 2);
        cv::putText(result_frame, convertFloat2String(nowValue),_point,cv::FONT_HERSHEY_COMPLEX,0.3,green);
    }
}

const uint8_t frame_row = 180;
const uint8_t frame_col = 240;

int main()
{   
    float cfgTopLeft,cfgTopRight,cfgMiddle,cfgBottomLeft,cfgBottomRight;
    float test2modcfg[5]={0};
    if(!getConfig("differ",differ)||!getConfig("topLeft",cfgTopLeft)||!getConfig("topRight",cfgTopRight)||!getConfig("middle",cfgMiddle)||!getConfig("bottomLeft",cfgBottomLeft)||!getConfig("bottomRight",cfgBottomRight)) return false;

    ArduCam::ArduCamTOFCamera tof;
    ArduCam::FrameBuffer *frame;
    
    if (tof.init(ArduCam::CSI,ArduCam::DEPTH_TYPE)){
        std::cerr<<"initialization failed"<<std::endl;
        exit(-1);
    }

    if (tof.start()){
        std::cerr<<"Failed to start camera"<<std::endl;
        exit(-1);
    }
    //  Modify the range also to modify the MAX_DISTANCE
    tof.setControl(ArduCam::RANGE,MAX_DISTANCE);

    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[43200];
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);

    int rect = 16;
    cv::Rect topLeftRect(0, 0, rect, rect);
    cv::Rect topRightRect(240-rect, 0, rect, rect);
    cv::Rect middleRect(120-(rect/2), 90-(rect/2), rect, rect);
    cv::Rect bottomLeftRect(0, 180-rect, rect, rect);
    cv::Rect bottomRightRect(240-rect, 180-rect, rect, rect); 

    cv::Point topLeftPoint(2,25);
    cv::Point topRightPoint(210,25);
    cv::Point middlePoint(118,78);
    cv::Point bottomLeftPoint(2,158);
    cv::Point bottomRightPoint(210,158);
    float record[5][3]={4.0,0,0,
                        4.0,0,0,
                        4.0,0,0,
                        4.0,0,0,
                        4.0,0,0,};
    // float tmp[5];
    float cfg[6]={cfgTopLeft,cfgTopRight,cfgMiddle,cfgBottomLeft,cfgBottomRight};
    cv::Rect rectArray[5] = {topLeftRect, topRightRect, middleRect, bottomLeftRect,bottomRightRect};
    cv::Point pointArray[5] = {topLeftPoint, topRightPoint, middlePoint, bottomLeftPoint, bottomRightPoint};
    for (uint8_t i = 0; i < 100; i ++)
    {
	do{
        frame = tof.requestFrame(200);
        }while (frame == nullptr);
        {
            depth_ptr = (float *)frame->getData(ArduCam::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getData(ArduCam::AMPLITUDE_FRAME);
            getPreview(preview_ptr, depth_ptr, amplitude_ptr);

            cv::Mat depth_frame(frame_row, frame_col, CV_32F, depth_ptr);
            cv::Mat amplitude_frame(frame_row, frame_col, CV_32F, amplitude_ptr);
            cv::Mat result_frame(frame_row, frame_col, CV_8U, preview_ptr);

            cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);

            amplitude_frame.forEach<float>([](float &p, const int *position) -> void
                                           {   
                if(p < 0) p=0;
                else if(p > 255) p=255; });
            amplitude_frame.convertTo(amplitude_frame, CV_8U);
            cv::imshow("amplitude", amplitude_frame);

            // drawResult(result_frame,cv::mean(depth_frame(topLeftRect)).val[0],cfgTopRight,topLeftRect,topLeftPoint);
            // drawResult(result_frame,cv::mean(depth_frame(topRightRect)).val[0],cfgTopLeft,topRightRect,topRightPoint);
            // drawResult(result_frame,cv::mean(depth_frame(middleRect)).val[0],cfgMiddle,middleRect,middlePoint);
            // drawResult(result_frame,cv::mean(depth_frame(bottomLeftRect)).val[0],cfgBottomLeft,bottomLeftRect,bottomLeftPoint);
            // drawResult(result_frame,cv::mean(depth_frame(bottomRightRect)).val[0],cfgBottomRight,bottomRightRect,bottomRightPoint);
            for(uint8_t j = 0; j< 5;j++){
                float tmp = cv::mean(depth_frame(rectArray[j])).val[0];
                drawResult(result_frame,tmp,cfg[j],rectArray[j],pointArray[j]);
                if(record[j][0] > tmp) record[j][0]=tmp;
                else if (record[j][1] < tmp) record[j][1]=tmp;
                record[j][2] += tmp;
            }
            cv::resize(result_frame, result_frame, cv::Size(960, 720));
            // std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;

            cv::imshow("preview", result_frame);

            if (cv::waitKey(1) == 27)
                break;
            display_fps();
        }
        tof.releaseFrame(frame);
    }

    char path[80];

    std::time_t t = std::time(0);
    tm *nowtime = localtime(&t);
    sprintf(path,"test_tof_%d_%d.db",1900+nowtime->tm_year,nowtime->tm_mon+1);

    int result = sqlite3_open_v2(path,&sql,SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE|SQLITE_OPEN_NOMUTEX|SQLITE_OPEN_SHAREDCACHE,NULL);
    if(result == SQLITE_OK){
       // std::clog << "open db success" << std::endl;
    }else {
        std::clog << "open db failure"<<std::endl;
    }

    const char *sqlCreateTable ="create table if not exists test_tof (tof_id integer PRIMARY KEY AUTOINCREMENT,point_position integer not NULL,eligibility INTEGER not null,test_time INTEGER NOT NULL,min_value integer not null,max_value integer not null,avg_value integer not null);";
    execut_statement(sql,sqlCreateTable);
    char buf[300];
    for(uint8_t j = 0;j < 5; j++){
        record[j][2] /= 100;
        sprintf(buf,"INSERT INTO test_tof(point_position,eligibility,test_time, min_value,max_value,avg_value) VALUES(%d,%d,%ld, %f,%f,%f);",j,std::abs(record[j][2]-cfg[j])>differ?0:1,t,record[j][0],record[j][1],record[j][2]);
        execut_statement(sql,buf);
        std::cout<<"位置:"<< (int)j <<"偏差值:"<<std::abs(record[j][2]-cfg[j])<<", 最小值 : "<< record[j][0]<<", 最大值: "<<record[j][1]<<", 平均值: "<<record[j][2]<<std::endl;
    }

    if (sql) {
        sqlite3_close_v2(sql);
        sql = nullptr;
    }
        int record_cnt = 0;
        do{
            frame = tof.requestFrame(200);
        }while (frame == nullptr);
        
        depth_ptr = (float *)frame->getData(ArduCam::DEPTH_FRAME);
        amplitude_ptr = (float *)frame->getData(ArduCam::AMPLITUDE_FRAME);
        getPreview(preview_ptr, depth_ptr, amplitude_ptr);

        cv::Mat depth_frame(frame_row, frame_col, CV_32F, depth_ptr);
        cv::Mat result_frame(frame_row, frame_col, CV_8U, preview_ptr);

        cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);

        for(uint8_t j = 0; j< 5;j++){
            drawResult(result_frame,record[j][2],cfg[j],rectArray[j],pointArray[j]);
        if(std::abs(record[j][2]-cfg[j])>differ)
            record_cnt ++;
        }
        cv::resize(result_frame, result_frame, cv::Size(960, 720));

        if(record_cnt >= 2){
        cv::putText(result_frame, "no pass",cv::Point(100,100),cv::FONT_HERSHEY_COMPLEX,5,orange,3);
        }else{
        cv::putText(result_frame, "pass",cv::Point(100,100),cv::FONT_HERSHEY_COMPLEX,5,green,3);
        }
        cv::imshow("preview", result_frame);
	tof.releaseFrame(frame);
        tof.stop();

    while(true){
        if (cv::waitKey(1) == 27){
            exit(-1);
        }
    }
    return 0;
}
