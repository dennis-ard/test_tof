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

sqlite3 *_sql = NULL;

cv::Scalar green(83, 168, 52);
cv::Scalar orange(42, 147, 252);

const uint8_t frame_row = 180;
const uint8_t frame_col = 240;

void execut_statement(sqlite3 *sql, const char *sql_stmt)
{
    sqlite3_stmt *stmt = NULL;
    int result = sqlite3_prepare_v2(sql, sql_stmt, -1, &stmt, NULL);

    if (result == SQLITE_OK)
    {
        // std::clog<< "statement ok"<<std::endl;
        //执行该语句
        sqlite3_step(stmt);
    }
    else
    {
        std::clog << "Problem executing statement";
    }
    //清理语句句柄，准备执行下一个语句
    sqlite3_finalize(stmt);
}

void open_sqlite(sqlite3 **sql)
{
    char path[80];

    std::time_t t = std::time(0);
    tm *nowtime = localtime(&t);
    sprintf(path, "test_tof_%d_%d.db", 1900 + nowtime->tm_year, nowtime->tm_mon + 1);

    int result = sqlite3_open_v2(path, sql, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, NULL);
    if (result == SQLITE_OK)
    {
        // std::clog << "open db success" << std::endl;
    }
    else
    {
        std::clog << "open db failure" << std::endl;
    }
}

void close_sqlite(sqlite3 *sql)
{
    if (sql)
    {
        sqlite3_close_v2(_sql);
        sql = nullptr;
    }
}

void create_table(sqlite3 *sql)
{
    const char *sqlCreateTable = "create table if not exists test_tof (tof_id integer PRIMARY KEY AUTOINCREMENT,point_position integer not NULL,eligibility INTEGER not null,test_time INTEGER NOT NULL,min_value integer not null,max_value integer not null,avg_value integer not null);";
    execut_statement(sql, sqlCreateTable);
}

void insert_record(sqlite3 *sql, float record[5][3], float cfg_param[6])
{
    char buf[300];
    std::time_t t = std::time(0);
    for (uint8_t j = 0; j < 5; j++)
    {
        record[j][2] /= 100;
        sprintf(buf, "INSERT INTO test_tof(point_position,eligibility,test_time, min_value,max_value,avg_value) VALUES(%d,%d,%ld, %f,%f,%f);", j, std::abs(record[j][2] - cfg_param[j]) > cfg_param[5] ? 0 : 1, t, record[j][0], record[j][1], record[j][2]);
        execut_statement(sql, buf);
        std::cout << "位置:" << (int)j << "偏差值:" << std::abs(record[j][2] - cfg_param[j]) << ", 最小值 : " << record[j][0] << ", 最大值: " << record[j][1] << ", 平均值: " << record[j][2] << std::endl;
    }
}

void sqlite_record(sqlite3 **sql, float record[5][3], float cfg_param[6])
{
    open_sqlite(sql);
    create_table(*sql);
    insert_record(*sql, record, cfg_param);
    close_sqlite(*sql);
}

std::string convertFloat2String(const float value, const int precision = 2)
{
    std::stringstream stream{};
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

bool readConfigFile(const char *cfgFilePath, const std::string &key, std::string &value)
{
    std::fstream cfgFile;
    cfgFile.open(cfgFilePath);
    if (!cfgFile.is_open())
    {
        std::cerr << "can not open file!" << std::endl;
        return false;
    }
    char tmp[80];
    while (!cfgFile.eof())
    {
        cfgFile.getline(tmp, 80);
        std::string line(tmp);
        auto pos = line.find('=');
        if (pos == std::string::npos)
            return false;
        std::string tmpKey = line.substr(0, pos);
        if (key == tmpKey)
        {
            value = line.substr(pos + 1);
            return true;
        }
    }
    return false;
}

bool getConfig(const std::string &key, float &value)
{
    std::string tmpValue;
    if (readConfigFile("./test.ini", key, tmpValue))
    {
        try
        {
            value = std::stof(tmpValue);
            return true;
        }
        catch (...)
        {
            std::cerr << "配置的值有问题" << std::endl;
        }
    }
    else
    {
        std::cout << "没找到配置项:" << key << std::endl;
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

void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr,int max_distance)
{
    auto len = 43200;
    for (auto i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(phase_image_ptr + i) / max_distance)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}

void drawResult(cv::Mat &result_frame, float nowValue, float standValue, cv::Rect rect, cv::Point point,float differ)
{
    if (std::abs(nowValue - standValue) > differ)
    {
        cv::rectangle(result_frame, rect, orange, 2);
        cv::putText(result_frame, convertFloat2String(nowValue), point, cv::FONT_HERSHEY_COMPLEX, 0.3, orange);
    }
    else
    {
        cv::rectangle(result_frame, rect, green, 2);
        cv::putText(result_frame, convertFloat2String(nowValue), point, cv::FONT_HERSHEY_COMPLEX, 0.3, green);
    }
}

int main()
{
    int rect = 16;
    float cfg_param_2_mod[6] = {0};
    float cfg_param_4_mod[6] = {0};

    ArduCam::ArduCamTOFCamera tof;
    ArduCam::FrameBuffer *frame;

    if (!getConfig("2topLeft", cfg_param_2_mod[0]) || !getConfig("2topRight", cfg_param_2_mod[1]) || !getConfig("2middle", cfg_param_2_mod[2]) || !getConfig("2bottomLeft", cfg_param_2_mod[3]) || !getConfig("2bottomRight", cfg_param_2_mod[4])||!getConfig("2differ", cfg_param_2_mod[5]))
    {
        return false;
    }
    if (!getConfig("4topLeft", cfg_param_4_mod[0]) || !getConfig("4topRight", cfg_param_4_mod[1]) || !getConfig("4middle", cfg_param_4_mod[2]) || !getConfig("4bottomLeft", cfg_param_4_mod[3]) || !getConfig("4bottomRight", cfg_param_4_mod[4])||!getConfig("4differ", cfg_param_4_mod[5]))
    {
        return false;
    }


    if (tof.init(ArduCam::CSI, ArduCam::DEPTH_TYPE))
    {
        std::cerr << "initialization failed" << std::endl;
        exit(-1);
    }

    if (tof.start())
    {
        std::cerr << "Failed to start camera" << std::endl;
        exit(-1);
    }
    
    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[43200];
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);

    cv::Rect rectArray[5] = {cv::Rect(0, 0, rect, rect), cv::Rect(240 - rect, 0, rect, rect), cv::Rect(120 - (rect / 2), 90 - (rect / 2), rect, rect), cv::Rect(0, 180 - rect, rect, rect), cv::Rect(240 - rect, 180 - rect, rect, rect)};
    cv::Point pointArray[5] = {cv::Point(2, 25), cv::Point(210, 25), cv::Point(118, 78), cv::Point(2, 158), cv::Point(210, 158)};
    for (uint8_t freq = 0; freq < 2; freq++)
    {
        bool insert_flag = true;
        float *cfg_param = nullptr;
        int max_distance = 0;
        if(freq == 0)
        {
            max_distance = 2;
            cfg_param = cfg_param_2_mod;
            tof.setControl(ArduCam::RANGE, max_distance);

        }else{
            max_distance = 4;
            cfg_param = cfg_param_4_mod;
            tof.setControl(ArduCam::RANGE, max_distance);
        }        float record[5][3] = {{4.0, 0, 0}, {4.0, 0, 0}, {4.0, 0, 0}, {4.0, 0, 0}, {4.0, 0, 0}};
        for (uint8_t i = 0; i < 101; i++)
        {
            do
            {
                frame = tof.requestFrame(200);
            } while (frame == nullptr);

            depth_ptr = (float *)frame->getData(ArduCam::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getData(ArduCam::AMPLITUDE_FRAME);
            getPreview(preview_ptr, depth_ptr, amplitude_ptr,max_distance);

            cv::Mat depth_frame(frame_row, frame_col, CV_32F, depth_ptr);
            cv::Mat amplitude_frame(frame_row, frame_col, CV_32F, amplitude_ptr);
            cv::Mat result_frame(frame_row, frame_col, CV_8U, preview_ptr);
            tof.releaseFrame(frame);

            cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);
            amplitude_frame.convertTo(amplitude_frame, CV_8U, 255.0 / 1024);
            cv::imshow("amplitude", amplitude_frame);
            if (i < 100)
            {
                for (uint8_t j = 0; j < 5; j++)
                {
                    float tmp = cv::mean(depth_frame(rectArray[j])).val[0];
                    drawResult(result_frame, tmp, cfg_param[j], rectArray[j], pointArray[j],cfg_param[5]);
                    if (record[j][0] > tmp)
                        record[j][0] = tmp;
                    else if (record[j][1] < tmp)
                        record[j][1] = tmp;
                    record[j][2] += tmp;
                }

                cv::resize(result_frame, result_frame, cv::Size(960, 720));

                cv::imshow("preview", result_frame);

                if (cv::waitKey(1) == 27)
                    break;

                display_fps();
            }
            else
            {
                i--;
                if(insert_flag == true){
                    insert_flag = false;
                    sqlite_record(&_sql,record,cfg_param);
                }
                int record_cnt = 0;
                for (uint8_t j = 0; j < 5; j++)
                {
                    drawResult(result_frame, record[j][2], cfg_param[j], rectArray[j], pointArray[j],cfg_param[5]);
                    if (std::abs(record[j][2] - cfg_param[j]) > cfg_param[5])
                        record_cnt++;
                }
                cv::resize(result_frame, result_frame, cv::Size(960, 720));

                if (record_cnt >= 2)
                {
                    cv::putText(result_frame, "no pass", cv::Point(100, 100), cv::FONT_HERSHEY_COMPLEX, 5, orange, 3);
                }
                else
                {
                    cv::putText(result_frame, "pass", cv::Point(100, 100), cv::FONT_HERSHEY_COMPLEX, 5, green, 3);
                }
                cv::imshow("preview", result_frame);
                if (cv::waitKey(1) == 'q')
                {
                    break;
                }
            }
        }
    }

    tof.stop();
    return 0;
}
