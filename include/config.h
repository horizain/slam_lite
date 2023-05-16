#ifndef __SLAM_LITE_CONFIG_H
#define __SLAM_LITE_CONFIG_H

#include "include/common_include.h"


namespace slamlite
{
class Config
{
  private:
    static std::shared_ptr<Config> _config;
    cv::FileStorage _file;

  // 私有构造函数，单例模式
    Config() {};

  public:
    ~Config();

    static bool SetParameterFile(const std::string &filename);

    template<typename T>
    static T Get(const std::string &key)
    {
        return T(Config::_config->_file[key]);
    }
};

} // namespace Config

#endif