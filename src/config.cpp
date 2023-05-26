#include "config.h"

namespace slamlite
{
    bool Config::SetParameterFile(const std::string &filename)
    {
        if (_config == nullptr)
            _config = std::shared_ptr<Config>(new Config);
        _config->_file = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (_config->_file.isOpened() == false)
        {
            _config->_file.release();
            return false;
        }
        return true;
    }

    Config::~Config()
    {
        if (_file.isOpened())
            _file.release();
    }

    // 因为类的声明并不会进行内存空间的分配。
    // 所以类的静态成员无法在类声明中定义。
    // 因此，类的静态成员需要类内声明，类外定义。并且注意定义尽量不要出现在头文件中，以免造成重复定义。
    std::shared_ptr<Config> Config::_config = nullptr;
} // namespace slamlite
