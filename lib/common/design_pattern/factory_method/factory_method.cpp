#include "factory_method.h"

namespace cem {
namespace fusion {

BaseClassMap &GlobalFactoryMap()
{
    static BaseClassMap factory_map;
    return factory_map;
}

bool GetRegisteredClasses(
    const std::string &base_class_name,
    std::vector<std::string> *registered_derived_classes_names)
{
    if (registered_derived_classes_names == nullptr)
    {
        return false;
    }
    BaseClassMap &map = GlobalFactoryMap();
    auto iter = map.find(base_class_name);
    if (iter == map.end())
    {

        return false;
    }
    for (auto pair : iter->second)
    {
        registered_derived_classes_names->push_back(pair.first);
    }
    return true;
}

} // namespace fusion
} // namespace cem
