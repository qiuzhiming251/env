#ifndef VERSION_INFO_PARAMS_H_
#define VERSION_INFO_PARAMS_H_

#include <string>

namespace cem {
namespace fusion {

struct VersionInfoParams
{
    std::string git_branch = "";
    std::string commit_id = "";
};

} // namespace fusion
} // namespace cem

#endif
