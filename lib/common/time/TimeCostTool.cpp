
#include "TimeCostTool.h"

namespace cem {
    namespace fusion {

TimeCostTool* TimeCostTool::instance = nullptr;
std::mutex TimeCostTool::mutex_;

TimeCostTool::TimeCostTool() {

}

TimeCostTool* TimeCostTool::getInstance() {
    if (instance == nullptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance == nullptr) {
            instance = new TimeCostTool();
        }
    }
    return instance;
}




    }
}