#ifndef CONCURRENT_OBJECT_POOL_H_
#define CONCURRENT_OBJECT_POOL_H_

#include <deque>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include "base/base_object_pool.h"

#define PERCEPTION_BASE_DISABLE_POOL

namespace cem {
namespace fusion {

static const size_t kPoolDefaultExtendNum = 10;
static const size_t kPoolDefaultSize = 100;


template <class T>
struct ObjectPoolDefaultInitializer
{
    void operator()(T *t) const {}
};

template <class ObjectType, size_t N = kPoolDefaultSize,
          class Initializer = ObjectPoolDefaultInitializer<ObjectType>>
class ConcurrentObjectPool : public BaseObjectPool<ObjectType>
{
public:

    using BaseObjectPool<ObjectType>::capacity_;

    static ConcurrentObjectPool &Instance()
    {
        static ConcurrentObjectPool pool(N);
        return pool;
    }

    std::shared_ptr<ObjectType> Get() override
    {

#ifndef PERCEPTION_BASE_DISABLE_POOL
        ObjectType *ptr = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.empty())
            {
                Add(1 + kPoolDefaultExtendNum);
            }
            ptr = queue_.front();
            queue_.pop();
        }

        kInitializer(ptr);
        return std::shared_ptr<ObjectType>(ptr, [&](ObjectType *obj_ptr) {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(obj_ptr);
        });
#else
        return std::shared_ptr<ObjectType>(new ObjectType);
#endif
    }

    void BatchGet(size_t num,
                  std::vector<std::shared_ptr<ObjectType>> *data) override
    {
#ifndef PERCEPTION_BASE_DISABLE_POOL
        std::vector<ObjectType *> buffer(num, nullptr);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() < num)
            {
                Add(num - queue_.size() + kPoolDefaultExtendNum);
            }
            for (size_t i = 0; i < num; ++i)
            {
                buffer[i] = queue_.front();
                queue_.pop();
            }
        }

        for (size_t i = 0; i < num; ++i)
        {
            kInitializer(buffer[i]);
            data->emplace_back(std::shared_ptr<ObjectType>(
                buffer[i], [&](ObjectType *obj_ptr) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    queue_.push(obj_ptr);
                }));
        }
#else
        for (size_t i = 0; i < num; ++i)
        {
            data->emplace_back(new ObjectType);
        }
#endif
    }

    void BatchGet(size_t num, bool is_front,
                  std::list<std::shared_ptr<ObjectType>> *data) override
    {
#ifndef PERCEPTION_BASE_DISABLE_POOL
        std::vector<ObjectType *> buffer(num, nullptr);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() < num)
            {
                Add(num - queue_.size() + kPoolDefaultExtendNum);
            }
            for (size_t i = 0; i < num; ++i)
            {
                buffer[i] = queue_.front();
                queue_.pop();
            }
        }

        for (size_t i = 0; i < num; ++i)
        {
            kInitializer(buffer[i]);
            is_front ? data->emplace_front(std::shared_ptr<ObjectType>(
                           buffer[i],
                           [&](ObjectType *obj_ptr) {
                               std::lock_guard<std::mutex> lock(mutex_);
                               queue_.push(obj_ptr);
                           }))
                     : data->emplace_back(std::shared_ptr<ObjectType>(
                           buffer[i], [&](ObjectType *obj_ptr) {
                               std::lock_guard<std::mutex> lock(mutex_);
                               queue_.push(obj_ptr);
                           }));
        }
#else
        for (size_t i = 0; i < num; ++i)
        {
            is_front ? data->emplace_front(new ObjectType)
                     : data->emplace_back(new ObjectType);
        }
#endif
    }

    void BatchGet(size_t num, bool is_front,
                  std::deque<std::shared_ptr<ObjectType>> *data) override
    {
#ifndef PERCEPTION_BASE_DISABLE_POOL
        std::vector<ObjectType *> buffer(num, nullptr);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() < num)
            {
                Add(num - queue_.size() + kPoolDefaultExtendNum);
            }
            for (size_t i = 0; i < num; ++i)
            {
                buffer[i] = queue_.front();
                queue_.pop();
            }
        }
        for (size_t i = 0; i < num; ++i)
        {
            kInitializer(buffer[i]);
            is_front ? data->emplace_front(std::shared_ptr<ObjectType>(
                           buffer[i],
                           [&](ObjectType *obj_ptr) {
                               std::lock_guard<std::mutex> lock(mutex_);
                               queue_.push(obj_ptr);
                           }))
                     : data->emplace_back(std::shared_ptr<ObjectType>(
                           buffer[i], [&](ObjectType *obj_ptr) {
                               std::lock_guard<std::mutex> lock(mutex_);
                               queue_.push(obj_ptr);
                           }));
        }
#else
        for (size_t i = 0; i < num; ++i)
        {
            is_front ? data->emplace_front(new ObjectType)
                     : data->emplace_back(new ObjectType);
        }
#endif
    }
#ifndef PERCEPTION_BASE_DISABLE_POOL

    void set_capacity(size_t capacity) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (capacity_ < capacity)
        {
            Add(capacity - capacity_);
        }
    }

    size_t RemainedNum() override { return queue_.size(); }
#endif
    ~ConcurrentObjectPool() override
    {
        if (cache_)
        {
            delete[] cache_;
            cache_ = nullptr;
        }
        for (auto &ptr : extended_cache_)
        {
            delete ptr;
        }
        extended_cache_.clear();
    }

protected:

#ifndef PERCEPTION_BASE_DISABLE_POOL
    void Add(size_t num)
    {
        for (size_t i = 0; i < num; ++i)
        {
            ObjectType *ptr = new ObjectType;
            extended_cache_.push_back(ptr);
            queue_.push(ptr);
        }
        capacity_ = kDefaultCacheSize + extended_cache_.size();
    }
#endif

    explicit ConcurrentObjectPool(const size_t default_size)
        : kDefaultCacheSize(default_size)
    {
#ifndef PERCEPTION_BASE_DISABLE_POOL
        cache_ = new ObjectType[kDefaultCacheSize];
        for (size_t i = 0; i < kDefaultCacheSize; ++i)
        {
            queue_.push(&cache_[i]);
        }
        capacity_ = kDefaultCacheSize;
#endif
    }
    std::mutex mutex_;
    std::queue<ObjectType *> queue_;

    ObjectType *cache_ = nullptr;
    const size_t kDefaultCacheSize;

    std::list<ObjectType *> extended_cache_;
    static const Initializer kInitializer;
};

} // namespace fusion
} // namespace cem

#endif
