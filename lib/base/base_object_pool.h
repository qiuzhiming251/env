#ifndef BASE_OBJECT_POOL_H_
#define BASE_OBJECT_POOL_H_

#include <deque>
#include <list>
#include <memory>
#include <vector>

namespace cem {
namespace fusion {


template <class ObjectType>
class BaseObjectPool
{
public:

    BaseObjectPool() = default;

    virtual ~BaseObjectPool() = default;

    virtual std::shared_ptr<ObjectType> Get() = 0;

    virtual void BatchGet(size_t num,
                          std::vector<std::shared_ptr<ObjectType>> *data) = 0;

    virtual void BatchGet(size_t num, bool is_front,
                          std::list<std::shared_ptr<ObjectType>> *data) = 0;

    virtual void BatchGet(size_t num, bool is_front,
                          std::deque<std::shared_ptr<ObjectType>> *data) = 0;

    virtual void set_capacity(size_t capacity) {}

    size_t get_capacity() { return capacity_; }

    virtual size_t RemainedNum() { return 0; }

protected:
    BaseObjectPool(const BaseObjectPool &rhs) = delete;
    BaseObjectPool &operator=(const BaseObjectPool &rhs) = delete;
    size_t capacity_ = 0;
};


template <class ObjectType>
class DummyObjectPool : public BaseObjectPool<ObjectType>
{
public:

    static DummyObjectPool &Instance()
    {
        static DummyObjectPool pool;
        return pool;
    }

    std::shared_ptr<ObjectType> Get() override
    {
        return std::shared_ptr<ObjectType>(new ObjectType);
    }

    void BatchGet(size_t num,
                  std::vector<std::shared_ptr<ObjectType>> *data) override
    {
        for (size_t i = 0; i < num; ++i)
        {
            data->emplace_back(std::shared_ptr<ObjectType>(new ObjectType));
        }
    }

    void BatchGet(size_t num, bool is_front,
                  std::list<std::shared_ptr<ObjectType>> *data) override
    {
        for (size_t i = 0; i < num; ++i)
        {
            is_front ? data->emplace_front(
                           std::shared_ptr<ObjectType>(new ObjectType))
                     : data->emplace_back(
                           std::shared_ptr<ObjectType>(new ObjectType));
        }
    }

    void BatchGet(size_t num, bool is_front,
                  std::deque<std::shared_ptr<ObjectType>> *data) override
    {
        for (size_t i = 0; i < num; ++i)
        {
            is_front ? data->emplace_front(
                           std::shared_ptr<ObjectType>(new ObjectType))
                     : data->emplace_back(
                           std::shared_ptr<ObjectType>(new ObjectType));
        }
    }

protected:

    DummyObjectPool() = default;
}; 

} // namespace fusion
} // namespace cem

#endif
