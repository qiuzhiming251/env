# std::set 使用指南

## 简介

`std::set` 是 C++ STL（标准模板库）中的关联容器，用于存储唯一元素，并且这些元素会自动排序。它基于红黑树实现，具有以下特点：

- 元素唯一性：不允许重复元素
- 自动排序：元素按特定顺序排列
- 高效查找：查找、插入和删除操作的时间复杂度为 O(log n)

## 基本语法

```cpp
#include <set>

// 声明一个 set
std::set<T> mySet;  // T 是元素类型

// 示例
std::set<int> numbers;
std::set<std::string> names;
```

## 常用操作

### 1. 插入元素

```cpp
std::set<int> mySet;

// 使用 insert 方法插入元素
mySet.insert(10);
mySet.insert(20);
mySet.insert(30);

// 插入重复元素不会生效
mySet.insert(20);  // 不会改变 set，因为 20 已存在

// 批量插入
std::vector<int> vec = {1, 2, 3, 4, 5};
mySet.insert(vec.begin(), vec.end());
```

### 2. 查找元素

```cpp
std::set<int> mySet = {10, 20, 30, 40, 50};

// 使用 find 方法查找元素
auto it = mySet.find(30);
if (it != mySet.end()) {
    std::cout << "找到了元素: " << *it << std::endl;
} else {
    std::cout << "未找到元素" << std::endl;
}

// 使用 count 方法检查元素是否存在
if (mySet.count(30)) {
    std::cout << "元素 30 存在" << std::endl;
}
```

### 3. 删除元素

```cpp
std::set<int> mySet = {10, 20, 30, 40, 50};

// 删除指定值的元素
mySet.erase(30);

// 通过迭代器删除元素
auto it = mySet.find(20);
if (it != mySet.end()) {
    mySet.erase(it);
}

// 删除范围内的元素
auto first = mySet.find(10);
auto last = mySet.find(50);
if (first != mySet.end() && last != mySet.end()) {
    mySet.erase(first, last);  // 删除 [first, last) 范围内的元素
}
```

### 4. 获取大小和清空

```cpp
std::set<int> mySet = {10, 20, 30, 40, 50};

// 获取元素个数
size_t size = mySet.size();

// 检查是否为空
if (mySet.empty()) {
    std::cout << "set 为空" << std::endl;
}

// 清空所有元素
mySet.clear();
```

## 迭代遍历

```cpp
std::set<int> mySet = {10, 20, 30, 40, 50};

// 正向遍历
for (const auto& element : mySet) {
    std::cout << element << " ";
}
std::cout << std::endl;

// 使用迭代器遍历
for (auto it = mySet.begin(); it != mySet.end(); ++it) {
    std::cout << *it << " ";
}
std::cout << std::endl;

// 反向遍历
for (auto it = mySet.rbegin(); it != mySet.rend(); ++it) {
    std::cout << *it << " ";
}
std::cout << std::endl;
```

## 自定义比较函数

默认情况下，`std::set` 使用 `<` 操作符进行排序。可以自定义比较函数来改变排序方式。

```cpp
// 使用函数指针作为比较函数
struct CustomCompare {
    bool operator()(const int& lhs, const int& rhs) const {
        return lhs > rhs;  // 降序排列
    }
};

std::set<int, CustomCompare> descendingSet;

// 或者使用 lambda 表达式（C++11 及以上）
auto cmp = [](const int& lhs, const int& rhs) { return lhs > rhs; };
std::set<int, decltype(cmp)> customSet(cmp);
```

## 常用方法总结

| 方法 | 功能 | 时间复杂度 |
|------|------|------------|
| insert() | 插入元素 | O(log n) |
| erase() | 删除元素 | O(log n) |
| find() | 查找元素 | O(log n) |
| count() | 计算元素个数（0或1） | O(log n) |
| size() | 返回元素个数 | O(1) |
| empty() | 判断是否为空 | O(1) |
| begin()/end() | 返回首尾迭代器 | O(1) |
| clear() | 清空所有元素 | O(n) |

## 实际应用示例

```cpp
#include <iostream>
#include <set>
#include <string>

int main() {
    // 存储唯一的用户名
    std::set<std::string> usernames;
    
    // 添加用户
    usernames.insert("alice");
    usernames.insert("bob");
    usernames.insert("charlie");
    usernames.insert("alice");  // 重复，不会添加
    
    std::cout << "用户数量: " << usernames.size() << std::endl;
    
    // 检查用户是否存在
    if (usernames.count("alice")) {
        std::cout << "用户 alice 存在" << std::endl;
    }
    
    // 遍历所有用户（按字母顺序）
    std::cout << "所有用户:" << std::endl;
    for (const auto& name : usernames) {
        std::cout << "  " << name << std::endl;
    }
    
    // 删除用户
    usernames.erase("bob");
    
    std::cout << "删除 bob 后的用户:" << std::endl;
    for (const auto& name : usernames) {
        std::cout << "  " << name << std::endl;
    }
    
    return 0;
}
```

## 注意事项

1. **元素唯一性**: `std::set` 中的每个元素都是唯一的，尝试插入重复元素不会有任何效果。
2. **自动排序**: 元素会根据比较函数自动排序，不能直接修改 set 中元素的值。
3. **性能考虑**: 对于频繁的插入和查找操作，set 是很好的选择；但如果需要频繁访问特定位置的元素，可能 vector 或 deque 更合适。
4. **内存占用**: set 相比于 vector 会有额外的内存开销，因为它需要维护树结构。

## 与其他容器的比较

| 容器 | 有序 | 允许重复 | 查找效率 | 插入/删除效率 |
|------|------|----------|----------|---------------|
| set | 是 | 否 | O(log n) | O(log n) |
| multiset | 是 | 是 | O(log n) | O(log n) |
| unordered_set | 否 | 否 | O(1) 平均 | O(1) 平均 |
| vector | 否 | 是 | O(n) | O(1) 末尾 |
| list | 否 | 是 | O(n) | O(1) 已知位置 |

选择合适的容器取决于具体的应用场景和性能要求。
