#ifndef LANDMARK_BUFFER_H
#define LANDMARK_BUFFER_H

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

enum class LandmarkType {
  UNKNOWN       = 0,
  LANE          = 1,
  LANE_MARKER   = 2,
  ROAD_EDGE     = 3,
  TRAFFIC_LIGHT = 4,
  STOP_LINE     = 5,
  CROSSWALK     = 6,
  JUNCTION      = 7,
  ARROW         = 8,
  SECTION_LANE  = 9,
  OTHER         = 10
};

struct Landmark {
  uint64_t     id_;
  LandmarkType type_;

  Landmark(uint64_t id, LandmarkType type) : id_(id), type_(type) {}
};

class LandmarkBuffer {
 private:
  std::unordered_map<LandmarkType, std::vector<std::shared_ptr<Landmark>>> landmarks_;

 public:
  // Default constructor
  LandmarkBuffer() = default;

  LandmarkBuffer(LandmarkBuffer &&other) noexcept : landmarks_(std::move(other.landmarks_)) {}

  std::shared_ptr<Landmark> CreateById(LandmarkType type, uint64_t id) {
    auto landmark = std::make_shared<Landmark>(id, type);
    landmarks_[type].emplace_back(landmark);
    return landmarks_[type].back();
  }

  std::shared_ptr<Landmark> CreateByType(LandmarkType type) {
    if (landmarks_.find(type) == landmarks_.end()) {
      return CreateById(type, 1);
    }
    const auto &landmark_vector = landmarks_.at(type);
    auto        id              = GenerateNewId(landmark_vector);
    if (!id.has_value()) {
      return nullptr;  // No available ID found
    }
    return CreateById(type, id.value());
  }

  void Add(const Landmark &landmark) { landmarks_[landmark.type_].emplace_back(std::make_unique<Landmark>(landmark)); }

  void BatchAdd(const std::vector<Landmark> &landmarks) {
    for (const auto &landmark : landmarks) {
      landmarks_[landmark.type_].emplace_back(std::make_unique<Landmark>(landmark));
    }
  }

  void Remove(LandmarkType type) { landmarks_.erase(type); }

  void Remove(LandmarkType type, uint64_t id) {
    if (landmarks_.find(type) == landmarks_.end()) {
      return;
    }
    auto &landmark_vector = landmarks_[type];
    landmark_vector.erase(std::remove_if(landmark_vector.begin(), landmark_vector.end(),
                                         [id](const std::shared_ptr<Landmark> &landmark) { return landmark->id_ == id; }),
                          landmark_vector.end());
  }

  std::shared_ptr<Landmark> FindById(LandmarkType type, uint64_t id) const {
    if (landmarks_.find(type) == landmarks_.end()) {
      return nullptr;
    }
    const auto &landmark_vector = landmarks_.at(type);
    for (const auto &landmark : landmark_vector) {
      if (landmark->id_ == id) {
        return landmark;
      }
    }
    return nullptr;
  }

  // get landmark by type
  std::optional<std::reference_wrapper<std::vector<std::shared_ptr<Landmark>>>> GetByType(LandmarkType type) {
    auto it = landmarks_.find(type);
    if (it == landmarks_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

 private:
  // Generate a new unique ID for a landmark
  std::optional<uint64_t> GenerateNewId(const std::vector<std::shared_ptr<Landmark>> &landmarks) {
    uint64_t new_id          = 1;
    bool     within_id_range = false;
    for (uint64_t i = 1; i <= 100UL; ++i) {
      auto landmark_iter =
          std::find_if(landmarks.begin(), landmarks.end(), [i](const std::shared_ptr<Landmark> &landmark) { return landmark->id_ == i; });
      if (landmark_iter == landmarks.end()) {
        new_id          = i;
        within_id_range = true;
        break;
      }
    }
    if (!within_id_range) {
      return std::nullopt;  // No available ID found within the range
    }
    return new_id;
  }
};

#endif  // LANDMARK_BUFFER_H