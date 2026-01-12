#pragma once

#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "cyber/common/log.h"

namespace cem {
namespace fusion {

struct DstCommonData
{
    
    bool init_ = false;

    size_t fod_loc_ = 0;
    std::vector<uint64_t> fod_subsets_;

    std::vector<size_t> fod_subset_cardinalities_;
    std::vector<std::string> fod_subset_names_;

    std::vector<std::vector<std::pair<size_t, size_t>>> combination_relations_;

    std::vector<std::vector<size_t>> subset_relations_;

    std::vector<std::vector<size_t>> inter_relations_;
    std::map<uint64_t, size_t> subsets_ind_map_;
};

typedef DstCommonData *DstCommonDataPtr;


class DstManager
{
public:
    static DstManager *Instance()
    {
        static DstManager dst_manager;
        return &dst_manager;
    }

    bool AddApp(const std::string &app_name,
                const std::vector<uint64_t> &fod_subsets,
                const std::vector<std::string> &fod_subset_names =
                    std::vector<std::string>());
    bool IsAppAdded(const std::string &app_name);

    DstCommonDataPtr GetAppDataPtr(const std::string &app_name);
    size_t FodSubsetToInd(const std::string &app_name,
                          const uint64_t &fod_subset);
    uint64_t IndToFodSubset(const std::string &app_name, const size_t &ind);

private:
    DstManager() {}
    void BuildSubsetsIndMap(DstCommonData *dst_data);

    void FodCheck(DstCommonData *dst_data);

    void ComputeCardinalities(DstCommonData *st_data);
    bool ComputeRelations(DstCommonData *dst_data);
    void BuildNamesMap(const std::vector<std::string> &fod_subset_names,
                       DstCommonData *dst_data);

private:

    std::map<std::string, DstCommonData> dst_common_data_;

    std::mutex map_mutex_;
};

class Dst
{
public:
    explicit Dst(const std::string &app_name);

    bool SetBbaVec(const std::vector<double> &bba_vec);
    bool SetBba(const std::map<uint64_t, double> &bba_map);

    void ComputeSptPlsUct() const;
    void ComputeProbability() const;
    const std::vector<double> &GetBbaVec() const { return bba_vec_; }
    const size_t GetBbaSize() const { return bba_vec_.size(); }
    double GetSubsetBfmass(uint64_t fod_subset) const;
    double GetIndBfmass(size_t ind) const;
    const std::vector<double> &GetSupportVec() const { return support_vec_; }
    const std::vector<double> &GetPlausibilityVec() const
    {
        return plausibility_vec_;
    }
    const std::vector<double> &GetUncertaintyVec() const
    {
        return uncertainty_vec_;
    }
    const std::vector<double> &GetProbabilityVec() const
    {
        return probability_vec_;
    }
    std::string PrintBba() const;

    friend Dst operator+(const Dst &lhs, const Dst &rhs);
    friend Dst operator*(const Dst &dst_evidence, double w);
    std::string Name() const { return app_name_; }

private:
    void Normalize();
    void SelfCheck() const;

private:
    std::string app_name_;
    mutable DstCommonDataPtr dst_data_ptr_ = nullptr;
    mutable std::vector<double> bba_vec_;
    mutable std::vector<double> support_vec_;
    mutable std::vector<double> plausibility_vec_;
    mutable std::vector<double> uncertainty_vec_;
    mutable std::vector<double> probability_vec_;
};

} // namespace fusion
} // namespace cem
