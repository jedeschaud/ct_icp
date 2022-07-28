#ifndef SlamCore_PREDICATES_H
#define SlamCore_PREDICATES_H

namespace slam {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// POSE PREDICATES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    inline auto pose_less_by_fid = [](const auto &lhs, const auto &rhs) {
        return lhs.dest_frame_id < rhs.dest_frame_id;
    };

    inline auto pose_less_by_timestamp = [](const auto &lhs, const auto &rhs) {
        return lhs.dest_timestamp < rhs.dest_timestamp;
    };

    inline auto pose_less_by_fid_and_timestamp = [](const auto &lhs, const auto &rhs) {
        return lhs.dest_frame_id < rhs.dest_frame_id || (
                lhs.dest_frame_id == rhs.dest_frame_id && (lhs.dest_timestamp < rhs.dest_timestamp));
    };

    template<int Idx, typename Compare = std::less<>>
    struct TupleComparator {

        template<typename TupleT>
        constexpr bool operator()(const TupleT &lhs, const TupleT &rhs) const {
            return comp_(std::get<Idx>(lhs), std::get<Idx>(rhs));
        }

    private:
        Compare comp_;
    };

}

#endif //SlamCore_PREDICATES_H
