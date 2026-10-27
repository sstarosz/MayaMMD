#pragma once
#include <vector>

namespace mmd::core::joint_display
{
struct HighlightCandidate
{
    unsigned int roles;
    bool drawable;
    bool self;
};
struct HighlightOwner
{
    unsigned int roles = 0;
    bool self = false;
};
inline HighlightOwner highlightOwner(const std::vector<HighlightCandidate>& candidates)
{
    HighlightOwner result;
    bool assigned = false;
    for (const auto& candidate : candidates)
    {
        result.roles |= candidate.roles;
        if (!assigned && candidate.drawable)
        {
            result.self = candidate.self;
            assigned = true;
        }
    }
    return result;
}
}
