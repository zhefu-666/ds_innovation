#pragma once

#include <opencv2/core/persistence.hpp>

#include <stdexcept>
#include <string>

namespace rescue {

struct FileReference {
    std::string file;
    std::string fragment;
};

inline FileReference splitFileReference(const std::string& reference) {
    const auto marker = reference.find('#');
    if (marker == std::string::npos)
        return {reference, {}};
    return {reference.substr(0, marker), reference.substr(marker + 1)};
}

inline cv::FileNode selectFileNode(cv::FileStorage& storage, const std::string& fragment) {
    cv::FileNode node = storage.root();
    if (fragment.empty())
        return node;
    std::string path = fragment;
    if (!path.empty() && path.front() == '/')
        path.erase(path.begin());
    size_t begin = 0;
    while (begin <= path.size()) {
        const size_t end = path.find('/', begin);
        std::string key = path.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
        if (!key.empty()) {
            std::string decoded;
            for (size_t i = 0; i < key.size(); ++i) {
                if (key[i] == '~' && i + 1 < key.size() && key[i + 1] == '0') {
                    decoded.push_back('~'); ++i;
                } else if (key[i] == '~' && i + 1 < key.size() && key[i + 1] == '1') {
                    decoded.push_back('/'); ++i;
                } else {
                    decoded.push_back(key[i]);
                }
            }
            node = node[decoded];
            if (node.empty())
                throw std::runtime_error("Missing configuration fragment: #" + fragment);
        }
        if (end == std::string::npos)
            break;
        begin = end + 1;
    }
    return node;
}

} // namespace rescue
