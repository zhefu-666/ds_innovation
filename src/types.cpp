#include "rescue/types.hpp"

namespace rescue {

cv::Point2f Detection::center() const {
    return cv::Point2f(box.x + box.width * 0.5f, box.y + box.height * 0.5f);
}

float Detection::area() const {
    return box.width * box.height;
}

} // namespace rescue
