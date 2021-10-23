/*
 * @Description: 
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-15 21:19:21
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-23 14:29:03
 */
/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_BASE_OBJECT_BUILDER_HPP_
#define OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_BASE_OBJECT_BUILDER_HPP_

#include <string>
#include <vector>

//#include"../../common_lib/include/common/types/object.hpp"
//#include"../../common_lib/include/common/types/type.h"
#include "common/types/object.hpp" 
#include "common/types/type.h"

namespace autosense {
namespace object_builder {

class BaseObjectBuilder {
 public:
    virtual void build(const std::vector<PointICloudPtr>& cloud_clusters,
                       std::vector<ObjectPtr>* objects) = 0;

    virtual void build(ObjectPtr object) = 0;

    virtual std::string name() const = 0;
};  // class BaseObjectBuilder

}  // namespace object_builder
}  // namespace autosense

#endif  // OBJECT_BUILDERS_INCLUDE_OBJECT_BUILDERS_BASE_OBJECT_BUILDER_HPP_
