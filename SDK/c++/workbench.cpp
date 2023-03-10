//
// Created by 刘智杰 on 2023/3/10.
//
#include "workbench.h"
#include <iostream>


Workbench::Workbench(size_t type_id, double x0, double y0, size_t frame_remain, size_t material_status,
              size_t product_status) : type_id_(type_id), x0_(x0), y0_(y0), frame_remain_(frame_remain),
              material_status_(material_status), product_status_(product_status)){}

