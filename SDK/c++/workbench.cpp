//
// Created by 刘智杰 on 2023/3/10.
//
#include "workbench.h"


Workbench::Workbench(int type_id, double x0, double y0, int frame_remain, int materials_status,
              int product_status) : type_id_(type_id), x0_(x0), y0_(y0), frame_remain_(frame_remain),
              materials_status_(materials_status), product_status_(product_status){}

