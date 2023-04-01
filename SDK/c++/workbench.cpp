//
// Created by 刘智杰 on 2023/3/10.
//
#include "workbench.h"


Workbench::Workbench(int type_id, double x0, double y0, int frame_remain, int materials_status,
              int product_status) : type_id_(type_id), pos_({x0, y0}), frame_remain_(frame_remain),
              materials_status_(materials_status), product_status_(product_status){}

bool Workbench::TryToBuy(int carry_id, double time_to_buy) { // carry_id : 物品的 id
    if(type_id_ >= 1 && type_id_ <= 7 && type_id_ == carry_id) {
        if(product_status_ || (product_status_ == 0 && frame_remain_ > 0 && time_to_buy >= frame_remain_)) return true;
    }
    return false;
}

bool Workbench::TryToSell(int carry_id) { // carry_id : 物品的 id
    if(type_id_ == 1 || type_id_ == 2 || type_id_ == 3) return false;
    if(type_id_ == 4 && (carry_id == 1 || carry_id == 2)) {
        if(!(materials_status_ & (1 << carry_id))) return true;
    }
    if(type_id_ == 5 && (carry_id == 1 || carry_id == 3)) {
        if(!(materials_status_ & (1 << carry_id))) return true;
    }
    if(type_id_ == 6 && (carry_id == 2 || carry_id == 3)) {
        if(!(materials_status_ & (1 << carry_id))) return true;
    }
    if(type_id_ == 7 && (carry_id == 4 || carry_id == 5 || carry_id == 6)) {
        if(!(materials_status_ & (1 << carry_id))) return true;
    }
    if(type_id_ == 8 && (carry_id == 7)) {
        if(!(materials_status_ & (1 << carry_id))) return true;
    }
    if(type_id_ == 9 && (carry_id >= 1 && carry_id <= 7)) {
        if(!(materials_status_ & (1 << carry_id))) return true;
    }
    return false;
}

int Workbench::ItemsAreMissing() {
    int s = 0;
    if(type_id_ == 4) {
        if(!(materials_status_ & (1 << 1))) s++;
        if(!(materials_status_ & (1 << 2))) s++;
    }
    else if(type_id_ == 5) {
        if(!(materials_status_ & (1 << 1))) s++;
        if(!(materials_status_ & (1 << 3))) s++;
    }
    else if(type_id_ == 6) {
        if(!(materials_status_ & (1 << 2))) s++;
        if(!(materials_status_ & (1 << 3))) s++;
    }
    else if(type_id_ == 7) {
        if(!(materials_status_ & (1 << 4))) s++;
        if(!(materials_status_ & (1 << 5))) s++;
        if(!(materials_status_ & (1 << 6))) s++;
    }
    return s;
}