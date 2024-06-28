/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_NODELET_H
#define MSCKF_VIO_NODELET_H

#include <msckf_vio/msckf_vio.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace msckf_vio {
// nodelet 需要继承自 nodelet::Nodelet 类
// nodelet不需要main函数，只用实现onInit函数即可
class MsckfVioNodelet : public nodelet::Nodelet {
   public:
    MsckfVioNodelet() { return; }
    ~MsckfVioNodelet() { return; }

   private:
    virtual void onInit();
    MsckfVioPtr msckf_vio_ptr;
};
}  // end namespace msckf_vio

#endif
