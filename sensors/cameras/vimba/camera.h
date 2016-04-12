// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SNARK_SENSORS_VIMBA_CAMERA_H_
#define SNARK_SENSORS_VIMBA_CAMERA_H_

#include <VimbaCPP/Include/Camera.h>
#include "snark/imaging/cv_mat/serialization.h"

namespace snark { namespace vimba {

class camera
{
    public:
        camera( const std::string& camera_id );
        camera( const AVT::VmbAPI::CameraPtr& camera_ptr ) : camera_( camera_ptr ) {}
        ~camera();

        void print_info( bool verbose ) const;
        void list_attributes( bool verbose ) const;
        void set_feature( const std::string& feature_name, const std::string& value = "" ) const;
        void set_features( const std::string& name_value_pairs ) const;
        void capture_images( boost::shared_ptr< snark::cv_mat::serialization > serialization ) const;

    private:
        VmbErrorType start_continuous_image_acquisition( boost::shared_ptr< snark::cv_mat::serialization > serialization ) const;
        void stop_continuous_image_acquisition() const;

        AVT::VmbAPI::CameraPtr camera_;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_CAMERA_H_
