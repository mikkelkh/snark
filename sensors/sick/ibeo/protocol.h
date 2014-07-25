// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#ifndef SNARK_SENSORS_SICK_PROTOCOL_H_
#define SNARK_SENSORS_SICK_PROTOCOL_H_

/// @file protocol.h
/// sick (ibeo) ldmrs laser communication protocol
/// @author vsevolod vlaskine (v.vlaskine@acfr.usyd.edu.au)

#include <iostream>
#include <string>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <snark/sensors/sick/ibeo/packets.h>

namespace snark {  namespace sick { namespace ibeo {

class protocol : public boost::noncopyable
{
    public:
        /// constructor for two-way communication with sensor
        protocol( std::iostream& stream );

        /// constructor for reading only (scans, warnings, etc)
        protocol( std::istream& stream );
        
        /// destructor
        ~protocol();
        
        /// reset sensor (send reset DSP)
        void reset_dsp();
        
        /// send command
        template < typename command > typename command::response write( const command& c );
        
        /// read scan data packet
        /// @note once scanning started, call readscan() often to flush receive buffer
        const scan_packet* readscan();
        
        /// return last fault, if any
        boost::optional< fault > last_fault();
        
        /// fault exception, quick and dirty (since comma::exception is so difficult to inherit from)
        struct faultException : public std::exception {};
        
    private:
        class impl;
        impl* m_pimpl;
};

} } } // namespace snark {  namespace sick { namespace ibeo {
    
#endif // #ifndef SNARK_SENSORS_SICK_PROTOCOL_H_
