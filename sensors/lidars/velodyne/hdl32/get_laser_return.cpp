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


#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/math/compare.h>
#include <comma/base/exception.h>
#include "../impl/angle.h"
#include "get_laser_return.h"

namespace snark {  namespace velodyne { namespace hdl32 {

struct timestamps
{
    // time offsets of the laser blocks
    static const double offsets[];

    // time offset of the laser return written to packet first (see velodyne HDL-64E S2 Manual, Appendix D)
    static const double first;

    // time offset of the laser return written to packet last (see velodyne HDL-64E S2 Manual, Appendix D)
    static const double last;

    // time elapsed between first and last laser returns
    static const double elapsed;

    // number of steps between first and last laser returns
    // ( upper and lower blocks are fired at the same time, see velodyne HDL-64E S2 Manual, Appendix D)
    static const std::size_t size;

    // step size between two laser returns (a convenience definition)
    static const double step;

    // step size between two laser returns
    static const double ethernetOutputDuration;

    // return laser return offset in a laser block
    static double offsetInBlock( std::size_t laser ) { return step * laser; }
};

// see velodyne HDL-64ES2 User's Manual, p.19:
// First transmission trigger to Ethernet tranmission enable: 419.3 microseconds
const double timestamps::offsets[] = {   -0.0004193, -0.0004193
                                       , -0.0003960, -0.0003960
                                       , -0.0003727, -0.0003727
                                       , -0.0003494, -0.0003494
                                       , -0.0003261, -0.0003261
                                       , -0.0003028, -0.0003028 };

const double timestamps::first( timestamps::offsets[0] );
const double timestamps::last( -0.0002803 );
const double timestamps::elapsed( last - first );
const std::size_t timestamps::size( 6 * 32 );
const double timestamps::step( elapsed / ( timestamps::size - 1 ) ); // see velodyne HDL-64ES2 User's Manual, p.19
const double timestamps::ethernetOutputDuration( 0.0001 ); // Ethernet output duration: 100 microseconds

// In the HDL-64E S2, the upper block and lower block collect distance points simultaneously, with
// each block issuing single laser pulses at a time. That is, each upper block laser fires in
// sequence and in unison to a corresponding laser from the lower block. For example, laser 32
// fires simultaneously with laser 0, laser 33 fires with laser 1, and so on. Unlike the HDL-64E,
// which issued three upper block returns for every lowestepr block return, the HDL-64E S2 has an
// equal number of upper and lower block returns. This is why when interpreting the delay table
// each sequential pair of data blocks will represent the upper and lower block respectively,
// and each upper and lower block pair of data blocks in the Ethernet packet will have the same
// delay values.
//
// Ethernet packets are assembled until the entire 1200 bytes have been collected, representing
// six upper block sequences and six lower block sequences. The packet is then transmitted via
// a UDP packet over Ethernet, starting from the last byte acquired. See a sample of the packet
// layout on page 20.
//
// Simply subtract from the timestamp of the output event of the packet each data value
// to arrive at the actual time the distance point was captured inside the HDL-64E S2
boost::posix_time::time_duration time_offset( unsigned int block, unsigned int laser )
{
    double offset = timestamps::offsets[ block ] + timestamps::step * laser - timestamps::ethernetOutputDuration;
    return boost::posix_time::microseconds( offset * 1000000 );
}

double azimuth( double rotation, unsigned int laser, double angularSpeed )
{
std::cerr<<"rot="<<angularSpeed<<std::endl;
    double a = rotation + angularSpeed * timestamps::step * laser + 90; // add 90 degrees for our system of coordinates (although this value is only output for later processing - can keep its own)
    if( comma::math::less( a, 360 ) ) { if( comma::math::less( a, 0 ) ) { a += 360; } }
    else { a -= 360; }
    return a;
}

double azimuth( const packet& packet, unsigned int block, unsigned int laser, double angularSpeed )
{
    return azimuth( double( packet.blocks[block].rotation() ) / 100, laser, angularSpeed );
}

const int lasers_per_block=32;
const int block_count=12;
//struct hdl64_s2_fw_v48
//{
//    typedef double block_time_table[lasers_per_block];
//    static block_time_table time_table[block_count];
//    static boost::posix_time::time_duration time_delay( unsigned int block, unsigned int laser )
//    {
//        if(laser<0 ||laser>=lasers_per_block) { COMMA_THROW( comma::exception, "laser id out of range" << laser ); }
//        if(block<0||block>=block_count) { COMMA_THROW(comma::exception, "block id out of range"<<block ); }
//        double delay = (time_table[block][laser] ) + timestamps::ethernetOutputDuration * 1e6;
//        return boost::posix_time::microseconds( delay);
//    }
//    static double azimuth(const packet& packet, unsigned int block, unsigned int laser, double angularSpeed )
//    {
//        // todo: angular speed correction with offset for angular velocity that calibration was measured on
//        double rotation = double( packet.blocks[block].rotation() ) / 100;
//        double laser_time = (time_table[block][0] - time_table[block][laser%lasers_per_block]) * 1e-6;
//        double a = rotation + 90;
//        a+=angularSpeed * laser_time;
//        if( comma::math::less( a, 360 ) ) { if( comma::math::less( a, 0 ) ) { a += 360; } }
//        else { a -= 360; }
//        return a;
//    }
//};

struct hdl32
{
    typedef double block_time_table[lasers_per_block];
    static block_time_table time_table[block_count];
    static boost::posix_time::time_duration time_delay( unsigned int block, unsigned int laser )
    {
        if(laser<0 ||laser>=lasers_per_block) { COMMA_THROW( comma::exception, "laser id out of range" << laser ); }
        if(block<0||block>=block_count) { COMMA_THROW(comma::exception, "block id out of range"<<block ); }
//        std::cerr<<"block="<<block<<", laser="<<laser<<std::endl;
        double delay = 0;//(time_table[block][laser] );// + timestamps::ethernetOutputDuration * 1e6;
        return boost::posix_time::microseconds( delay);
    }
    static double azimuth(const packet& packet, unsigned int block, unsigned int laser, double angularSpeed )
    {
        // todo: angular speed correction with offset for angular velocity that calibration was measured on
        double rotation = double( packet.blocks[block].rotation() ) / 100;
        double laser_time = 0;//(time_table[block][0] - time_table[block][laser%lasers_per_block]) * 1e-6;
//        double laser_time = 0;
//        std::cerr<<"laser time"<<std::endl;
        double a = rotation + 90;
        a+=angularSpeed * laser_time;
        if( comma::math::less( a, 360 ) ) { if( comma::math::less( a, 0 ) ) { a += 360; } }
        else { a -= 360; }
        return a;
    }
};

//delay in microseconds for each data block, laser
//each upper lasers is fired with a lower laser at the same time, e.g. lasers 0 and 32 fire at the same time then 1 and 33 ...
//each row is for one block and each column is for one laser id 
hdl32::block_time_table hdl32::time_table[block_count]=
{
	{ 542.59, 541.44, 540.29, 539.14, 537.98, 536.83, 535.68, 534.53, 533.38, 532.22, 531.07, 529.92, 528.77, 527.62, 526.46, 525.31, 524.16, 523.01, 521.86, 520.70, 519.55, 518.40, 517.25, 516.10, 514.94, 513.79, 512.64, 511.49, 510.34, 509.18, 508.03, 506.88 },
	{ 496.51, 495.36, 494.21, 493.06, 491.90, 490.75, 489.60, 488.45, 487.30, 486.14, 484.99, 483.84, 482.69, 481.54, 480.38, 479.23, 478.08, 476.93, 475.78, 474.62, 473.47, 472.32, 471.17, 470.02, 468.86, 467.71, 466.56, 465.41, 464.26, 463.10, 461.95, 460.80 },
	{ 450.43, 449.28, 448.13, 446.98, 445.82, 444.67, 443.52, 442.37, 441.22, 440.06, 438.91, 437.76, 436.61, 435.46, 434.30, 433.15, 432.00, 430.85, 429.70, 428.54, 427.39, 426.24, 425.09, 423.94, 422.78, 421.63, 420.48, 419.33, 418.18, 417.02, 415.87, 414.72 },
	{ 404.35, 403.20, 402.05, 400.90, 399.74, 398.59, 397.44, 396.29, 395.14, 393.98, 392.83, 391.68, 390.53, 389.38, 388.22, 387.07, 385.92, 384.77, 383.62, 382.46, 381.31, 380.16, 379.01, 377.86, 376.70, 375.55, 374.40, 373.25, 372.10, 370.94, 369.79, 368.64 },
	{ 358.27, 357.12, 355.97, 354.82, 353.66, 352.51, 351.36, 350.21, 349.06, 347.90, 346.75, 345.60, 344.45, 343.30, 342.14, 340.99, 339.84, 338.69, 337.54, 336.38, 335.23, 334.08, 332.93, 331.78, 330.62, 329.47, 328.32, 327.17, 326.02, 324.86, 323.71, 322.56 },
	{ 312.19, 311.04, 309.89, 308.74, 307.58, 306.43, 305.28, 304.13, 302.98, 301.82, 300.67, 299.52, 298.37, 297.22, 296.06, 294.91, 293.76, 292.61, 291.46, 290.30, 289.15, 288.00, 286.85, 285.70, 284.54, 283.39, 282.24, 281.09, 279.94, 278.78, 277.63, 276.48 },
	{ 266.11, 264.96, 263.81, 262.66, 261.50, 260.35, 259.20, 258.05, 256.90, 255.74, 254.59, 253.44, 252.29, 251.14, 249.98, 248.83, 247.68, 246.53, 245.38, 244.22, 243.07, 241.92, 240.77, 239.62, 238.46, 237.31, 236.16, 235.01, 233.86, 232.70, 231.55, 230.40 },
	{ 220.03, 218.88, 217.73, 216.58, 215.42, 214.27, 213.12, 211.97, 210.82, 209.66, 208.51, 207.36, 206.21, 205.06, 203.90, 202.75, 201.60, 200.45, 199.30, 198.14, 196.99, 195.84, 194.69, 193.54, 192.38, 191.23, 190.08, 188.93, 187.78, 186.62, 185.47, 184.32 },
	{ 173.95, 172.80, 171.65, 170.50, 169.34, 168.19, 167.04, 165.89, 164.74, 163.58, 162.43, 161.28, 160.13, 158.98, 157.82, 156.67, 155.52, 154.37, 153.22, 152.06, 150.91, 149.76, 148.61, 147.46, 146.30, 145.15, 144.00, 142.85, 141.70, 140.54, 139.39, 138.24 },
	{ 127.87, 126.72, 125.57, 124.42, 123.26, 122.11, 120.96, 119.81, 118.66, 117.50, 116.35, 115.20, 114.05, 112.90, 111.74, 110.59, 109.44, 108.29, 107.14, 105.98, 104.83, 103.68, 102.53, 101.38, 100.22, 99.07, 97.92, 96.77, 95.62, 94.46, 93.31, 92.16 },
	{ 81.79, 80.64, 79.49, 78.34, 77.18, 76.03, 74.88, 73.73, 72.58, 71.42, 70.27, 69.12, 67.97, 66.82, 65.66, 64.51, 63.36, 62.21, 61.06, 59.90, 58.75, 57.60, 56.45, 55.30, 54.14, 52.99, 51.84, 50.69, 49.54, 48.38, 47.23, 46.08 },
	{ 35.71, 34.56, 33.41, 32.26, 31.10, 29.95, 28.80, 27.65, 26.50, 25.34, 24.19, 23.04, 21.89, 20.74, 19.58, 18.43, 17.28, 16.13, 14.98, 13.82, 12.67, 11.52, 10.37, 9.22, 8.06, 6.91, 5.76, 4.61, 3.46, 2.30, 1.15, -0.00 },
};

//hdl64_s2_fw_v48::block_time_table hdl64_s2_fw_v48::time_table[block_count]=
//{
//    { 288, 286.74, 285.54, 284.34, 282, 280.74, 279.54, 278.34, 276, 274.74, 273.54, 272.34, 270, 268.74, 267.54, 266.34, 264, 262.74, 261.54, 260.34, 258, 256.74, 255.54, 254.34, 252, 250.74, 249.54, 248.34, 246, 244.74, 243.54, 242.34 },
//    { 288, 286.74, 285.54, 284.34, 282, 280.74, 279.54, 278.34, 276, 274.74, 273.54, 272.34, 270, 268.74, 267.54, 266.34, 264, 262.74, 261.54, 260.34, 258, 256.74, 255.54, 254.34, 252, 250.74, 249.54, 248.34, 246, 244.74, 243.54, 242.34 },
//    { 240, 238.74, 237.54, 236.34, 234, 232.74, 231.54, 230.34, 228, 226.74, 225.54, 224.34, 222, 220.74, 219.54, 218.34, 216, 214.74, 213.54, 212.34, 210, 208.74, 207.54, 206.34, 204, 202.74, 201.54, 200.34, 198, 196.74, 195.54, 194.34 },
//    { 240, 238.74, 237.54, 236.34, 234, 232.74, 231.54, 230.34, 228, 226.74, 225.54, 224.34, 222, 220.74, 219.54, 218.34, 216, 214.74, 213.54, 212.34, 210, 208.74, 207.54, 206.34, 204, 202.74, 201.54, 200.34, 198, 196.74, 195.54, 194.34 },
//    { 192, 190.74, 189.54, 188.34, 186, 184.74, 183.54, 182.34, 180, 178.74, 177.54, 176.34, 174, 172.74, 171.54, 170.34, 168, 166.74, 165.54, 164.34, 162, 160.74, 159.54, 158.34, 156, 154.74, 153.54, 152.34, 150, 148.74, 147.54, 146.34 },
//    { 192, 190.74, 189.54, 188.34, 186, 184.74, 183.54, 182.34, 180, 178.74, 177.54, 176.34, 174, 172.74, 171.54, 170.34, 168, 166.74, 165.54, 164.34, 162, 160.74, 159.54, 158.34, 156, 154.74, 153.54, 152.34, 150, 148.74, 147.54, 146.34 },
//    { 144, 142.74, 141.54, 140.34, 138, 136.74, 135.54, 134.34, 132, 130.74, 129.54, 128.34, 126, 124.74, 123.54, 122.34, 120, 118.74, 117.54, 116.34, 114, 112.74, 111.54, 110.34, 108, 106.74, 105.54, 104.34, 102, 100.74, 99.54, 98.34 },
//    { 144, 142.74, 141.54, 140.34, 138, 136.74, 135.54, 134.34, 132, 130.74, 129.54, 128.34, 126, 124.74, 123.54, 122.34, 120, 118.74, 117.54, 116.34, 114, 112.74, 111.54, 110.34, 108, 106.74, 105.54, 104.34, 102, 100.74, 99.54, 98.34 },
//    { 96, 94.74, 93.54, 92.34, 90, 88.74, 87.54, 86.34, 84, 82.74, 81.54, 80.34, 78, 76.74, 75.54, 74.34, 72, 70.74, 69.54, 68.34, 66, 64.74, 63.54, 62.34, 60, 58.74, 57.54, 56.34, 54, 52.74, 51.54, 50.34 },
//    { 96, 94.74, 93.54, 92.34, 90, 88.74, 87.54, 86.34, 84, 82.74, 81.54, 80.34, 78, 76.74, 75.54, 74.34, 72, 70.74, 69.54, 68.34, 66, 64.74, 63.54, 62.34, 60, 58.74, 57.54, 56.34, 54, 52.74, 51.54, 50.34 },
//    { 48, 46.74, 45.54, 44.34, 42, 40.74, 39.54, 38.34, 36, 34.74, 33.54, 32.34, 30, 28.74, 27.54, 26.34, 24, 22.74, 21.54, 20.34, 18, 16.74, 15.54, 14.34, 12, 10.74, 9.54, 8.34, 6, 4.74, 3.54, 2.34 },
//    { 48, 46.74, 45.54, 44.34, 42, 40.74, 39.54, 38.34, 36, 34.74, 33.54, 32.34, 30, 28.74, 27.54, 26.34, 24, 22.74, 21.54, 20.34, 18, 16.74, 15.54, 14.34, 12, 10.74, 9.54, 8.34, 6, 4.74, 3.54, 2.34 },
//};

static bool is_upper( unsigned int block ) { return ( block & 0x1 ) == 0; }

laser_return get_laser_return( const packet& packet
                             , unsigned int block
                             , unsigned int laser
                             , const boost::posix_time::ptime& timestamp
                             , double angularSpeed
                             , bool legacy)
{
    laser_return r;
    r.id = laser;// + ( is_upper( block ) ? 0 : 32 );
    r.intensity = packet.blocks[block].lasers[laser].intensity();
    r.range = double( packet.blocks[block].lasers[laser].range() ) / 500;

    if (legacy)
    {
        r.timestamp = timestamp + time_offset( block, laser );
        r.azimuth = azimuth( packet, block, laser, angularSpeed );
    }
    else
    {
//        r.timestamp = timestamp - hdl64_s2_fw_v48::time_delay( block, laser );
//        r.azimuth = hdl64_s2_fw_v48::azimuth(packet, block, laser, angularSpeed);
    	r.timestamp = timestamp - hdl32::time_delay( block, laser );
		r.azimuth = hdl32::azimuth(packet, block, laser, angularSpeed);
    }
    return r;
}

double time_span(bool legacy)
{
    if(legacy)
        return double( ( time_offset( 0, 0 ) - time_offset( 11, 0 ) ).total_microseconds() ) / 1e6;
    else
    	return double( (hdl32::time_delay(0,0) - hdl32::time_delay(11,0) ).total_microseconds() ) / 1e6;
//        return double( (hdl64_s2_fw_v48::time_delay(0,0) - hdl64_s2_fw_v48::time_delay(11,0) ).total_microseconds() ) / 1e6;
}

} } } // namespace snark {  namespace velodyne { namespace hdl32 {
