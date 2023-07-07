//
// Created by YuanSu on 23-April-10.
//

#ifndef _DATAPACKETHANDEXO_H_
#define _DATAPACKETHANDEXO_H_

#include <chrono>
#include <Eigen/Eigen>

namespace myDeviceUDP {
    namespace datapacket {
        struct ToM2slave {
            unsigned int ID;			// to test the delay time and the packet losing rate
            double time_stamping;

        };


        struct slave2ToM {
            unsigned int ID;
            double time_stamping;

            float thumb_posi[3];
            float thumb_ori[4];
            float index_posi[3];
            float index_ori[4];
            float middle_posi[3];
            float middle_ori[4];
            float thumb_angle[4];
            float index_angle[4];
            float middle_angle[4];

        };

        struct stm32_to_slave{
            float sensor_data[30];
        };


    }
}



#endif //_DATAPACKETHANDEXO_H_
