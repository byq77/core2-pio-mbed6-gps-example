#include "mbed.h"
#include "gps.h"
#include "ros.h"
#include "sensor_msgs/NavSatFix.h"

#define GPS1_TX SENS3_PIN3
#define GPS1_RX SENS3_PIN4

#define GPS2_TX EXT_PIN7
#define GPS2_RX EXT_PIN6

constexpr std::chrono::milliseconds GPS_UPDATE_DT{100ms};
constexpr std::chrono::milliseconds SPIN_DELAY{10ms};

class RosGps : private ros::NodeHandle
{
public:

    RosGps()
    {
        // initialise ros node
        initNode();

        _gps1_msg.header.frame_id = "gps1";
        _gps2_msg.header.frame_id = "gps2";

        // advertise publisher
        advertise(_gps1_pub);
        advertise(_gps2_pub);
    };

    /**
     * @brief Convert raw longitude and latitude data to ros format
     * 
     * @param value raw longitude or latitude
     * @param nsew n,s,e,w
     * @return converted value 
     */
    double convertToRosSatPos(double value, char nsew)
    {
        switch(nsew)
        {
            case 'n':
            case 'N':
            case 'e':
            case 'E':
                return value;
            default:
                return (-1)*value;
        }
    }

    /**
     * @brief Run the node.
     */
    void spin()
    {
        auto time = Kernel::Clock::now();
        auto gps_check_time = time + GPS_UPDATE_DT;

        while (1)
        {
            time = Kernel::Clock::now();

            if(time >= gps_check_time)
            {
                if(_gps1.update())
                {
                    loginfo("GPS1:");
                    loginfo(_gps1.getLastMsg());
                    _gps1_msg.latitude = convertToRosSatPos(_gps1.latitude, _gps1.ns);
                    _gps1_msg.longitude = convertToRosSatPos(_gps1.longitude, _gps1.ew);
                    sensor_msgs::NavSatStatus nav_sat_status;
                    nav_sat_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                    nav_sat_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; 
                    _gps1_msg.status = nav_sat_status;

                    _gps1_pub.publish(&_gps1_msg);
                }

                if(_gps2.update())
                {
                    loginfo("GPS2:");
                    loginfo(_gps1.getLastMsg());
                    _gps2_msg.latitude = convertToRosSatPos(_gps2.latitude, _gps2.ns);
                    _gps2_msg.longitude = convertToRosSatPos(_gps2.longitude, _gps2.ew);
                    sensor_msgs::NavSatStatus nav_sat_status;
                    nav_sat_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                    nav_sat_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; 
                    _gps2_msg.status = nav_sat_status;

                    _gps2_pub.publish(&_gps2_msg);
                }

                gps_check_time+=GPS_UPDATE_DT;
            }

            spinOnce();
            ThisThread::sleep_until(time + SPIN_DELAY);
        }
    };

private:
    DigitalOut _sens_power_{SENS_POWER_ON, 1}; // POWER SENSOR ON

    DigitalOut _gps1_comm_indicator_{LED1, 0};
    DigitalOut _gps2_comm_indicator_{LED2, 0};

    GPS _gps1{GPS1_TX, GPS1_RX};
    GPS _gps2{GPS2_TX, GPS2_RX};

    sensor_msgs::NavSatFix _gps1_msg{};
    sensor_msgs::NavSatFix _gps2_msg{};

    ros::Publisher _gps1_pub{"gps1", &_gps1_msg};
    ros::Publisher _gps2_pub{"gps2", &_gps2_msg};
};

int main()
{
    RosGps test;
    test.spin();

    return 0;
}