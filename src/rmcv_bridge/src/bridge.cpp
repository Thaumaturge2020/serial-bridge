#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "serial_msg/msg/uint8_array.hpp"
#include "rmcv_bridge/crc.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

using std::cout;
using std::endl;
using namespace std::chrono_literals;

/*
    规则：
    首先有id和text
    [id,text]
    算crc
    [id,text,crc]
    转义规则：
    (char)0x7d -> (char array)0x7f + (char)0x00
    (char)0x7e -> (char array)0x7f + (char)0x01
    (char)0x7f -> (char array)0x7f + (char)0x02
    转义字符串：
    id转义，text转义，crc转义
    [id',text',crc']
    补头尾：
    [0x7d,id',text',crc',0x7e]
*/

namespace rmcv_bridge {
    class MinimalSerialBridge : public rclcpp::Node {
    public :
        std::vector<rclcpp::Publisher<serial_msg::msg::Uint8Array>::SharedPtr> serial_read_list;
        std::vector<rclcpp::Subscription<serial_msg::msg::Uint8Array>::SharedPtr> serial_write_list;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr serial_geometry_suber;

        static constexpr int msg_sub_num = 1;
        static constexpr int msg_pub_num = 1;
        static constexpr int msg_num = 256;
        int fd;
        static constexpr int FULL_LEN = 2048;
        uint8_t read_msg_array[msg_num][FULL_LEN];
        uint8_t write_msg_array[FULL_LEN];
        uint8_t my_temp_buffer[FULL_LEN];
        uint8_t *my_write_ptr[msg_num];
        uint8_t *my_read_ptr[msg_num];
        int my_setup[msg_num];
        
        std::shared_ptr<rclcpp::Node> node1;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_port_reader;

        std::string serial_port;

        int openUart() {
            if(serial_port==""){
            RCLCPP_INFO(this->get_logger(),"can't open this.");
            return -1;
            }
            RCLCPP_INFO(this->get_logger(),"my_str is %s",serial_port.c_str());
            int fd = open(serial_port.c_str(), O_RDWR);
            RCLCPP_INFO(this->get_logger(),"my_fd is %d",fd);
            if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "can't open %s", serial_port.c_str());
            //rclcpp::shutdown();
            return -1;
            }
            termios oldtio = {0};
            termios newtio = {0};
            tcgetattr(fd, &oldtio);
            //设置波特率为115200
            newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
            newtio.c_iflag = 0; // IGNPAR | ICRNL
            newtio.c_oflag = 0;
            newtio.c_lflag = 0; // ICANON
            newtio.c_cc[VTIME] = 0;
            newtio.c_cc[VMIN] = 1;
            tcflush(fd, TCIOFLUSH);
            tcsetattr(fd, TCSANOW, &newtio);

            //设置为非阻塞模式，这个在读串口的时候会用到
            fcntl(fd, F_SETFL, O_NONBLOCK);
            return fd;
        }

        void SerialPort_Subscription(const std_msgs::msg::String msg){
            RCLCPP_INFO(this->get_logger(), "received....");
            serial_port = msg.data;
            // if (fd < 0) {
            //     fd = openUart();
            //     return;
            // }
            RCLCPP_INFO(this->get_logger(), msg.data.c_str());
            return;
        }

        MinimalSerialBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : Node("minimal_rmcv_bridge"),
                  timer_(this->create_wall_timer(std::chrono::milliseconds(5), [this] { behaviour_publish(); }))
            {
            serial_port = "";

            std::stringstream ss;
            std::string str;
            RCLCPP_INFO(this->get_logger(),"test_node_launched");
            using namespace std::placeholders;
            /*for (int i = 0; i < msg_sub_num; ++i) {
                ss.clear(); ss << "/subscription/message";  ss << i;    ss >> str;
                std::function<void(serial_msg::msg::Uint8Array::ConstSharedPtr)> binded =
                        std::bind(&MinimalSerialBridge::behaviour_subscribe, this, _1, i);
                serial_write_list.push_back(this->create_subscription<serial_msg::msg::Uint8Array>(str, 10, binded));
            }*/

            ss.clear(); ss << "/subscription/message0"; ss >> str;
            // std::function<void(geometry_msgs::msg::WrenchStamped::ConstSharedPtr)> binded =
            //             std::bind(&MinimalSerialBridge::velocity_subscribe, this, _1);
            cout << "sub!!!!!!" << endl;
            serial_geometry_suber = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "/cmd_vel", 10, std::bind(&MinimalSerialBridge::velocity_subscribe, this, _1));

            for (int i = 0; i < msg_pub_num; ++i){
                ss.clear(); ss << "/publisher/message";     ss << i;    ss >> str;
                serial_read_list.push_back(this->create_publisher<serial_msg::msg::Uint8Array>(str, 10));
            }

            serial_port_reader = this->create_subscription<std_msgs::msg::String>("serial_id",10,std::bind(&MinimalSerialBridge::SerialPort_Subscription, this, _1));
            
            fd = openUart();
            memset(my_setup,0,sizeof(my_setup));
            cout << "construct!!!!" << endl;
        }

        void velocity_subscribe(geometry_msgs::msg::WrenchStamped::UniquePtr msg){
            cout << "success!!!!" << endl;
            double velo_x = msg->wrench.force.x,  //
                velo_y = msg->wrench.force.y,     //
                yaw_msg = msg->wrench.torque.z;

            cout << "velo_x  " << velo_x << "velo_y  " << velo_y << "yaw_msg  " << yaw_msg << endl;

            uint8_t vel_msg[24] = {0};
            memcpy(vel_msg,&velo_x,sizeof(double));
            memcpy(vel_msg+8,&velo_y,sizeof(double));
            memcpy(vel_msg+16,&yaw_msg,sizeof(double));
            serial_msg::msg::Uint8Array vel_msg_array;
            vel_msg_array.data.resize(24);
            memcpy(vel_msg_array.data.data(),vel_msg,sizeof(uint8_t)*24);
            behaviour_subscribe(std::make_shared<serial_msg::msg::Uint8Array>(vel_msg_array),0x00);
            RCLCPP_INFO(this->get_logger(),"velocity published %s",vel_msg_array.data.data());
            return;
        }

        void behaviour_publish() {
            if(fd < 0){
                fd = openUart();
                return;
            }
            long int ret = read(fd, my_temp_buffer, FULL_LEN);
            uint8_t *p_str = my_temp_buffer;
            // RCLCPP_INFO(this->get_logger(),"my_ret:%d",ret);
            if(ret < 0) return;
            RCLCPP_INFO(this->get_logger(),"%ld",ret);
            // RCLCPP_INFO(this->get_logger(),"%s",my_temp_buffer);
            static int type = -1;

            static int state = 0;

            /*
                header : 0x7d type(instruction translation)
                text : text(instruction translation)
                tail : 0x7e
            */

            /*
                state 0:end
                state 1:start(type)
                state 2:start(text)
                state 3:translate the character (type)
                state 4:translate the character (text)
            */

            for (int i = 0; i < ret; ++i){
                RCLCPP_INFO(this->get_logger(),"%02x",*p_str);
                char c = *p_str;
                switch (c){
                    case 0x7d:state = 1;
                        break;
                    case 0x7e:{
                        state = 0;
                        if(my_write_ptr[type] == nullptr) {RCLCPP_ERROR(this->get_logger(),"fail this time!!"); state = 0; break;}
                        size_t now_len = my_write_ptr[type]-read_msg_array[type];
                        if(now_len <= 2) RCLCPP_INFO(this->get_logger(),"ERROR_buffer");
                        uint16_t crccode_16 = crc_16_with_id(read_msg_array[type], now_len - 2,(unsigned char)type);
                        my_write_ptr[type] = nullptr;
                        
                        uint16_t crccode_16_rec = read_msg_array[type][now_len-1]*256 + read_msg_array[type][now_len-2];
                        
                        RCLCPP_INFO(this->get_logger(),"%s",read_msg_array[type]);

                        RCLCPP_INFO(this->get_logger(),"%ld %ld",crccode_16,crccode_16_rec);
                        
                        if(crccode_16==crccode_16_rec){
                            now_len -= 2;
                            serial_msg::msg::Uint8Array msg;
                            msg.data.resize(now_len);
                            RCLCPP_INFO(this->get_logger(),"%ld %d",now_len,type);
                            memcpy(msg.data.data(),read_msg_array[type], now_len);
                            serial_read_list[type]->publish(msg);
                            type = -1;
                        }
                    }
                        break;
                    
                    case 0x7f:
                        switch (state){
                            case 1:state = 3;break;
                            case 2:state = 4;break;
                            default:RCLCPP_ERROR(this->get_logger(),"translation ERROR!!");state = 4;break;
                        }
                        break;
                
                    default:
                        switch (state){
                            case 0:RCLCPP_ERROR(this->get_logger(),"start ERROR!!");break;
                            case 1:type = c;
                                    my_write_ptr[type] = read_msg_array[type];
                                    state = 2;break;
                            case 2:
                                    if(my_write_ptr[type] == nullptr) {RCLCPP_ERROR(this->get_logger(),"fail this time!!"); state = 0; break;}
                                    *(my_write_ptr[type]++) = c;
                                    state = 2;break;
                            case 3:type = 0x7d+c;
                                    my_write_ptr[type] = read_msg_array[type];
                                    state = 2;break;
                            case 4:
                                    if(my_write_ptr[type] == nullptr) {RCLCPP_ERROR(this->get_logger(),"fail this time!!"); state = 0; break;}
                                    *(my_write_ptr[type]++) = 0x7d+c;
                                    state = 2;break;
                            default:RCLCPP_ERROR(this->get_logger(),"received ERROR!!"); state = 0; break;
                        }
                        break;
                }
                ++p_str;
            }
        }

#define ADD(target_char)\
            { \
                *(my_buffer_ptr++) = target_char; \
            }

        void behaviour_subscribe(serial_msg::msg::Uint8Array::ConstSharedPtr msg, int type) {
            if(fd < 0){
                fd = openUart();
                return;
            }
            RCLCPP_INFO(this->get_logger(),"this can be dealed by me...");
            int len = (int)sizeof(uint8_t) * msg->data.size();
            uint8_t* now_msg = new uint8_t[2*len+5];
            now_msg[0] = type;
            memcpy(now_msg+1, msg->data.data(), len);
            
            len += 1;//for modify the length;
            std::cout << rmcv_bridge::crc_tab16[1] << "---------------------------------crc16[1]\n\n" << std::endl;
            uint16_t crccode_16 = crc_16(now_msg, len);
            uint8_t *p_str = now_msg;
            uint8_t *my_buffer_ptr = write_msg_array;   
            const uint8_t *head_ptr = write_msg_array;
            
            //0:normal
            //1:translation

            ADD(0x7d);

            for(int i = 0;i < len ;++i){
                char c = *p_str;
                switch (c){
                    case 0x7d: ADD(0x7f);ADD(0x00);break;
                    case 0x7e: ADD(0x7f);ADD(0x01);break;
                    case 0x7f: ADD(0x7f);ADD(0x02);break;
                    default: ADD(c);
                    break;
                }
                ++p_str;
            }

            uint8_t crc1 = (crccode_16 >> 8);
            uint8_t crc2 = (crccode_16 & 255);


            switch (crc2){
                case 0x7d: ADD(0x7f);ADD(0x00);break;
                case 0x7e: ADD(0x7f);ADD(0x01);break;
                case 0x7f: ADD(0x7f);ADD(0x02);break;
                default: ADD(crc2);
                break;
            }

            switch (crc1){
                case 0x7d: ADD(0x7f);ADD(0x00);break;
                case 0x7e: ADD(0x7f);ADD(0x01);break;
                case 0x7f: ADD(0x7f);ADD(0x02);break;
                default: ADD(crc1);
                break;
            }
            ADD(0x7e);

            write(fd,head_ptr,my_buffer_ptr - head_ptr);
            RCLCPP_INFO(this->get_logger(),"succeed.fd is %d ",fd);
            delete(now_msg);
            return;
        }
#undef ADD

    private :
        rclcpp::TimerBase::SharedPtr timer_;
    };
}


RCLCPP_COMPONENTS_REGISTER_NODE(rmcv_bridge::MinimalSerialBridge)