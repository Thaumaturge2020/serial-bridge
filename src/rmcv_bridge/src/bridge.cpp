#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "serial_msg/msg/uint8_array.hpp"
#include "rmcv_bridge/crc.hpp"


using namespace std::chrono_literals;

namespace rmcv_bridge {
    class MinimalSerialBridge : public rclcpp::Node {
    public :
        std::vector<rclcpp::Publisher<serial_msg::msg::Uint8Array>::SharedPtr> serial_read_list;
        std::vector<rclcpp::Subscription<serial_msg::msg::Uint8Array>::SharedPtr> serial_write_list;
        static constexpr int msg_num = 20;
        int fd;
        static constexpr int FULL_LEN = 2048;
        uint8_t read_msg_array[msg_num][FULL_LEN];
        uint8_t write_msg_array[FULL_LEN];
        uint8_t my_temp_buffer[FULL_LEN];
        uint8_t *my_write_ptr[msg_num];
        uint8_t *my_read_ptr[msg_num];

        int openUart(const char *Uart_str) {
            int fd = open(Uart_str, O_RDWR | O_NOCTTY);
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

        MinimalSerialBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : Node("minimal_rmcv_bridge"),
                  timer_(this->create_wall_timer(std::chrono::milliseconds(1), [this] { behaviour_publish(); })) {
            std::stringstream ss;
            std::string str;
            using namespace std::placeholders;
            for (int i = 0; i < msg_num; ++i) {
                ss.clear();
                ss << "/sublisher_message/";
                ss << i;
                ss >> str;
                std::function<void(serial_msg::msg::Uint8Array::ConstSharedPtr)> binded =
                        std::bind(&MinimalSerialBridge::behaviour_subscribe, this, _1, i);
                serial_write_list.push_back(
                        this->create_subscription<serial_msg::msg::Uint8Array>(
                                str, 10, binded));

                ss.clear();
                ss << "/pubscriber_message/";
                ss << i;
                ss >> str;
                serial_read_list.push_back(this->create_publisher<serial_msg::msg::Uint8Array>(str, 10));
            }
            fd = openUart("/dev/ttyS0");
        }

        void behaviour_publish() {
            long int ret = read(fd, my_temp_buffer, FULL_LEN);
            uint8_t *p_str = my_temp_buffer;
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "buffer incorrect!!!");
                rclcpp::shutdown();
                return;
            }
            static int setup = 0, type = 0;
            for (int i = 0; i < ret; ++i) {
                if (setup == 1) {
                    type = *p_str;
                    setup = 0;
                    my_write_ptr[type] = read_msg_array[type];
                }
                if (*p_str == 0x7e) {
                    setup = 1;
                } else if (*p_str == 0x7f) {
                    serial_msg::msg::Uint8Array msg;
                    size_t now_len = my_write_ptr[type] - read_msg_array[type];
                    msg.data.resize(now_len);
                    memcpy(msg.data.data(), read_msg_array[type], now_len);
                    serial_read_list[type]->publish(msg);

                } else if (*p_str == 0x7d) {
                    ++p_str;
                    *(my_write_ptr[type]++) = (*p_str + 0x7d);
                } else {
                    *(my_write_ptr[type]++) = *p_str;
                }
                ++p_str;
            }
        }

#define ADD(target_char)\
            { \
                *(my_buffer_ptr++) = target_char; \
            }

        void behaviour_subscribe(serial_msg::msg::Uint8Array::ConstSharedPtr msg, int type) {
            int len = (int)sizeof(uint8_t) * msg->data.size();
            uint8_t* now_msg = new uint8_t[len+len];
            memcpy(now_msg, msg->data.data(), len);
            uint16_t crccode_16 = crc_16(now_msg, len);
            uint8_t *pstr = now_msg;
            uint8_t *my_buffer_ptr = write_msg_array;
            const uint8_t *head_ptr = write_msg_array,
                    *tail_ptr = write_msg_array + FULL_LEN;

            ADD(0x7e);ADD((uint8_t) type);
            for (int i = 0; i < len; ++i) {
                if (*pstr == 0x7f || *pstr == 0x7e || *pstr == 0x7d) {
                    ADD(0x7d);ADD(*pstr - 0x7d);
                } else {
                    ADD(*pstr);
                }
                ++pstr;
            }
            ADD((uint8_t) (crccode_16 >> 8));ADD((uint8_t)(crccode_16 & 255));ADD(0x7f);
            write(fd,head_ptr,my_buffer_ptr - head_ptr);
            delete(now_msg);
        }
#undef ADD

    private :
        rclcpp::TimerBase::SharedPtr timer_;
    };
}


RCLCPP_COMPONENTS_REGISTER_NODE(rmcv_bridge::MinimalSerialBridge)