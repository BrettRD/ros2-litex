#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <libusb-1.0/libusb.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "example_interfaces/msg/u_int32_multi_array.hpp"



using namespace std::chrono_literals;


#define CTRL_IN			(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_OTHER | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_OTHER | LIBUSB_ENDPOINT_OUT)



#define CSR_BASE 0x0L
#define CSR_PPM_INPUT_BASE (CSR_BASE + 0x3000L)
#define CSR_PPM_INPUT_CHANNELS 2
#define CSR_PPM_OUTPUT_BASE (CSR_BASE + 0x3800L)
#define CSR_PPM_OUTPUT_CHANNELS 2
#define CSR_PWM_INPUT_BASE (CSR_BASE + 0x7800L)
#define CSR_TRIV_REG_BASE (CSR_BASE + 0x8000L)



class WishbonePublisher : public rclcpp::Node
{
  public:
    WishbonePublisher()
    : Node("minimal_publisher"), count_(0)
    {

      uint16_t idVendor=0x1209;
      uint16_t idProduct=0x5bf0;



      triv_reg_pub = this->create_publisher<example_interfaces::msg::UInt8>("triv_reg", 10);
      ppm_in_pub = this->create_publisher<example_interfaces::msg::UInt32MultiArray>("ppm_in", 10);



      if (0 <= libusb_init(NULL))
      {
        devh = libusb_open_device_with_vid_pid(NULL, idVendor, idProduct);
        if(devh != NULL)
        {
          if(0 <= libusb_claim_interface(devh, 0))
          {
            RCLCPP_INFO(this->get_logger(), "claimed interface");


            this->triv_reg_timer = this->create_wall_timer(500ms, std::bind(&WishbonePublisher::triv_reg_callback, this));
            this->ppm_in_timer = this->create_wall_timer(200ms, std::bind(&WishbonePublisher::ppm_in_callback, this));

          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "could not claim interface");
            libusb_close(devh);
            devh = NULL;
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "No device");
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "libusb did not init");
      }
    }

    ~WishbonePublisher()
    {
      if(devh)
      {
        libusb_release_interface(devh, 0);
        libusb_close(devh);
      }
    }



  private:


    int wishbone_poke(uint32_t address, uint32_t* data)
    {
      int r;
      uint8_t bRequest = 0x00;
      uint16_t addr_h, addr_l;
      unsigned char buf[4];
      uint16_t len = 4;
      addr_h = address >> 16;
      addr_l = address & 0xFFFF;

      for(int i=0; i<4; i++)
      {
        buf[i] = (*data >> (8*i)) & 0xFF;
      }

      r = libusb_control_transfer(devh, CTRL_OUT, bRequest, addr_l, addr_h, buf, len, 0);
      if (r < 0) {
        RCLCPP_ERROR(this->get_logger(), "poke error: %s", libusb_error_name(r));
      }
      else if ((unsigned int) r < len) {
        RCLCPP_ERROR(this->get_logger(), "poke error: short write (%d)", r);
      }
      return r;
    }

    int wishbone_peek(uint32_t address, uint32_t* data)
    {
      int r;
      uint8_t bRequest = 0x00;
      uint16_t addr_h, addr_l;
      unsigned char buf[4];
      uint16_t len = 4;
      addr_h = address >> 16;
      addr_l = address & 0xFFFF;

      r = libusb_control_transfer(devh, CTRL_IN, bRequest, addr_l, addr_h, buf, len, 0);
      if (r < 0) {
        RCLCPP_ERROR(this->get_logger(), "peek error: %s", libusb_error_name(r));
      }
      else if ((unsigned int) r < len) {
        RCLCPP_ERROR(this->get_logger(), "peek error: short read (%d)", r);
      }
      else
      {
        *data = 0;
        for(int i=0; i<4; i++)
        {
          *data += ((uint32_t)(buf[i])) << (8*i);
        }
      }

      return r;
    }




    void read_ppm(uint32_t* data, int count)
    {
      if(count > CSR_PPM_INPUT_CHANNELS) count = CSR_PPM_INPUT_CHANNELS;
      for(int i=0; i<count; i++)
      {
        wishbone_peek(CSR_PPM_INPUT_BASE + (4*i), &(data[i]));
      }
    }

    void write_ppm(uint32_t* data, int count)
    {
      if(count > CSR_PPM_OUTPUT_CHANNELS) count = CSR_PPM_OUTPUT_CHANNELS;
      for(int i=0; i<count; i++)
      {
        wishbone_poke(CSR_PPM_OUTPUT_BASE + (4*i), &(data[i]));
      }
    }

    void read_pwm(uint32_t* data)
    {
        wishbone_peek(CSR_PWM_INPUT_BASE, data);
    }

    void read_triv_reg(uint32_t* data)
    {
        wishbone_peek(CSR_TRIV_REG_BASE, data);
    }


    void triv_reg_callback()
    {
      uint32_t data = 0;
      auto message = example_interfaces::msg::UInt8();
      read_triv_reg(&data);
      message.data = data & 0xff;
      triv_reg_pub->publish(message);
    }


    void ppm_in_callback()
    {
      auto message = example_interfaces::msg::UInt32MultiArray();
      message.data.resize(CSR_PPM_INPUT_CHANNELS);
      read_ppm(message.data.data(), CSR_PPM_INPUT_CHANNELS);
      ppm_in_pub->publish(message);
    }



    struct libusb_device_handle *devh = NULL;
    rclcpp::TimerBase::SharedPtr triv_reg_timer;
    rclcpp::TimerBase::SharedPtr ppm_in_timer;
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr triv_reg_pub;
    rclcpp::Publisher<example_interfaces::msg::UInt32MultiArray>::SharedPtr ppm_in_pub;

    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WishbonePublisher>());
    rclcpp::shutdown();
    return 0;
  }