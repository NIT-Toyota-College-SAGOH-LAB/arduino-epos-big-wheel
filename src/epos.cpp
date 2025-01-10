#define DEBUG 1

#include <queue>
#include <chrono>
#include <ctime>
#include <ncurses.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace constants
{
	// パワーアシスト　パラメータ
	static constexpr double Kav = 2.0; // 前後方向アシスト比率
	static constexpr double Kat = 3.0; // 旋回方向アシスト比率
	static constexpr double R = 52.5 * pow(10.0, -2.0);  // 車輪の半径(m)
	static constexpr double Lh = 0.546 / 2; // 力覚センサ, 回転中心までの距離
	static constexpr double Lw = 0.81; // 横幅
	static constexpr double i = 156.0 * 2.0; // 伝達比(ギヤヘッド＋プーリ）
	static constexpr double Km = 30.2 * pow(10.0, -3.0); // トルク定数(Nm/A)
}

namespace msg_constants
{
	constexpr static auto SECTION_DATA_LENGTH = 8u;
	constexpr static std::array<char, 3> HEADERS = {'G', 'H', 'I'};
	const static std::string END_CODE("Z");
}

// first is raw data
// second is encoder node number
using encoder_node_data_t = std_msgs::msg::String::SharedPtr;

struct robot_state
{
	int16_t wheel_L_current_mA = 0;
	int16_t wheel_R_current_mA = 0;

	double wheel_L_angular_velocity_rad_s = 0.0;
	double wheel_R_angular_velocity_rad_s = 0.0;

	double motor1_speed_average_rpm = 0.0;
	double motor2_speed_average_rpm = 0.0;

	int16_t ref1_current_mA = 0;
	int16_t ref2_current_mA = 0;

	static constexpr double ENCODER_RESOLUTION = 500 * 4;

	static constexpr double sampling_rate_Hz = 20.0;

	static constexpr int16_t CONTINUOS_CURRENT_mA = 5000;
	static constexpr int16_t PEAK_CURRENT_mA = CONTINUOS_CURRENT_mA * 2;
};

class arduino_epos : public rclcpp::Node
{
	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::TimerBase::SharedPtr m_fast_timer;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_state_sub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_transmit_pub;
	robot_state m_state;

	std::queue<encoder_node_data_t> m_queue_msg;

	std::array<uint32_t, 3> m_pre_encoder_states;

	std_msgs::msg::String::SharedPtr m_buffer;

	std::chrono::_V2::system_clock::time_point m_pre_time;
	

	void timer_callback();
	void fast_timer_callback();
    void state_callback(const std_msgs::msg::String::SharedPtr msg);

	void encoder_node();

public:
	arduino_epos();
	~arduino_epos();
};

arduino_epos::arduino_epos() : Node("arduino_epos")
{
	this->m_timer = this->create_wall_timer(50ms, std::bind(&arduino_epos::timer_callback, this));
	this->m_fast_timer = this->create_wall_timer(1ms, std::bind(&arduino_epos::fast_timer_callback, this));
	this->m_state_sub = this->create_subscription<std_msgs::msg::String>(
		"epos_recv", 10, std::bind(&arduino_epos::state_callback, this, std::placeholders::_1));
	this->m_transmit_pub = this->create_publisher<std_msgs::msg::String>("epos_transmit", 10);

	this->m_pre_time = std::chrono::system_clock::now();
}

arduino_epos::~arduino_epos()
{
	std::string end_msg;

	static constexpr char header1 = 'h';
	static constexpr char header2 = 'g';
	static const std::string end_code("z\n");

	end_msg += header1;
	end_msg += "0000";
	end_msg += header2;
	end_msg += "0000";
	end_msg += end_code;

	std_msgs::msg::String msg;

	msg.data = end_msg;

	this->m_transmit_pub->publish(msg);
}

void arduino_epos::timer_callback()
{
	auto ch = getch();

	double input;
	if(ch == 'w')
	{
		input = 200.;
	}
	else if(ch == 's')
	{
		input = -200.;
	}
	else
	{
		input = 0.;
	}

	int16_t wheel_val = input;

	std::string write_buffer("");

	static constexpr char header1 = 'h';
	static constexpr char header2 = 'g';
	static const std::string end_code("z\n");
	
	uint16_t u_wheel_val = wheel_val;

	write_buffer += header1;
	if (u_wheel_val < 0x1000)
	{
		write_buffer += '0';
	}
	if (u_wheel_val < 0x0100)
	{
		write_buffer += '0';
	}
	if (u_wheel_val < 0x0010)
	{
		write_buffer += '0';
	}
	
	std::stringstream write_one_digit_data;
	write_one_digit_data << std::hex << u_wheel_val;
	write_buffer += write_one_digit_data.str();

    write_buffer += header2;
	write_buffer += write_one_digit_data.str();

	write_buffer += end_code;

	std_msgs::msg::String msg;
	msg.data = write_buffer;

	this->m_transmit_pub->publish(msg);
	
}

void arduino_epos::fast_timer_callback()
{
	encoder_node();
}

void arduino_epos::state_callback(const std_msgs::msg::String::SharedPtr msg)
{
	// カノニカルモードでやってるからこないと困る
	if(msg->data.size() != msg_constants::HEADERS.size() * (msg_constants::SECTION_DATA_LENGTH + 1) + msg_constants::END_CODE.size())
	{
		RCLCPP_INFO(this->get_logger(), "msg size: %d, necessary data size: %d", msg->data.size(), msg_constants::HEADERS.size() * (msg_constants::SECTION_DATA_LENGTH + 1) + msg_constants::END_CODE.size());
		RCLCPP_INFO(this->get_logger(), "Invalid data length");
		return;
	}

	if(msg->data[0] != 'G' || msg->data[9] != 'H' || msg->data[18] != 'I')
	{
		RCLCPP_INFO(this->get_logger(), "Invalid data header");
		return;
	}

	m_queue_msg.push(msg);
	
}

void arduino_epos::encoder_node()
{
	if(m_queue_msg.size() == 0)
	{
		return;
	}
	using std::chrono::system_clock;
	using namespace std::chrono_literals;

    encoder_node_data_t msg = m_queue_msg.front();
	m_queue_msg.pop();

	// decode and calculate wheel angular velocity
	static double pre_enc_rotations[2] = {0.0, 0.0};
	double enc_rotations[2] = {0.0, 0.0};
	double weight_rotation = 0;

	for(int enc_rot_num = 0; enc_rot_num < 2; enc_rot_num++)
	{
		uint64_t rot_digit = 0;
		for(int i = 0; i < 8; i++)
		{
			rot_digit |= (uint64_t)msg->data[1 + enc_rot_num * 9 + i] << ((7-i) * 8);
		}

		int64_t rot_i_digit = rot_digit;

		constexpr double REDUCTION_RATIO = 156.0/2;

		enc_rotations[enc_rot_num] = rot_i_digit / robot_state::ENCODER_RESOLUTION / REDUCTION_RATIO * 2.0 * M_PI;
	}
	auto duration = std::chrono::system_clock::now() - m_pre_time;

	double duration_sec = std::chrono::duration<double> (duration).count();
	m_state.wheel_L_angular_velocity_rad_s = (enc_rotations[0] - pre_enc_rotations[0]) / duration_sec;
	m_state.wheel_R_angular_velocity_rad_s = (enc_rotations[1] - pre_enc_rotations[1]) / duration_sec;

	for(int i = 0; i < 2; i++)
	{
		pre_enc_rotations[i] = enc_rotations[i];
	}

	RCLCPP_INFO(this->get_logger(), "R_angular_velocity: %lf", m_state.wheel_R_angular_velocity_rad_s);
	RCLCPP_INFO(this->get_logger(), "L_angular_velocity: %lf", m_state.wheel_L_angular_velocity_rad_s);

	m_pre_time = std::chrono::system_clock::now();
}

int main(int argc, char ** argv)
{
	initscr();
	cbreak();
	noecho();
	scrollok(stdscr, TRUE);
	nodelay(stdscr, TRUE);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<arduino_epos>());
	rclcpp::shutdown();

	endwin();
}
