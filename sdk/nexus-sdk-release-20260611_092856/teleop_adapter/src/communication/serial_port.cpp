/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/src/communication/serial_port.cpp
 * @Description: 串口通信实现，提供串口打开、关闭、读写等基本操作
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/communication/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <cstring>
#include <cerrno>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Constructor & Destructor
 ******************************************************************************/

/**
 * @brief  构造函数
 * @param  null
 * @retval null
 */
SerialPort::SerialPort() 
    : fd_(-1), is_open_(false) {
}

/**
 * @brief  析构函数
 * @param  null
 * @retval null
 */
SerialPort::~SerialPort() {
    close();
}

/*******************************************************************************
 * Public Methods
 ******************************************************************************/

/**
 * @brief  打开串口（带配置）
 * @param  config 串口配置
 * @retval 成功返回true，否则返回false
 */
bool SerialPort::open(const Config& config) {
    // 保存配置
    config_ = config;
    // 调用无参 open()
    return open();
}

/**
 * @brief  打开串口
 * @param  null
 * @retval 成功返回true，否则返回false
 */
bool SerialPort::open() {
    std::lock_guard<std::mutex> lock(mutex_);

    // 如果已经打开，先关闭
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        is_open_ = false;
    }

    // 打开串口设备（非阻塞模式）
    fd_ = ::open(config_.port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        setError("Failed to open port " + config_.port_name + ": " + std::strerror(errno));
        return false;
    }

    // 配置串口参数
    if (!configure()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 清空串口缓冲区（初始化时清除可能存在的旧数据）
    ::tcflush(fd_, TCIOFLUSH);

    is_open_ = true;
    last_error_.clear();
    return true;
}

/**
 * @brief  关闭串口
 * @param  null
 * @retval null
 */
void SerialPort::close() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        is_open_ = false;
    }
}

/**
 * @brief  检查串口是否打开
 * @param  null
 * @retval 已打开返回true，否则返回false
 */
bool SerialPort::isOpen() const {
    return is_open_.load();
}

/**
 * @brief  从串口读取数据
 * @param  buffer 数据缓冲区
 * @param  length 缓冲区长度
 * @param  timeout_ms 超时时间（毫秒）
 * @retval 读取的字节数，错误返回-1
 */
int SerialPort::read(uint8_t* buffer, size_t length, int timeout_ms) {
    if (!isOpen() || buffer == nullptr || length == 0) {
        setError("Invalid read parameters or port not open");
        return -1;
    }

    // 确定超时时间
    int timeout = (timeout_ms == -1) ? config_.timeout_ms : timeout_ms;

    fd_set read_fds;
    struct timeval tv;

    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    // 设置超时
    if (timeout >= 0) {
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
    }

    // 使用 select 等待数据
    int ret = ::select(fd_ + 1, &read_fds, nullptr, nullptr, 
                       (timeout >= 0) ? &tv : nullptr);

    if (ret < 0) {
        if (errno == EINTR) {
            // 被信号中断，返回 0 表示暂无数据
            return 0;
        }
        setError("Select error: " + std::string(std::strerror(errno)));
        return -1;
    } else if (ret == 0) {
        // 超时，无数据可读
        return 0;
    }

    // 有数据可读
    if (FD_ISSET(fd_, &read_fds)) {
        // 一次性读取所有可用数据，不使用循环
        ssize_t n = ::read(fd_, buffer, length);

        if (n > 0) {
            return static_cast<int>(n);
        } else if (n == 0) {
            // 无数据
            return 0;
        } else {
            // n < 0，发生错误
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 暂时无数据
                return 0;
            } else if (errno == EINTR) {
                // 被信号中断
                return 0;
            } else {
                setError("Read error: " + std::string(std::strerror(errno)));
                return -1;
            }
        }
    }

    return 0;
}

/**
 * @brief  向串口写入数据
 * @param  data 数据指针
 * @param  length 数据长度
 * @retval 写入的字节数，错误返回-1
 */
int SerialPort::write(const uint8_t* data, size_t length) {
    if (!isOpen() || data == nullptr || length == 0) {
        setError("Invalid write parameters or port not open");
        return -1;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 一次性写入，不重试
    ssize_t n = ::write(fd_, data, length);

    if (n < 0) {
        setError("Write error: " + std::string(std::strerror(errno)));
        return -1;
    }

    // 检查是否完整写入
    if (static_cast<size_t>(n) < length) {
        setError("Write incomplete: " + std::to_string(n) + "/" + 
                 std::to_string(length) + " bytes written");
        return -1;
    }

    return static_cast<int>(n);
}

/**
 * @brief  获取可读字节数
 * @param  null
 * @retval 可读字节数，错误返回-1
 */
int SerialPort::getBytesAvailable() const {
    if (!isOpen()) {
        return -1;
    }

    int bytes_available = 0;
    if (::ioctl(fd_, FIONREAD, &bytes_available) < 0) {
        return -1;
    }

    return bytes_available;
}

/**
 * @brief  清空缓冲区
 * @param  null
 * @retval null
 */
void SerialPort::clearBuffer() {
    if (!isOpen()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // 使用 tcflush 清空内核缓冲区
    ::tcflush(fd_, TCIOFLUSH);
}

/**
 * @brief  获取最后的错误信息
 * @param  null
 * @retval 错误信息字符串
 */
std::string SerialPort::getLastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

/**
 * @brief  获取通信类型
 * @param  null
 * @retval 类型字符串
 */
std::string SerialPort::getType() const {
    return "Serial";
}

/**
 * @brief  获取串口配置
 * @param  null
 * @retval 串口配置结构体
 */
SerialPort::Config SerialPort::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

/*******************************************************************************
 * Private Methods
 ******************************************************************************/

/**
 * @brief  配置串口参数
 * @param  null
 * @retval 成功返回true，否则返回false
 */
bool SerialPort::configure() {
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    // 获取当前配置
    if (::tcgetattr(fd_, &tty) != 0) {
        setError("Failed to get terminal attributes: " + std::string(std::strerror(errno)));
        return false;
    }

    // 设置波特率
    int baud_const = baudrateToConst(config_.baudrate);
    if (baud_const == 0) {
        setError("Unsupported baudrate: " + std::to_string(config_.baudrate));
        return false;
    }
    ::cfsetospeed(&tty, baud_const);
    ::cfsetispeed(&tty, baud_const);

    // 设置数据位
    tty.c_cflag &= ~CSIZE;
    switch (config_.data_bits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default:
            setError("Invalid data bits: " + std::to_string(config_.data_bits));
            return false;
    }

    // 设置停止位
    if (config_.stop_bits == 1) {
        tty.c_cflag &= ~CSTOPB;
    } else if (config_.stop_bits == 2) {
        tty.c_cflag |= CSTOPB;
    } else {
        setError("Invalid stop bits: " + std::to_string(config_.stop_bits));
        return false;
    }

    // 设置校验位
    switch (config_.parity) {
        case 'N':  // 无校验
            tty.c_cflag &= ~PARENB;
            break;
        case 'E':  // 偶校验
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD;
            break;
        case 'O':  // 奇校验
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD;
            break;
        default:
            setError("Invalid parity: " + std::string(1, config_.parity));
            return false;
    }

    // 启用接收器，忽略调制解调器状态行
    tty.c_cflag |= (CLOCAL | CREAD);

    // 禁用硬件流控制
    tty.c_cflag &= ~CRTSCTS;

    // 禁用软件流控制
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 原始模式（禁用所有输入和输出处理）
    tty.c_lflag = 0;  // 禁用规范模式、回显等
    tty.c_oflag = 0;  // 禁用输出处理

    // 禁止所有输入字符转换和特殊字符处理
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // 非阻塞读取配置
    tty.c_cc[VMIN] = 0;   // 最少读取 0 个字符
    tty.c_cc[VTIME] = 0;  // 不等待

    // 应用配置
    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        setError("Failed to set terminal attributes: " + std::string(std::strerror(errno)));
        return false;
    }

    return true;
}

/**
 * @brief  设置错误信息
 * @param  error 错误信息
 * @retval null
 */
void SerialPort::setError(const std::string& error) {
    // mutex_ 已在调用函数中锁定，或者此函数在已锁定的上下文中调用
    last_error_ = error;
}

/**
 * @brief  波特率转常量
 * @param  baudrate 波特率值
 * @retval 波特率常量
 */
int SerialPort::baudrateToConst(int baudrate) const {
    switch (baudrate) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:      return 0;
    }
}

} // namespace teleop_adapter
