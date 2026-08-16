#include "vehicle_display.h"

#include "vehicle_diagnostics.h"

#include <stdarg.h>
#include <stdio.h>

constexpr auto DISPLAY_PAGE_COUNT = 4U;
constexpr auto DISPLAY_ROWS = 10U;
constexpr auto DISPLAY_PAGE_REFRESHES = 20U;

static void write_line(VehicleDisplayLineWriter writer, void *context,
                       uint8_t row, const char *format, ...)
{
    char line[48];
    va_list arguments;
    va_start(arguments, format);
    static_cast<void>(vsnprintf(line, sizeof(line), format, arguments));
    va_end(arguments);
    writer(row, line, context);
}

void vehicle_display_init(VehicleDisplay *display)
{
    if(display != nullptr)
    {
        display->page = 0U;
        display->refreshes_on_page = 0U;
    }
}

void vehicle_display_next_page(VehicleDisplay *display)
{
    if(display != nullptr)
    {
        display->page = static_cast<uint8_t>((display->page + 1U) % DISPLAY_PAGE_COUNT);
        display->refreshes_on_page = 0U;
    }
}

void vehicle_display_render(VehicleDisplay *display,
                            const VehicleTelemetry *t,
                            bool calibration_valid,
                            uint32_t log_dropped_lines,
                            VehicleDisplayLineWriter writer, void *context)
{
    if((display == nullptr) || (t == nullptr) || (writer == nullptr))
    {
        return;
    }
    write_line(writer, context, 0U, "%s P%u CAL:%c", vehicle_state_name(t->state),
               static_cast<unsigned int>(display->page), calibration_valid ? 'Y' : 'N');
    switch(display->page)
    {
        case 0U:
            write_line(writer, context, 1U, "ENC raw L:%ld R:%ld",
                       static_cast<long>(t->left_wheel.delta_count), static_cast<long>(t->right_wheel.delta_count));
            write_line(writer, context, 2U, "ENC ext L:%lld", static_cast<long long>(t->left_wheel.count));
            write_line(writer, context, 3U, "ENC ext R:%lld", static_cast<long long>(t->right_wheel.count));
            write_line(writer, context, 4U, "SPD L:%+.3f R:%+.3f",
                       static_cast<double>(t->left_wheel.speed_mps), static_cast<double>(t->right_wheel.speed_mps));
            write_line(writer, context, 5U, "MT raw:%u stat:%02X", static_cast<unsigned int>(t->steering.raw_angle),
                       static_cast<unsigned int>(t->steering.magnetic_status));
            write_line(writer, context, 6U, "MT ext:%lld", static_cast<long long>(t->steering.continuous_count));
            write_line(writer, context, 7U, "MT rel:%lld", static_cast<long long>(t->steering.relative_count));
            write_line(writer, context, 8U, "STEER:%+.4f rad", static_cast<double>(t->steering.angle_rad));
            write_line(writer, context, 9U, "FAULT:%08lX", static_cast<unsigned long>(t->fault_flags));
            break;
        case 1U:
            write_line(writer, context, 1U, "ACC %d %d %d", t->imu.raw_acc[0],
                       t->imu.raw_acc[1], t->imu.raw_acc[2]);
            write_line(writer, context, 2U, "GYR %d %d %d", t->imu.raw_gyro[0],
                       t->imu.raw_gyro[1], t->imu.raw_gyro[2]);
            write_line(writer, context, 3U, "MAG %d %d %d", t->imu.raw_mag[0],
                       t->imu.raw_mag[1], t->imu.raw_mag[2]);
            write_line(writer, context, 4U, "TEMP raw:%d %.1fC", t->imu.raw_temperature,
                       static_cast<double>(t->imu.temperature_c));
            write_line(writer, context, 5U, "GYRO bias:%+.6f", static_cast<double>(t->pose.gyro_z_bias_radps));
            write_line(writer, context, 6U, "OMEGA I:%+.3f W:%+.3f",
                       static_cast<double>(t->pose.omega_imu_radps), static_cast<double>(t->pose.omega_wheel_radps));
            write_line(writer, context, 7U, "OMEGA S:%+.3f", static_cast<double>(t->pose.omega_steering_radps));
            write_line(writer, context, 8U, "IMU:%c MAG:%c", t->imu.accel_gyro_valid ? 'Y' : 'N',
                       t->imu.magnetometer_valid ? 'Y' : 'N');
            write_line(writer, context, 9U, "FAULT:%08lX", static_cast<unsigned long>(t->fault_flags));
            break;
        case 2U:
            write_line(writer, context, 1U, "X:%+.3f Y:%+.3f", static_cast<double>(t->pose.x_m),
                       static_cast<double>(t->pose.y_m));
            write_line(writer, context, 2U, "YAW:%+.4f", static_cast<double>(t->pose.yaw_rad));
            write_line(writer, context, 3U, "V:%+.3f STATIC:%c", static_cast<double>(t->pose.vehicle_speed_mps),
                       t->pose.stationary ? 'Y' : 'N');
            write_line(writer, context, 4U, "PATH n:%lu s:%.2f",
                       static_cast<unsigned long>(t->path_point_count), static_cast<double>(t->path_length_m));
            write_line(writer, context, 5U, "IDX n:%lu t:%lu",
                       static_cast<unsigned long>(t->tracker.nearest_index),
                       static_cast<unsigned long>(t->tracker.target_index));
            write_line(writer, context, 6U, "CTE:%+.3f HE:%+.3f",
                       static_cast<double>(t->tracker.cross_track_error_m),
                       static_cast<double>(t->tracker.heading_error_rad));
            write_line(writer, context, 7U, "LOOK:%.3f SLIP:%c",
                       static_cast<double>(t->tracker.lookahead_m), t->pose.wheel_slip ? 'Y' : 'N');
            write_line(writer, context, 8U, "TARGET V:%+.3f", static_cast<double>(t->target_speed_mps));
            write_line(writer, context, 9U, "TARGET ST:%+.4f", static_cast<double>(t->target_steering_rad));
            break;
        default:
            write_line(writer, context, 1U, "PWM L:%+.3f R:%+.3f",
                       static_cast<double>(t->actuators.left_motor_duty),
                       static_cast<double>(t->actuators.right_motor_duty));
            write_line(writer, context, 2U, "PWM ST:%+.3f", static_cast<double>(t->actuators.steering_motor_duty));
            write_line(writer, context, 3U, "FAULT:%08lX", static_cast<unsigned long>(t->fault_flags));
            write_line(writer, context, 4U, "LATCH:%08lX", static_cast<unsigned long>(t->latched_fault_flags));
            write_line(writer, context, 5U, "LOG DROP:%lu", static_cast<unsigned long>(log_dropped_lines));
            write_line(writer, context, 6U, "TS:%llu us", static_cast<unsigned long long>(t->timestamp_us));
            write_line(writer, context, 7U, "AUTO:%s", calibration_valid ? "ARMABLE" : "LOCKED");
            write_line(writer, context, 8U, "NO GPS / MAG RAW ONLY");
            write_line(writer, context, 9U, "CMD: HELP on UART0");
            break;
    }
    display->refreshes_on_page++;
    if(display->refreshes_on_page >= DISPLAY_PAGE_REFRESHES)
    {
        vehicle_display_next_page(display);
    }
}
