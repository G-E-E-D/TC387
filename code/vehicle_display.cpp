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
            write_line(writer, context, 5U, "SPI raw:%u err:%lu", static_cast<unsigned int>(t->steering.raw_angle),
                       static_cast<unsigned long>(t->steering.communication_error_count));
            write_line(writer, context, 6U, "SPI rel:%ld jump:%lu", static_cast<long>(t->steering.relative_count),
                       static_cast<unsigned long>(t->steering.jump_error_count));
            write_line(writer, context, 7U, "SPI cont:%lld", static_cast<long long>(t->steering.continuous_count));
            write_line(writer, context, 8U, "STEER:%+.4f rad", static_cast<double>(t->steering.angle_rad));
            write_line(writer, context, 9U, "FAULT:%08lX", static_cast<unsigned long>(t->fault_flags));
            /* Keep the remaining historical page rows out of the automatic
             * display; the SPI sample is now the steering source. */
            break;
        case 1U:
            write_line(writer, context, 1U, "ACC %d %d %d", t->imu.raw_acc[0],
                       t->imu.raw_acc[1], t->imu.raw_acc[2]);
            write_line(writer, context, 2U, "GYR %d %d %d", t->imu.raw_gyro[0],
                       t->imu.raw_gyro[1], t->imu.raw_gyro[2]);
            write_line(writer, context, 3U, "IMU:%c MAG:%c", t->imu.accel_gyro_valid ? 'Y' : 'N',
                       t->imu.magnetometer_valid ? 'Y' : 'N');
            write_line(writer, context, 4U, "X:%+.3f Y:%+.3f", static_cast<double>(t->pose.x_m),
                       static_cast<double>(t->pose.y_m));
            write_line(writer, context, 5U, "YAW:%+.4f", static_cast<double>(t->pose.yaw_rad));
            write_line(writer, context, 6U, "OMEGA I:%+.3f W:%+.3f", static_cast<double>(t->pose.omega_imu_radps),
                       static_cast<double>(t->pose.omega_wheel_radps));
            write_line(writer, context, 7U, "OMEGA S:%+.3f", static_cast<double>(t->pose.omega_steering_radps));
            write_line(writer, context, 8U, "PATH n:%lu s:%.2f", static_cast<unsigned long>(t->path_point_count),
                       static_cast<double>(t->path_length_m));
            write_line(writer, context, 9U, "FAULT:%08lX", static_cast<unsigned long>(t->fault_flags));
            break;
        case 2U:
            write_line(writer, context, 1U, "TAG seq:%lu c:%u", static_cast<unsigned long>(t->guide_target.camera_sequence),
                       static_cast<unsigned int>(t->guide_target.confidence));
            write_line(writer, context, 2U, "TAG xy:%d,%d sz:%u", static_cast<int>(t->guide_target.center_x_px),
                       static_cast<int>(t->guide_target.center_y_px), static_cast<unsigned int>(t->guide_target.size_px));
            write_line(writer, context, 3U, "GUIDE x:%+.2f y:%+.2f", static_cast<double>(t->guide_target.target_x_forward_m),
                       static_cast<double>(t->guide_target.target_y_left_m));
            write_line(writer, context, 4U, "AIM x:%+.2f y:%+.2f", static_cast<double>(t->forward_tracker.aim_x_m),
                       static_cast<double>(t->forward_tracker.aim_y_m));
            write_line(writer, context, 5U, "FWD V:%+.3f ST:%+.3f", static_cast<double>(t->forward_tracker.target_speed_mps),
                       static_cast<double>(t->forward_tracker.target_steering_rad));
            write_line(writer, context, 6U, "CURV:%+.3f AGE:%llu P:%lu",
                       static_cast<double>(t->forward_tracker.curvature_per_m),
                       static_cast<unsigned long long>(t->guide_target.age_us),
                       static_cast<unsigned long>(t->guide_target.process_us_last));
            write_line(writer, context, 7U, "REV IDX:%lu/%lu", static_cast<unsigned long>(t->tracker.nearest_index),
                       static_cast<unsigned long>(t->tracker.target_index));
            write_line(writer, context, 8U, "REV V:%+.3f ST:%+.3f", static_cast<double>(t->tracker.target_speed_mps),
                       static_cast<double>(t->tracker.target_steering_rad));
            write_line(writer, context, 9U, "FAULT:%08lX", static_cast<unsigned long>(t->fault_flags));
            break;
        case 3U:
            write_line(writer, context, 1U, "PWM L:%+.3f R:%+.3f",
                       static_cast<double>(t->actuators.left_motor_duty),
                       static_cast<double>(t->actuators.right_motor_duty));
            write_line(writer, context, 2U, "PWM ST:%+.3f", static_cast<double>(t->actuators.steering_motor_duty));
            write_line(writer, context, 3U, "FAULT:%08lX", static_cast<unsigned long>(t->fault_flags));
            write_line(writer, context, 4U, "LATCH:%08lX", static_cast<unsigned long>(t->latched_fault_flags));
            write_line(writer, context, 5U, "LOG DROP:%lu", static_cast<unsigned long>(log_dropped_lines));
            write_line(writer, context, 6U, "TS:%llu us", static_cast<unsigned long long>(t->timestamp_us));
            write_line(writer, context, 7U, "AUTO:%s", calibration_valid ? "ARMABLE" : "LOCKED");
            write_line(writer, context, 8U, "SPI1 steering / RAM path");
            write_line(writer, context, 9U, "CMD: HELP on UART0");
            break;
        default:
            break;
    }
    display->refreshes_on_page++;
    if(display->refreshes_on_page >= DISPLAY_PAGE_REFRESHES)
    {
        vehicle_display_next_page(display);
    }
}
