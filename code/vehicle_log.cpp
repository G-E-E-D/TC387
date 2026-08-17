#include "vehicle_log.h"

#include "vehicle_config.h"

#include <stdio.h>
#include <string.h>

static const char g_csv_header[] =
    "timestamp_us,state,left_encoder_count,right_encoder_count,"
    "left_speed_mps,right_speed_mps,steering_raw,steering_relative_count,"
    "steering_continuous_count,steering_angle_rad,steering_comm_errors,"
    "steering_jump_errors,camera_sequence,tag_confidence,tag_center_x,"
    "tag_center_y,tag_size,vision_process_us,vision_process_max_us,"
    "guide_x,guide_y,guide_distance,follow_distance,"
    "forward_aim_x,forward_aim_y,forward_curvature,forward_speed,"
    "forward_steering,imu_gz,omega_wheel,omega_steer,pose_x,pose_y,"
    "pose_yaw,fused_speed,target_speed,target_steering,left_pwm,right_pwm,"
    "steering_pwm,path_index,cross_track_error,heading_error,fault_flags\r\n";

static bool log_push(VehicleLog *log, const uint8_t *data, size_t size)
{
    size_t i;
    if((log == nullptr) || (data == nullptr) || (size > static_cast<size_t>(VEHICLE_LOG_BUFFER_SIZE - log->used)))
    {
        if(log != nullptr)
        {
            log->dropped_lines++;
        }
        return false;
    }
    for(i = 0U; i < size; ++i)
    {
        log->data[log->write_index] = data[i];
        log->write_index = (log->write_index + 1U) % VEHICLE_LOG_BUFFER_SIZE;
    }
    log->used += static_cast<uint32_t>(size);
    return true;
}

void vehicle_log_init(VehicleLog *log)
{
    if(log != nullptr)
    {
        log->read_index = 0U;
        log->write_index = 0U;
        log->used = 0U;
        log->dropped_lines = 0U;
        log->csv_enabled = false;
        log->header_pending = true;
    }
}

void vehicle_log_set_csv_enabled(VehicleLog *log, bool enabled)
{
    if(log != nullptr)
    {
        if(enabled && !log->csv_enabled)
        {
            log->header_pending = true;
        }
        log->csv_enabled = enabled;
    }
}

bool vehicle_log_is_csv_enabled(const VehicleLog *log)
{
    return (log != nullptr) && log->csv_enabled;
}

bool vehicle_log_enqueue_text(VehicleLog *log, const char *text)
{
    return (text != nullptr) && log_push(log, (const uint8_t *)text, strlen(text));
}

bool vehicle_log_enqueue_csv(VehicleLog *log, const VehicleTelemetry *t)
{
    char line[VEHICLE_CSV_LINE_SIZE];
    int count;
    if((log == nullptr) || (t == nullptr) || !log->csv_enabled)
    {
        return false;
    }
    if(log->header_pending)
    {
        if(!log_push(log, (const uint8_t *)g_csv_header, sizeof(g_csv_header) - 1U))
        {
            return false;
        }
        log->header_pending = false;
    }
    count = snprintf(line, sizeof(line),
        "%llu,%u,%lld,%lld,%.5f,%.5f,%u,%ld,%lld,%.6f,%lu,%lu,"
        "%lu,%u,%d,%d,%u,%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,"
        "%.5f,%.5f,%.6f,%.5f,%.5f,%.6f,%.5f,%.5f,%.5f,"
        "%.5f,%.5f,%.5f,%.5f,%.5f,%lu,%.5f,%.6f,%lu\r\n",
        static_cast<unsigned long long>(t->timestamp_us),
        static_cast<unsigned int>(t->state),
        static_cast<long long>(t->left_wheel.count),
        static_cast<long long>(t->right_wheel.count),
        static_cast<double>(t->left_wheel.speed_mps),
        static_cast<double>(t->right_wheel.speed_mps),
        static_cast<unsigned int>(t->steering.raw_angle),
        static_cast<long>(t->steering.relative_count),
        static_cast<long long>(t->steering.continuous_count),
        static_cast<double>(t->steering.angle_rad),
        static_cast<unsigned long>(t->steering.communication_error_count),
        static_cast<unsigned long>(t->steering.jump_error_count),
        static_cast<unsigned long>(t->guide_target.camera_sequence),
        static_cast<unsigned int>(t->guide_target.confidence),
        static_cast<int>(t->guide_target.center_x_px),
        static_cast<int>(t->guide_target.center_y_px),
        static_cast<unsigned int>(t->guide_target.size_px),
        static_cast<unsigned long>(t->guide_target.process_us_last),
        static_cast<unsigned long>(t->guide_target.process_us_max),
        static_cast<double>(t->guide_target.target_x_forward_m),
        static_cast<double>(t->guide_target.target_y_left_m),
        static_cast<double>(t->guide_target.distance_m),
        static_cast<double>(t->desired_follow_distance_m),
        static_cast<double>(t->forward_tracker.aim_x_m),
        static_cast<double>(t->forward_tracker.aim_y_m),
        static_cast<double>(t->forward_tracker.curvature_per_m),
        static_cast<double>(t->forward_tracker.target_speed_mps),
        static_cast<double>(t->forward_tracker.target_steering_rad),
        static_cast<double>(t->imu.angular_rate_radps[2]),
        static_cast<double>(t->pose.omega_wheel_radps),
        static_cast<double>(t->pose.omega_steering_radps),
        static_cast<double>(t->pose.x_m),
        static_cast<double>(t->pose.y_m),
        static_cast<double>(t->pose.yaw_rad),
        static_cast<double>(t->pose.vehicle_speed_mps),
        static_cast<double>(t->target_speed_mps),
        static_cast<double>(t->target_steering_rad),
        static_cast<double>(t->actuators.left_motor_duty),
        static_cast<double>(t->actuators.right_motor_duty),
        static_cast<double>(t->actuators.steering_motor_duty),
        static_cast<unsigned long>(t->tracker.nearest_index),
        static_cast<double>(t->tracker.cross_track_error_m),
        static_cast<double>(t->tracker.heading_error_rad),
        static_cast<unsigned long>(t->fault_flags));
    if((count <= 0) || (static_cast<size_t>(count) >= sizeof(line)))
    {
        log->dropped_lines++;
        return false;
    }
    return log_push(log, (const uint8_t *)line, static_cast<size_t>(count));
}

size_t vehicle_log_flush(VehicleLog *log, size_t byte_budget,
                         VehicleLogTryWriteByte writer, void *context)
{
    size_t sent = 0U;
    if((log == nullptr) || (writer == nullptr))
    {
        return 0U;
    }
    while((sent < byte_budget) && (log->used > 0U))
    {
        uint8_t byte = log->data[log->read_index];
        if(!writer(byte, context))
        {
            break;
        }
        log->read_index = (log->read_index + 1U) % VEHICLE_LOG_BUFFER_SIZE;
        log->used--;
        sent++;
    }
    return sent;
}

uint32_t vehicle_log_dropped_lines(const VehicleLog *log)
{
    return (log != nullptr) ? log->dropped_lines : 0U;
}
