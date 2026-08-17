#include "zf_common_headfile.h"
#include "vision_ips_fast.h"

#if defined(__TASKING__)
/* Full-frame gray-to-RGB565 conversion is safe to optimize independently. */
#pragma optimize acefgiklmnoprsuvwy
#pragma tradeoff 0
#endif

constexpr auto VISION_IPS_PIXEL_COUNT = MT9V03X_W * MT9V03X_H;

#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#endif
IFX_ALIGN(4) static uint16 s_gray_rgb565_lut[256];
IFX_ALIGN(4) static uint16 s_rgb565_frame[VISION_IPS_PIXEL_COUNT];
#if defined(__TASKING__)
#pragma section all restore
#endif

static void vision_ips_write_command(uint8 command)
{
    gpio_low(IPS200_DC_PIN_SPI);
    spi_write_8bit(IPS200_SPI, command);
    gpio_high(IPS200_DC_PIN_SPI);
}

static void vision_ips_set_region(uint16 x1, uint16 y1, uint16 x2, uint16 y2)
{
    vision_ips_write_command(0x2AU);
    spi_write_16bit(IPS200_SPI, x1);
    spi_write_16bit(IPS200_SPI, x2);

    vision_ips_write_command(0x2BU);
    spi_write_16bit(IPS200_SPI, y1);
    spi_write_16bit(IPS200_SPI, y2);

    vision_ips_write_command(0x2CU);
}

void vision_ips_fast_init()
{
    uint16 gray;

    for(gray = 0U; gray < 256U; ++gray)
    {
        s_gray_rgb565_lut[gray] = (uint16)(((gray & 0xF8U) << 8U)
                                          | ((gray & 0xFCU) << 3U)
                                          |  (gray >> 3U));
    }
}

static void vision_ips_draw_bbox(int16_t x, int16_t y, int16_t width, int16_t height, uint16 color)
{
    int16_t x0 = x;
    int16_t y0 = y;
    int16_t x1 = static_cast<int16_t>(x + width - 1);
    int16_t y1 = static_cast<int16_t>(y + height - 1);
    int16_t pixel;

    if ((width <= 1) || (height <= 1))
    {
        return;
    }
    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 >= static_cast<int16_t>(MT9V03X_W)) { x1 = static_cast<int16_t>(MT9V03X_W) - 1; }
    if (y1 >= static_cast<int16_t>(MT9V03X_H)) { y1 = static_cast<int16_t>(MT9V03X_H) - 1; }
    if ((x0 >= x1) || (y0 >= y1))
    {
        return;
    }

    for (pixel = x0; pixel <= x1; ++pixel)
    {
        s_rgb565_frame[(uint32)y0 * MT9V03X_W + (uint16)pixel] = color;
        s_rgb565_frame[(uint32)y1 * MT9V03X_W + (uint16)pixel] = color;
    }
    for (pixel = y0; pixel <= y1; ++pixel)
    {
        s_rgb565_frame[(uint32)pixel * MT9V03X_W + (uint16)x0] = color;
        s_rgb565_frame[(uint32)pixel * MT9V03X_W + (uint16)x1] = color;
    }
}

static void vision_ips_draw_line(vision_tag_point_t start,
                                 vision_tag_point_t end,
                                 uint16 color)
{
    int16_t x = start.x;
    int16_t y = start.y;
    int16_t dx = static_cast<int16_t>((end.x >= start.x) ? (end.x - start.x) : (start.x - end.x));
    int16_t sx = (start.x < end.x) ? 1 : -1;
    int16_t dy = static_cast<int16_t>(-((end.y >= start.y) ? (end.y - start.y) : (start.y - end.y)));
    int16_t sy = (start.y < end.y) ? 1 : -1;
    int16_t error = static_cast<int16_t>(dx + dy);

    while (1)
    {
        if ((x >= 0) && (x < static_cast<int16_t>(MT9V03X_W))
            && (y >= 0) && (y < static_cast<int16_t>(MT9V03X_H)))
        {
            s_rgb565_frame[(uint32)y * MT9V03X_W + (uint16)x] = color;
        }
        if ((x == end.x) && (y == end.y))
        {
            break;
        }
        {
            int16_t doubled = static_cast<int16_t>(2 * error);
            if (doubled >= dy) { error = static_cast<int16_t>(error + dy); x = static_cast<int16_t>(x + sx); }
            if (doubled <= dx) { error = static_cast<int16_t>(error + dx); y = static_cast<int16_t>(y + sy); }
        }
    }
}

void vision_ips_fast_show_gray(const uint8_t *image,
                               const vision_tag_result_t *result)
{
    uint32 pixel;
    uint16 row;

    if(nullptr == image)
    {
        return;
    }

    /* Linear conversion with no scaling divisions. */
    for(pixel = 0U; pixel < VISION_IPS_PIXEL_COUNT; ++pixel)
    {
        s_rgb565_frame[pixel] = s_gray_rgb565_lut[image[pixel]];
    }

    if ((result != nullptr) && (result->valid != 0U))
    {
        uint16 color = (result->predicted != 0U) ? RGB565_YELLOW : RGB565_GREEN;
        if ((result->detected != 0U) && (result->has_corners != 0U))
        {
            uint8 corner;
            for (corner = 0U; corner < 4U; ++corner)
            {
                vision_ips_draw_line(result->corners[corner],
                                     result->corners[(corner + 1U) & 3U], color);
            }
        }
        else
        {
            vision_ips_draw_bbox(result->bbox.x, result->bbox.y,
                                 result->bbox.width, result->bbox.height, color);
        }
    }

    gpio_low(IPS200_CS_PIN_SPI);
    vision_ips_set_region(0U, 0U, MT9V03X_W - 1U, MT9V03X_H - 1U);
    /*
     * Keep each QSPI transaction to one scan line.  A single 22560-pixel
     * stream is faster on paper, but the long uninterrupted transfer can
     * produce lower-frame black blocks on the single-row-header IPS200.
     * This mirrors the proven SeekFree driver while keeping LUT conversion.
     */
    for(row = 0U; row < MT9V03X_H; ++row)
    {
        spi_write_16bit_array(IPS200_SPI,
                              &s_rgb565_frame[(uint32)row * MT9V03X_W],
                              MT9V03X_W);
    }
    gpio_high(IPS200_CS_PIN_SPI);
}


