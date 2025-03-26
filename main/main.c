#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "math.h"
#include "string.h"

#include "lvgl.h"
#include "esp_lcd_gc9a01.h" // 屏幕驱动
#include "ui/ui.h"

#include "bmp280.h"
#include "sht4x.h"
#include "sgp4x.h"

#define I2C_SCL_GPIO 18
#define I2C_SDA_GPIO 19
#define I2C_NUM 0
#define I2C_CLK_SPEED_HZ 400000

i2c_master_dev_handle_t bmp280_handle;
i2c_master_dev_handle_t sht4x_handle;
i2c_master_dev_handle_t sgp4x_handle;


// Using SPI2
#define LCD_HOST SPI2_HOST

// 屏幕尺寸
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 240

#define EXAMPLE_PIN_NUM_LCD_CS 5
#define EXAMPLE_PIN_NUM_LCD_DC 4
#define EXAMPLE_PIN_NUM_LCD_RST 6
#define EXAMPLE_PIN_NUM_DATA0 2
#define EXAMPLE_PIN_NUM_SCLK 1

// 日志标签
static const char *TAG = "main";

static SemaphoreHandle_t lvgl_mux = NULL;

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE 4096
#define EXAMPLE_LVGL_TASK_PRIORITY 2

char buf_Temp[64];
char buf_Humi[64];
char buf_Press[64];


i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io, uint8_t i2c_port)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C master bus created");
    return bus_handle;
}

// 回调函数:颜色传输完成 (通知LVGL刷新)
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

// 回调函数:刷新屏幕
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    /* Copy a buffer's content to a specific area of the display */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

// 回调函数:增加LVGL的tick
static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

// 测试UI
void create_centered_circle(void)
{
    // 创建一个基础对象作为圆
    lv_obj_t *circle = lv_obj_create(lv_scr_act());

    // 设置对象尺寸（宽高相同形成圆）
    lv_obj_set_size(circle, 100, 100); // 直径100像素

    // 将对象居中于屏幕
    lv_obj_center(circle);

    // 设置样式属性
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);                     // 圆形半径
    lv_obj_set_style_bg_color(circle, lv_palette_main(LV_PALETTE_ORANGE), 0); // 填充颜色
    lv_obj_set_style_border_width(circle, 0, 0);                              // 边框宽度设为0
}

// LVGL任务
static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    ESP_LOGI(TAG, "Display LVGL UI");

    // create_centered_circle();
    ui_init();
    // lv_demo_widgets();
    //  lv_demo_music();

    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // ESP_LOGI(TAG, "LVGL task");
        /* Lock the mutex due to the LVGL APIs are not thread-safe */
        if (example_lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            /* Release the mutex */
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

/**
 * @brief 周期性读取 BMP280 测量数据的任务
 */
void bmp280_read_task(void *pvParameters)
{
    float temperature, pressure;

    while (1)
    {
        esp_err_t err = bmp280_read_measurement(bmp280_handle, &temperature, &pressure);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %.2f hPa", temperature, pressure);
            sprintf(buf_Press, "%.2f hPa", pressure);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read BMP280 measurement");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sht4x_read_task(void *pvParameters)
{
    float temperature, humidity;

    while (1)
    {

        esp_err_t err = sht4x_start_measurement(sht4x_handle, SHT4X_CMD_READ_MEASUREMENT_HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        err = sht4x_read_measurement(sht4x_handle, &temperature, &humidity);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Temperature: %.2f C, Humidity: %.2f %%", temperature, humidity);
            sprintf(buf_Temp, "%.2f C", temperature);
            sprintf(buf_Humi, "%.2f %%", humidity);
            
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read temperature and humidity");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sgp4x_read_task(void *pvParameters)
{
    uint16_t sraw_voc, sraw_nox;

    while (1)
    {
        esp_err_t err = sgp4x_start_measurement(sgp4x_handle, SGP4X_CMD_MEAS_RAW_SIGNALS);
        vTaskDelay(pdMS_TO_TICKS(50));
        err = sgp4x_read_measurement(sgp4x_handle, &sraw_voc, &sraw_nox);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "VOC: %d, NOX: %d", sraw_voc, sraw_nox);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read VOC and NOX");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    // 初始化SPI总线
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_SCLK, EXAMPLE_PIN_NUM_DATA0, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 创建屏幕句柄
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_DC, notify_lvgl_flush_ready, &disp_drv);

    // 将LCD连接到SPI总线
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // 创建屏幕驱动句柄
    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    // 屏幕配置
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST, // Set to -1 if not use
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16, // Implemented by LCD command `3Ah` (16/18)
                              //.vendor_config = &vendor_config,
    };
    // 创建屏幕实例
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    // 屏幕复位
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    // 初始化屏幕
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // 打开屏幕
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // 初始化LVGL
    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();
    // 申请内存 两个buf
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1); // 检查内存是否申请成功
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // 初始化LVGL显示缓冲区
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 50);

    // 初始化LVGL显示驱动
    ESP_LOGI(TAG, "Initialize LVGL display driver");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;              // 设置屏幕水平分辨率
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;              // 设置屏幕垂直分辨率
    disp_drv.flush_cb = lvgl_flush_cb;                 // 设置刷新回调函数
    disp_drv.draw_buf = &disp_buf;                     // 设置显示缓冲区
    disp_drv.user_data = panel_handle;                 // 设置用户数据
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv); // 注册显示驱动

    // 创建定时器
    ESP_LOGI(TAG, "Install LVGL tick timer");
    /* Tick interface for LVGL (using esp_timer to generate 2ms periodic event) */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    // 初始化 I2C 总线
    i2c_master_bus_handle_t bus_handle = i2c_bus_init(I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_NUM);


    // 在 I2C 总线上创建设备实例
    bmp280_handle = bmp280_device_create(bus_handle, BMP280_I2C_ADDR, I2C_CLK_SPEED_HZ);
    sht4x_handle = sht4x_device_create(bus_handle, SHT4X_I2C_ADDR_0, I2C_CLK_SPEED_HZ);
    sgp4x_handle = sgp4x_device_create(bus_handle, SGP4X_I2C_ADDR, I2C_CLK_SPEED_HZ);

    // 探测 BMP280 传感器（200ms 超时）
     esp_err_t err = i2c_master_probe(bus_handle, BMP280_I2C_ADDR, 200);
     if (err == ESP_OK)
     {
         ESP_LOGI(TAG, "BMP280 sensor found");
         // 初始化传感器（包含芯片 ID 校验、校准数据读取及配置传感器工作模式）
         err = bmp280_init_device(bmp280_handle);
         if (err == ESP_OK)
         {
             ESP_LOGI(TAG, "BMP280 sensor initialization success");
             xTaskCreate(bmp280_read_task, "bmp280_read_task", 4096, NULL, 5, NULL);
         }
         else
         {
             ESP_LOGE(TAG, "BMP280 sensor initialization failed");
             bmp280_device_delete(bmp280_handle);
         }
     }
     else
     {
         ESP_LOGE(TAG, "BMP280 sensor not found");
         bmp280_device_delete(bmp280_handle);
     }

    // 探测 sht4x 传感器（200ms 超时）
    err = i2c_master_probe(bus_handle, SHT4X_I2C_ADDR_0, 200);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "SHT4X sensor found");
        xTaskCreate(sht4x_read_task, "sht4x_read_task", 4096, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(TAG, "SHT4X sensor not found");
        sht4x_device_delete(sht4x_handle);
    }

    // 探测 sgp4x 传感器（200ms 超时）
    err = i2c_master_probe(bus_handle, SGP4X_I2C_ADDR, 200);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "SGP4X sensor found");
        xTaskCreate(sgp4x_read_task, "sgp4x_read_task", 4096, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(TAG, "SGP4X sensor not found");
        sgp4x_device_delete(sgp4x_handle);
    }

    // 创建LVGL任务
    ESP_LOGI(TAG, "Create LVGL task");
    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    // 创建BMP280任务
    // xTaskCreate(bmp280_task, "BMP280", 4096, NULL, 2, NULL);

    // 创建sht4x任务
    // xTaskCreate(sht4x_task, "SHT4x", 4096, NULL, 2, NULL);

    static int counter = 0;

    while (1)
    {
        // sprintf(buffer, "Counter: %d", counter++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        lv_label_set_text(ui_Label2, buf_Temp);
        lv_label_set_text(ui_Label4, buf_Humi);
        lv_label_set_text(ui_Label6, buf_Press);

    }
}