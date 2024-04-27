/**
*版权所有 (c) 2014 -2021，Nordic Semiconductor ASA
*
*版权所有。
*
*以源代码和二进制形式重新分发和使用，无论是否经过修改，
*是允许的，前提是满足以下条件：
*
*1. 源代码的再分发必须保留上述版权声明，本
*条件清单和以下免责声明。
*
*2. 以二进制形式重新分配，嵌入 Nordic 的除外
*产品中的半导体 ASA 集成电路或软件更新
*此类产品，必须复制上述版权声明，此列表
*文档和/或其他内容中的条件和以下免责声明
*分发时提供的材料。
*
*3. 既不是 Nordic Semiconductor ASA 的名称，也不是其子公司的名称
*贡献者可用于认可或推广由此衍生的产品
*未经特定事先书面许可的软件。
*
*4. 本软件无论是否经过修改，都只能与
*Nordic Semiconductor ASA 集成电路。
*
*5. 本许可下以二进制形式提供的任何软件均不得逆向
*设计、反编译、修改和/或反汇编。
*
*本软件由 Nordic SEMICONDUCTOR ASA “按原样”和任何快递提供
*或默示保证，包括但不限于默示保证
*适销性、非侵权性和特定用途的适用性
*免责声明。在任何情况下，NORDIC SEMICONDUCTOR ASA 或贡献者均不得
*对任何直接、间接、附带、特殊、示范或
*间接损害（包括但不限于购买替代品
*商品或服务；使用、数据或利润的损失；或业务中断）
*无论出于何种原因以及根据任何责任理论，无论是否有合同，严格
*任何原因引起的责任或侵权行为（包括疏忽或其他原因）
*本软件的使用，即使已被告知可能造成此类损害。
*
*/
/**@文件
*
*@defgroup ble_sdk_app_hts_main main.c
*@{
*@ingroup ble_sdk_app_hts
*@brief 健康温度计服务示例应用程序主文件。
*
*此文件包含使用健康温度计服务的示例应用程序的源代码
*它还包括电池和设备信息服务的示例代码。
*此应用程序使用@ref srvlib_conn_params 模块。
*/

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hts.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

const uint8_t * DEVICE_NAME;
/*TWI 实例 ID。*/
#define TWI_INSTANCE_ID     0

/*温度传感器的通用地址定义。*/
#define LM75B_ADDR          (0x90U >> 1)

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

/*QMI的通用地址定义。*/
#define QMI_ADDR          (0x28U)//(0x6BU >> 1)

#define QMI_TEMP_L          0x33U //温度_L
#define QMI_TEMP_H          0x34U //温度_H

#define QMI_CTRL_7             0x08U //
#define QMI_CTRL_2          0x03U //

#define QMI_WHO_AM_I          0x00U //
/*
QMI8658C 配备内部 16 位嵌⼊式温度传感器，每当启⽤加速
度计或陀螺仪时，该传感器默认⾃动打开
配置为惯性测量单元的设备，即加速度计和陀螺仪组合传感
器
CTRL7 gSN=0, aEN =1, gEN
=1, mEN =0
CTRL2 aODR != 11xx
i2c地址0x6B，向0x08写入0xC3，向0x03写入0xFC，之后读取0x33
*/
/*QMI 的模式。*/
#define QMI_MODE_1 0xC3U
#define QMI_MODE_2 0xFCU
/*LM75B 的模式。*/
#define NORMAL_MODE 0U

/*指示 TWI 上的操作是否已结束。*/
static volatile bool m_xfer_done = false;

/*TWI 实例。*/
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/*从温度传感器读取样本的缓冲区。*/
static uint8_t m_sample;

/**
*@brief 用于在 QMI 加速度计上设置活动模式的函数。
*/
void QMI_set_mode(void)
{
    ret_code_t err_code;

    uint8_t reg[2] = {QMI_CTRL_7, QMI_MODE_1};
    err_code = nrf_drv_twi_tx(&m_twi, QMI_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    //reg[0] = QMI_CTRL_2;
    //reg[1] = 0xFCU;
    uint8_t reg1[2] = {QMI_CTRL_2, QMI_MODE_2};
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, QMI_ADDR, reg1,sizeof(reg1), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /*写入指针字节。*/
    
    //reg[0] = LM75B_REG_TEMP;
    //m_xfer_done = false;
    //err_code = nrf_drv_twi_tx(&m_twi, QMI_ADDR, reg, 1, false);
   // APP_ERROR_CHECK(err_code);
    //while (m_xfer_done == false);
    
    
}

/**
*@brief 用于在 MMA7660 加速度计上设置活动模式的函数。
*/
void LM75B_set_mode(void)
{
    ret_code_t err_code;

/*写入 LM75B_REG_CONF "0" 将温度传感器设置为正常模式。*/
    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

/*写入指针字节。*/
    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}
//nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
/**
*@brief 用于处理来自温度传感器的数据的函数。
*
*@param[in] temp 从传感器读取的温度（以摄氏度为单位）。
*/
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
*@brief TWI 事件处理程序。
*/
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
*@brief UART 初始化。
*/
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
*@brief 从温度传感器读取数据的函数。
*/

static void read_sensor_data()
{
    m_xfer_done = false;

/*从指定地址读取 1 个字节 -跳过专门用于温度小数部分的 3 位。*/
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, QMI_ADDR, &m_sample, sizeof(m_sample));
    //ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

#define DEVICE_NAME                     "Nordic_HTS"/**< 设备名称。将包含在广告数据中。*/
#define MANUFACTURER_NAME               "NordicSemiconductor"/**< 制造商。将传递给设备信息服务。*/
char* MODEL_NUM
//#define MODEL_NUM                       "simahaokes"//“NS-HTS-示例”/**< 型号。将传递给设备信息服务。 */
#define MANUFACTURER_ID                 0x1122334455/**< 制造商 ID，系统 ID 的一部分。将传递给设备信息服务。*/
#define ORG_UNIQUE_ID                   0x667788/**< 组织唯一 ID，系统 ID 的一部分。将传递给设备信息服务。*/

#define APP_BLE_OBSERVER_PRIO           3/**< 应用程序的 BLE 观察者优先级。您不需要修改该值。*/
#define APP_BLE_CONN_CFG_TAG            1/**< 标识 SoftDevice BLE 配置的标签。*/

#define APP_ADV_INTERVAL                40/**< 广告间隔（以0.625毫秒为单位。该值对应25毫秒）。*/

#define APP_ADV_DURATION                18000/**< 广告时长（180秒），以10毫秒为单位。*/

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)/**< 电池电量测量间隔（滴答声）。*/
#define MIN_BATTERY_LEVEL               81/**< 模拟测量函数返回的最低电池电量。*/
#define MAX_BATTERY_LEVEL               100/**< 模拟测量函数返回的最大电池电量。*/
#define BATTERY_LEVEL_INCREMENT         1/**< 每次调用模拟测量函数时电池电量递增/递减的值。*/

#define TEMP_TYPE_AS_CHARACTERISTIC     0/**< 确定温度类型是作为特征 (1) 还是作为测量字段 (0) 给出。*/

#define MIN_CELCIUS_DEGREES             3688/**< 用于模拟测量功能的最低温度（以摄氏度为单位）（乘以 100 以避免浮点运算）。*/
#define MAX_CELCIUS_DEGRESS             3972/**< 用于模拟测量功能的最高温度（以摄氏度为单位）（乘以 100 以避免浮点运算）。*/
#define CELCIUS_DEGREES_INCREMENT       36/**< 每次调用模拟测量函数时温度递增/递减的值（乘以 100 以避免浮点运算）。*/

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)/**< 最小可接受的连接间隔（0.5 秒）*/
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)/**< 最大可接受的连接间隔（1 秒）。*/
#define SLAVE_LATENCY                   0/**< 从机延迟。*/
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)/**< 连接监控超时（4 秒）。*/

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)/**< 从启动事件（连接或开始指示）到第一次调用 sd_ble_gap_conn_param_update 的时间（5 秒）。*/
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)/**< 第一次调用后每次调用 sd_ble_gap_conn_param_update 之间的时间（30 秒）。*/
#define MAX_CONN_PARAMS_UPDATE_COUNT    3/**< 放弃连接参数协商之前的尝试次数。*/

#define SEC_PARAM_BOND                  1/**< 执行绑定。*/
#define SEC_PARAM_MITM                  0/**< 不需要中间人保护。*/
#define SEC_PARAM_LESC                  0/**< LE 安全连接未启用。*/
#define SEC_PARAM_KEYPRESS              0/**< 未启用按键通知。*/
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE/**< 无 I/O 功能。*/
#define SEC_PARAM_OOB                   0/**< 带外数据不可用。*/
#define SEC_PARAM_MIN_KEY_SIZE          7/**< 最小加密密钥大小。*/
#define SEC_PARAM_MAX_KEY_SIZE          16/**< 最大加密密钥大小。*/

#define DEAD_BEEF                       0xDEADBEEF/**< 用作堆栈转储上的错误代码的值，可用于识别堆栈展开上的堆栈位置。*/


APP_TIMER_DEF(m_battery_timer_id);/**< 电池计时器。*/
BLE_BAS_DEF(m_bas);/**< 用于识别电池服务的结构。*/
BLE_HTS_DEF(m_hts);/**< 用于标识健康温度计服务的结构体。*/
NRF_BLE_GATT_DEF(m_gatt);/**< GATT 模块实例。*/
NRF_BLE_QWR_DEF(m_qwr);/**< 排队写入模块的上下文。*/
BLE_ADVERTISING_DEF(m_advertising);/**< 广告模块实例。*/
NRF_BLE_GQ_DEF(m_ble_gatt_queue,/**< BLE GATT 队列实例。*/
               NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t          m_conn_handle = BLE_CONN_HANDLE_INVALID;/**< 当前连接的句柄。*/
static bool              m_hts_meas_ind_conf_pending = false;/**< 用于跟踪指示确认何时挂起的标志。*/
static sensorsim_cfg_t   m_battery_sim_cfg;/**< 电池电量传感器模拟器配置。*/
static sensorsim_state_t m_battery_sim_state;/**< 电池电量传感器模拟器状态。*/
static sensorsim_cfg_t   m_temp_celcius_sim_cfg;/**< 温度模拟器配置。*/
static sensorsim_state_t m_temp_celcius_sim_state;/**< 温度模拟器状态。*/

static ble_uuid_t m_adv_uuids[] =/**< 通用唯一的服务标识符。*/
{
    {BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


static void advertising_start(bool erase_bonds);
static void temperature_measurement_send(void);


/**@brief SoftDevice 中断言的回调函数。
*
*@details 如果 SoftDevice 中发生断言，将调用此函数。
*
*@warning 该处理程序仅作为示例，不适合最终产品。你需要分析
*如果发生断言，您的产品应该如何反应。
*@warning 在来自 SoftDevice 的断言中，系统只能在复位时恢复。
*
*@param[in] line_num 失败的 ASSERT 调用的行号。
*@param[in] file_name 失败的 ASSERT 调用的文件名。
*/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief 用于处理服务错误的函数。
*
*@details 指向此函数的指针将传递给可能需要通知的每个服务
*关于错误的应用程序。
*
*@param[in] nrf_error 包含错误信息的错误代码。
*/
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief 用于处理 Peer Manager 事件的函数。
*
*@param[in] p_evt 对等管理器事件。
*/
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    bool       is_indication_enabled;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            //如果启用指示，则发送单个温度测量值。
//注意：要使其正常工作，请确保在之前调用 ble_hts_on_ble_evt()
//ble_evt_dispatch() 中的 pm_evt_handler()。
            err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
            APP_ERROR_CHECK(err_code);
            if (is_indication_enabled)
            {
                temperature_measurement_send();
            }
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief 用于执行电池测量并更新电池服务中的电池电量特征的功能。
*/
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief 用于处理电池测量计时器超时的函数。
*
*@details 每次电池电量测量计时器到期时都会调用此函数。
*
*@param[in] p_context 用于传递一些任意信息（上下文）的指针
*app_start_timer() 调用超时处理程序。
*/
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief 用于填充模拟健康温度计测量的功能。
*/
static void hts_sim_measurement(ble_hts_meas_t * p_meas)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };

    uint32_t celciusX100;

    p_meas->temp_in_fahr_units = false;
    p_meas->time_stamp_present = true;
    p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

    celciusX100 = sensorsim_measure(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);

    p_meas->temp_in_celcius.exponent = -2;
    p_meas->temp_in_celcius.mantissa = celciusX100;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
    p_meas->time_stamp               = time_stamp;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

    //更新模拟时间戳
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}


/**@brief 用于定时器初始化的函数。
*
*@details 初始化定时器模块。这将创建并启动应用程序计时器。
*/
static void timers_init(void)
{
    ret_code_t err_code;

    //初始化定时器模块。
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    //创建计时器。
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于 GAP 初始化的函数。
*
*@details 该函数设置所有必需的 GAP（通用访问配置文件）参数
*设备包括设备名称、外观和首选连接参数。
*/

static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                           strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于初始化 GATT 模块的函数。
*/
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于模拟和发送一个温度测量值的函数。
*/
static void temperature_measurement_send(void)
{
    ble_hts_meas_t simulated_meas;
    ret_code_t     err_code;

    if (!m_hts_meas_ind_conf_pending)
    {
        hts_sim_measurement(&simulated_meas);

        err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);

        switch (err_code)
        {
            case NRF_SUCCESS:
                //测量结果已成功发送，等待确认。
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                //忽略错误。
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}


/**@brief 用于处理健康温度计服务事件的函数。
*
*@details 将为所有传递的健康温度计服务事件调用此函数
*到应用程序。
*
*@param[in] p_hts 健康温度计服务结构。
*@param[in] p_evt 从健康温度计服务收到的事件。
*/
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            //指示已启用，发送单个温度测量值
            temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            //无需实施。
            break;
    }
}


/**@brief 用于处理排队写入模块错误的函数。
*
*@details 指向此函数的指针将传递给可能需要通知的每个服务
*应用程序出现错误。
*
*@param[in] nrf_error 包含错误信息的错误代码。
*/
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief 用于初始化应用程序将使用的服务的函数。
*
*@details 初始化健康温度计、电池和设备信息服务。
*/
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hts_init_t     hts_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dis_sys_id_t   sys_id;

    //初始化排队写入模块。
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    //初始化健康体温计服务
    memset(&hts_init, 0, sizeof(hts_init));

    hts_init.evt_handler                 = on_hts_evt;
    hts_init.p_gatt_queue                = &m_ble_gatt_queue;
    hts_init.error_handler               = service_error_handler;
    hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    //此处可以更改/增加健康温度计服务的秒级别。
    hts_init.ht_meas_cccd_wr_sec = SEC_JUST_WORKS;
    hts_init.ht_type_rd_sec      = SEC_OPEN;

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);

    //初始化电池服务。
    memset(&bas_init, 0, sizeof(bas_init));

    //此处可以更改/增加电池服务的秒级别。
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    //初始化设备信息服务。
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于初始化传感器模拟器的函数。
*/
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    //温度以摄氏度为单位（乘以 100 以避免浮点运算）。
    m_temp_celcius_sim_cfg.min          = MIN_CELCIUS_DEGREES;
    m_temp_celcius_sim_cfg.max          = MAX_CELCIUS_DEGRESS;
    m_temp_celcius_sim_cfg.incr         = CELCIUS_DEGREES_INCREMENT;
    m_temp_celcius_sim_cfg.start_at_max = false;

    sensorsim_init(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);
}


/**@brief 用于启动应用程序计时器的函数。
*/
static void application_timers_start(void)
{
    ret_code_t err_code;

    //启动应用程序计时器。
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于处理连接参数模块的函数。
*
*@details 连接参数模块中的所有事件都会调用此函数
*传递给应用程序。
*@note 这个函数所做的只是断开连接。这可以通过简单地完成
*设置disconnect_on_fail配置参数，但我们使用事件
*处理程序机制来演示其使用。
*
*@param[in] p_evt 从连接参数模块接收的事件。
*/
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief 用于处理连接参数错误的函数。
*
*@param[in] nrf_error 包含错误信息的错误代码。
*/
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief 用于初始化连接参数模块的函数。
*/
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief 将芯片置于睡眠模式的功能。
*
*@note 这个函数不会返回。
*/
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    //准备唤醒按钮。
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    //进入系统关闭模式（此函数不会返回；唤醒将导致重置）。
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于处理广告事件的函数。
*
*@details 将针对传递给应用程序的广告事件调用此函数。
*
*@param[in] ble_adv_evt 广告事件。
*/
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief 用于处理 BLE 事件的函数。
*
*@param[in] p_ble_evt 蓝牙堆栈事件。
*@param[in] p_context 未使用。
*/
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
            m_hts_meas_ind_conf_pending = false;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            //在 GATT 客户端超时事件上断开连接。
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            //发生 GATT 服务器超时事件时断开连接。
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            //无需实施。
            break;
    }
}


/**@brief 用于初始化 BLE 堆栈的函数。
*
*@details 初始化 SoftDevice 和 BLE 事件中断。
*/
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    //使用默认设置配置 BLE 堆栈。
//获取应用程序 RAM 的起始地址。
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    //启用 BLE 堆栈。
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    //注册 BLE 事件的处理程序。
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief 用于处理来自 BSP 模块的事件的函数。
*
*@param[in] event 按下按钮生成的事件。
*/
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                temperature_measurement_send();
            }
            break;

        default:
            break;
    }
}


/**@brief 用于对等管理器初始化的函数。
*/
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    //用于所有安全程序的安全参数。
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief 从持久存储中清除债券信息。
*/
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于初始化广告功能的函数。
*
*@details 对所需的广告数据进行编码并将其传递到堆栈。
*还构建了一个在开始广告时传递到堆栈的结构。
*/
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief 用于初始化按钮和 LED 的函数。
*
*@param[out] p_erase_bonds 如果按下清除绑定按钮来唤醒应用程序，则为 true。
*/
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief 用于初始化nrf日志模块的函数。
*/
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief 用于初始化电源管理的函数。
*/
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief 用于处理空闲状态的函数（主循环）。
*
*@details 如果没有挂起的日志操作，则休眠直到下一个事件发生。
*/
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief 启动广告的功能。
*/
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        //广告由 PM_EVT_PEERS_DELETE_SUCCEEDED 事件启动。
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
//static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


/**@brief 应用程序主入口的函数。
*/
int main(void)
{
    bool erase_bonds;

    

    //初始化。
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);

    twi_init();
    //QMI_set_mode();
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    //APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    //NRF_LOG_DEFAULT_BACKENDS_INIT();

    //NRF_LOG_INFO("TWI scanner started.");
    //NRF_LOG_FLUSH();
    //twi_init();

    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            //if(address==1){MODEL_NUM="bbbbb";}
            if (address!=1)
            {
                //if(address==0x6AU){MODEL_NUM="1234";}
                //MODEL_NUM="ftgyuh";
              
                //MODEL_NUM[0] = (char)(((int)address));  
                //MODEL_NUM[1] = '\0'; // 添加空字符以标记字符串的结尾  
                break;
            }
            
            //NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
        }
        //NRF_LOG_FLUSH();
    }
    //if (address==1){MODEL_NUM="1234567";}

    if (!detected_device)
    {
        //MODEL_NUM="SDFG";
        //NRF_LOG_INFO("No device was found.");
        //NRF_LOG_FLUSH();
    }

    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();


    //ret_code_t err_code;
    uint8_t reg[2] = {QMI_CTRL_7, QMI_MODE_1};
    err_code = nrf_drv_twi_tx(&m_twi, QMI_ADDR, reg, sizeof(reg), false);
    //APP_ERROR_CHECK(err_code);

    uint8_t reg1[2] = {QMI_CTRL_2, QMI_MODE_2};
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, QMI_ADDR, reg1,sizeof(reg1), false);
    //APP_ERROR_CHECK(err_code);

    reg1[1] = QMI_TEMP_L;
    err_code = nrf_drv_twi_tx(&m_twi, QMI_ADDR, reg1,1, false);
    //APP_ERROR_CHECK(err_code);

    /*从温度传感器读取样本的缓冲区。*/
    static uint8_t m_sample;

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, QMI_ADDR, &m_sample, sizeof(m_sample));
    //APP_ERROR_CHECK(err_code);
    
    //QMI_set_mode();
    //read_sensor_data();
    MODEL_NUM="0000";
    if(m_sample<26){MODEL_NUM="NNNN";}
    if(m_sample<20){MODEL_NUM="SSSS";}
    if(m_sample<15){MODEL_NUM="DDDD";}
    if(m_sample<10){MODEL_NUM="AAAA";}
    //开始执行。
    NRF_LOG_INFO("Health Thermometer example started.");
    application_timers_start();
    advertising_start(erase_bonds);

    //进入主循环。
    for (;;)
    {
        idle_state_handle();
    }
}


/**
*@}


    
    if(address<0x25U){MODEL_NUM="NNNN";}
    if(address>0x27U){MODEL_NUM="SSSS";}
    if(address==0x00U){MODEL_NUM="0000";}
    if(address==0x27U){MODEL_NUM="AAAA";}
    if(address==0x28U){MODEL_NUM="BBBB";}/////////
    if(address==0x29U){MODEL_NUM="CCCC";}
    if(address==0x2DU){MODEL_NUM="DDDD";}

while (true)
    {
        nrf_delay_ms(500);

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        read_sensor_data();
        NRF_LOG_FLUSH();
    }


*/
