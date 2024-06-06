/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "main.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME
static struct k_work_delayable button_enable_work;
static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;
#if deUartFunction
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static const struct device *const MCU_uart = DEVICE_DT_GET(UART0_DEVICE_MCU);
static const struct device *const RFID_uart = DEVICE_DT_GET(UART1_DEVICE_MCU);
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
#endif
#if deButtonFunction
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,{0});
static const struct gpio_dt_spec Rxbutton = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios,{0});
static struct gpio_callback button_cb_data;
#endif
struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

#if deButtonFunction
#if deCapRdyButton
void CapRdyButton_enable_work_handler(struct k_work *work)
{
    LOG_INF("Re-enabling button interrupts.");
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
}
void CapRdyButtonInterrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
#if deLED2_Chack2
	if(u8LED3_SwitchFlag == deDisable){
		u8LED3_SwitchFlag = deEnable;
		de_LED4_ON;
	}
	else{
		u8LED3_SwitchFlag = deDisable;
		de_LED4_OFF;
	}
	// u8RxPinChangeLock = deEnable;
#endif	
	
}
void CapReadyPinInit(void){
	int ret;
	int err = 0;	
	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}
	
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}
		ret = gpio_pin_interrupt_configure_dt(&button,GPIO_INT_EDGE_TO_ACTIVE );
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, CapRdyButtonInterrupt, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	k_work_init_delayable(&button_enable_work, CapRdyButton_enable_work_handler);
	// gpio_pin_interrupt_configure_dt(&Rxbutton, GPIO_INT_DISABLE);
	
}
#endif
#if deRxButtonfunc
void RxButton_enable_work_handler(struct k_work *work)
{
    LOG_INF("Re-enabling button interrupts.");
    gpio_pin_interrupt_configure_dt(&Rxbutton, GPIO_INT_EDGE_TO_ACTIVE);
}
void RxButtonInterrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
#if deLED2_Chack2
	if(u8LED2_SwitchFlag == deDisable){
		u8LED2_SwitchFlag = deEnable;
		de_LED3_ON;
	}
	else{
		u8LED2_SwitchFlag = deDisable;
		de_LED3_OFF;
	}
	// u8RxPinChangeLock = deEnable;
#endif	
	
}

void RxButtonEn(void){
	int ret;
	int err = 0;	
	if (!gpio_is_ready_dt(&Rxbutton)) {
		printk("Error: button device %s is not ready\n",
		       Rxbutton.port->name);
		return 0;
	}
	
	ret = gpio_pin_configure_dt(&Rxbutton, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, Rxbutton.port->name, Rxbutton.pin);
		return 0;
	}
		ret = gpio_pin_interrupt_configure_dt(&Rxbutton,GPIO_INT_EDGE_TO_ACTIVE );
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, Rxbutton.port->name, Rxbutton.pin);
		return 0;
	}
	
	gpio_init_callback(&button_cb_data, RxButtonInterrupt, BIT(Rxbutton.pin));
	gpio_add_callback(Rxbutton.port, &button_cb_data);
	k_work_init_delayable(&button_enable_work, RxButton_enable_work_handler);
	
}
#endif
#endif

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;
	if (buttons & KEY_PASSKEY_ACCEPT) {
		if(u8LED3_SwitchFlag == deDisable){
			u8LED3_SwitchFlag = deEnable;
			de_LED4_ON;
		}
		else{
			u8LED3_SwitchFlag = deDisable;
			de_LED4_OFF;
		}	
	}
	if (buttons & KEY_PASSKEY_REJECT) {
		if(u8LED2_SwitchFlag == deDisable){
			u8LED2_SwitchFlag = deEnable;
			de_LED3_ON;
		}
		else{
			u8LED2_SwitchFlag = deDisable;
			de_LED3_OFF;
		}
	}
#if deButtonSampleCode
	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
#endif	
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

#if deMCU_URFunc
void MCU_DataPro(void){ //MCU Data Process
	int err,ret;
	if(u8MCUURGDDFlag == deEnable){		
#if deLED2_Chack1		
		if(u8LED2_SwitchFlag == deDisable){
			u8LED2_SwitchFlag = deEnable;
			de_LED2_ON;
		}
		else{
			u8LED2_SwitchFlag = deDisable;
			de_LED2_OFF;
		}
#endif
		switch(u8OB_DataBuf[u8OB_DataLen]){
			case '#':
				switch(u8OB_DataBuf[u8OB_DataLen-1]){
					case '5':
#if deLED1_Chack3
						if(u8LED1_SwitchFlag == deDisable){
							u8LED1_SwitchFlag = deEnable;
							de_LED2_ON;
							err = dk_buttons_init(button_changed);
							if (err) {
								LOG_ERR("Cannot init buttons (err: %d)", err);
							}

							// ret = gpio_pin_configure_dt(&Rxbutton, GPIO_INPUT);
							// if (ret != 0) {
								// printk("Error %d: failed to configure %s pin %d\n",
									   // ret, Rxbutton.port->name, Rxbutton.pin);
								// return 0;
							// }

							// err = dk_buttons_init(button_changed);
							// err = gpio_pin_interrupt_configure_dt(&button,GPIO_INT_DISABLE);
							// if (err) {
								// LOG_ERR("Cannot disable callbacks()");
								// return err;
							// }
							// err = dk_buttons_init(button_changed);
							// if (err) {
								// LOG_ERR("Cannot init buttons (err: %d)", err);
							// }
#if deRxButtonfunc							
							// RxButtonEn();
							// gpio_pin_interrupt_configure_dt(&Rxbutton, GPIO_INT_DISABLE);
#endif
						}
						else{
							u8LED1_SwitchFlag = deDisable;
							de_LED2_OFF;
							err = dk_buttons_init(NULL);
							if (err) {
								LOG_ERR("Cannot init buttons (err: %d)", err);
							}
							CapReadyPinInit();
							// RxButtonEn();
							// CapReadyPinInit();
							// ret = gpio_pin_configure_dt(&Rxbutton, GPIO_DISCONNECTED);
							// if (ret != 0) {
								// printk("Error %d: failed to configure %s pin %d\n",
									   // ret, Rxbutton.port->name, Rxbutton.pin);
								// return 0;
							// }

							// CapReadyPinInit();
							// err = gpio_pin_interrupt_configure_dt(&Rxbutton,GPIO_INT_DISABLE);
							// if (err) {
								// LOG_ERR("Cannot disable callbacks()");
								// return err;
							// }

#if deRxButtonfunc							
							// RxButtonEn();
							gpio_pin_interrupt_configure_dt(&Rxbutton, GPIO_INT_DISABLE);
#endif
						}
#endif					
						// u8RxPinChangeLock = deEnable;
					break;
					default:
					break;
				}
			break;
			
			default:
			break;
		}
		u8MCUURGDDFlag = deDisable;	
	}
}

void MCU_SendData(uint8_t* MCU_DataBuff,uint8_t MCU_DataLen){
	for(int i = 0; i < MCU_DataLen;i++){
		uart_poll_out(MCU_uart, MCU_DataBuff[i]);
	}
}
void MCU_UR_Interrupt(const struct device *dev, void *user_data){
	uint8_t c;
	if (!uart_irq_update(MCU_uart)) {
		return;
	}
	if (!uart_irq_rx_ready(MCU_uart)) {
		return;
	}
	/* read until FIFO empty */
	while (uart_fifo_read(MCU_uart, &c, 1) == 1) {
//check Enter Interrupt	
#if deLED1_Chack1
		if(u8LED1_SwitchFlag == deDisable){
			u8LED1_SwitchFlag = deEnable;
			de_LED1_ON;
		}
		else{
			u8LED_SwitchFlag = deDisable;
			de_LED1_OFF;
		}
#endif		
#if deMCU_UR_SampleCode		
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
#endif
#if deMCU_URFunc //iF Plus communication protocol
		if(c == deMCU_CommEndNum || u8MCUURGD_Flag == deEnable){
			u8MCUUR_Buffer[u16MCUUR_BufNum] = c;
			if(c != deMCU_CommEndNum){
				if(u16MCUUR_BufNum >= 3){
					u8MCUURDataLen = u8MCUUR_Buffer[2];
					switch(u8MCUUR_Buffer[1]){
/* 						case deBLE_DataMod:
						break;
						
						case deRFID_DataMod:
						break;*/						
						case deOB_DataMod:
#if deUR_CheckValue1
						u8Debug1ValueBuf[1] = u16MCUUR_BufNum;
						MCU_SendData(u8Debug1ValueBuf,3);				
						u8Debug2ValueBuf[1] = u16MCUUR_BufNum-3;
						MCU_SendData(u8Debug2ValueBuf,3);				
#endif
						u8OB_DataBuf[u16MCUUR_BufNum-3] = c;
						u8OB_DataLen = u16MCUUR_BufNum-3;
#if deUR_CheckValue1
						MCU_SendData(u8OB_DataBuf,3);				
#endif							
						break;
						
						default:
						break;
					}
				}
				u16MCUUR_BufNum++;

			}
			else{
				u8MCUUR_Buffer[u16MCUUR_BufNum] = c;
				u8MCUURCRC = u8MCUUR_Buffer[u16MCUUR_BufNum-1];
				u16MCUUR_BufNum = deValueReset;
				u8MCUURGD_Flag = deDisable;
				c = deValueReset;
				u8MCUURGDDFlag = deEnable;
// check End byte 0x81 Enter function
#if deLED1_Chack2
				if(u8LED1_SwitchFlag == deDisable){
					u8LED1_SwitchFlag = deEnable;
					de_LED1_ON;
				}
				else{
					u8LED1_SwitchFlag = deDisable;
					de_LED1_OFF;
				}
#endif		

			}				
		}
		else if(c == deMCU_CommStartNum){
			u8MCUURGD_Flag = deEnable;
			u8MCUUR_Buffer[u16MCUUR_BufNum] = c;
			u16MCUUR_BufNum++;

		}
#endif
	}
}
void MCU_UR_Init(void){
	char tx_buf[MSG_SIZE];
	// int ret
	if (!device_is_ready(MCU_uart)) {
		printk("UART device not found!");
		return 0;
	}
	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(MCU_uart, MCU_UR_Interrupt, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}	
	deMCU_UR_On;
}

#endif
#if deUR_SampleCode
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data){
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data[0]);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}
static void uart_work_handler(struct k_work *item){
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}
static bool uart_test_async_api(const struct device *dev){
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}
static int uart_init(void){
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}
#endif
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	// err = dk_buttons_init(button_changed);
	// if (err) {
		// LOG_ERR("Cannot init buttons (err: %d)", err);
	// }
	
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}
int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();
#if deMCU_URFunc	
	MCU_UR_Init();
#endif	
#if deUR_SampleCode
	err = uart_init();
	if (err) {
		error();
	}
#endif
	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}
#if deRxButtonfunc
	RxButtonEn();
	// gpio_pin_interrupt_configure_dt(&Rxbutton, GPIO_INT_DISABLE);

#endif	
#if deCapRdyButton
	CapReadyPinInit();
#endif
	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}
	for (;;) {
		dk_set_led(DK_LED1, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
#if deMCU_URFunc		
		MCU_DataPro();
#endif	
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
