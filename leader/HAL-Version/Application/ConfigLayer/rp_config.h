#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "arm_math.h"
/* Exported macro ------------------------------------------------------------*/
/* ʱ��� */
#define		TIME_STAMP_250MS	250
#define 	TIME_STAMP_500MS	500
#define    	TIME_STAMP_400MS    400
#define		TIME_STAMP_5000MS	5000

/* Exported types ------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
 *	@brief	��������
 *	@class	driver
 */
typedef enum drv_type{
	DRV_CAN1,
	DRV_CAN2,
	DRV_PWM_FRIC_L,
	DRV_PWM_FRIC_R,
	DRV_PWM_SERVO,
	DRV_IIC,
	DRV_UART1,
	DRV_UART2,
	DRV_UART3,
	DRV_UART4,
	DRV_UART5,
} drv_type_t;

/**
 *	@brief	iic����
 *	@class	driver
 */
typedef struct drv_iic {
	enum drv_type 	type;	
} drv_iic_t;

/**
 *	@brief	can����
 *	@class	driver
 */
typedef struct drv_can {
	enum drv_type 	type;
	uint32_t		can_id;
	uint32_t		std_id;
	uint8_t			drv_id;
	void			(*tx_data)(struct drv_can *self, int16_t txData);
} drv_can_t;

/**
 *	@brief	pwm����
 *	@class	driver
 */
typedef struct drv_pwm {
	enum drv_type	type;
	void			(*output)(struct drv_pwm *self, int16_t pwm);
} drv_pwm_t;

/**
 *	@brief	uart����
 *	@class	driver
 */
typedef struct drv_uart {
	enum drv_type	type;
	void			(*tx_byte)(struct drv_uart *self, uint8_t byte);
} drv_uart_t;

/* �豸�� --------------------------------------------------------------------*/
/**
 *	@brief	�豸id�б�
 *	@class	device
 */
typedef enum {
	DEV_ID_RC = 0,
	DEV_ID_IMU = 1,
	DEV_ID_CHASSIS_LF = 2,
	DEV_ID_CHASSIS_RF = 3,
	DEV_ID_CHASSIS_LB = 4,
	DEV_ID_CHASSIS_RB = 5,
	DEV_ID_CNT = 6,
} dev_id_t;

/**
 *	@brief	���̵���豸����
 *	@class	device
 */
typedef enum {
	CHAS_LF,
	CHAS_RF,
	CHAS_LB,
	CHAS_RB,
	CHAS_MOTOR_CNT,
} chassis_motor_cnt_t;

/**
 *	@brief	�豸����״̬
 *	@class	device
 */
typedef enum {
	DEV_ONLINE,
	DEV_OFFLINE,
} dev_work_state_t;

/**
 *	@brief	�������
 *	@class	device
 */
typedef enum {
	NONE_ERR,		// ����(�޴���)
	DEV_ID_ERR,		// �豸ID����
	DEV_INIT_ERR,	// �豸��ʼ������
	DEV_DATA_ERR,	// �豸���ݴ���
} dev_errno_t;

/**
 *	@brief	�豸�ṹ�嶨��ģ��
 *	@class	device
 */
typedef struct device {
	void				*info;		// �Զ�������豸��Ϣ�ṹ��
	void				*driver;	// �Զ�������豸�����ṹ��
	void				(*init)(struct device *self);	// �豸��ʼ������
	void				(*update)(struct device *self, uint8_t *rxBuf);	// �豸���ݸ��º���
	void				(*check)(struct device *self);	// �豸���ݼ�麯��
	void				(*heart_beat)(struct device *self);	// �豸������
	dev_work_state_t	work_state;	// �豸����״̬
	dev_errno_t			errno;		// ���Զ�������豸�������
	dev_id_t			id;			// �豸id
} device_t;

/* Ӧ�ò� --------------------------------------------------------------------*/
/**
 *	@brief	pid������
 *	@class	controller
 */
typedef struct pid_ctrl {
	float	target;
	float	measure;
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float	integral;
	float 	integral_max;
	float 	out_max;
} pid_ctrl_t;

/* Remote Mode Enum */
typedef enum {
	RC = 0,
	KEY = 1,
	REMOTE_MODE_CNT = 2,
} remote_mode_t;

typedef enum {
	SYS_STATE_NORMAL,	// ϵͳ����
	SYS_STATE_RCLOST,	// ң��ʧ��
	SYS_STATE_RCERR,	// ң�س���
	SYS_STATE_WRONG,	// ����ϵͳ����
} sys_state_t;

typedef enum {
	SYS_MODE_NORMAL,	// ����ģʽ
	SYS_MODE_CNT,
} sys_mode_t;

//typedef struct {
//	struct {
//		uint8_t reset_start;
//		uint8_t reset_ok;
//	}gimbal;
//} flag_t;

typedef struct {
	remote_mode_t		remote_mode;	// ���Ʒ�ʽ
	sys_state_t			state;			// ϵͳ״̬
	sys_mode_t			mode;			// ϵͳģʽ
} system_t;

//extern flag_t	flag;
extern system_t sys;

/* Exported functions --------------------------------------------------------*/
//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(uint32_t addr);	//���ö�ջ��ַ 

#endif