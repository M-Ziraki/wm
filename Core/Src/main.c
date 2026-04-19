/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
// #define DRAIN_ENABLED //for disable or enable first drain ######################################
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SocketDma.h"
#include "stm32f1xx_it.h"
#include "stdio.h"
#include "math.h"
// first git commit
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	MOTOR_RELAY_CMD_OFF = 0,
	MOTOR_RELAY_CMD_UI1,
	MOTOR_RELAY_CMD_UI2
} MotorRelayCommand;

typedef enum
{
	MOTOR_RELAY_STATE_STABLE = 0,
	MOTOR_RELAY_STATE_HOLD_CURRENT,
	MOTOR_RELAY_STATE_DEADTIME,
	MOTOR_RELAY_STATE_SETTLE
} MotorRelayState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
_Bool run_flag, run_motor = 0, washing_flag, washst_flag, push_flag, first_turn_motor = 0, first_grab = 0, turn_motor = 0, mission_flag, end_flag = 0, ms250_flag = 0;
_Bool spin_mode, rxw_flag, prewashmode = 0, washmode, key_flag, DrainP_temp, verify_flag = 0, smt_flag = 0, runmotorc_flag = 0, ms1000_flag = 0; ////////////0
_Bool Valve3_temp, Valve2_temp, Valve1_temp, needheat_flag, pause_flag, Heater_temp, Error_sflag = 0, allow_relay, ressa = 0, procng_flag = 0, EEErase = 0;
_Bool txw_flag, reg_flag = 1, unreg_flag = 0, Error_flag = 0, resume_flag, stop_flag, hydroread_flag, firstadc_flag = 1, DnOpDr_flag = 0, oow_flag = 1;
_Bool E01_flag = 0, E02_flag = 0, E11_flag = 0, E21_flag = 0, E22_flag = 0, E23_flag = 0, E24_flag = 0, E30_flag = 0, E31_flag = 0, E32_flag = 0, E33_flag = 0, vaoff_flag = 0, mixok_flag = 0, hydp_flag = 0, hyde_flag = 0, hydro_flag = 0, hydpin_flag = 0, hydpin2_flag = 0;
_Bool E34_flag = 0, E35_flag = 0, E39_flag = 0, E51_flag = 0, E61_flag = 0, E62_flag = 0, E63_flag = 0, coldwash_flag = 0, E41_flag = 0, hwater_flag = 0, wef_flag = 0, drm_flag = 0, drt_flag = 0, dr2_flag = 0;
_Bool dml_flag = 0, dmr_flag = 0, ddr_flag = 0, dv1_flag = 0, dv2_flag = 0, dv3_flag = 0, ddl_flag = 0, dhr_flag = 0, dt30_flag = 0, spchg_flag = 0, notsave_flag = 0;
_Bool dwl1_flag = 0, dwl2_flag = 0, debug_show = 0, showfb_flag = 0, ChLk_flag = 0, plkey_flag = 0, plkey_flag2 = 0, progs_flag = 0, wshc_flag = 0, norx_flag = 0;
_Bool l1ok_flag = 0, l2ok_flag = 0, buzz = 0, ee30_flag = 0, ee3s_flag = 0, eef_flag = 0, eee_flag = 0, doorlock_flag = 0, test_flag = 0, swdr_flag = 0, chgdr_flag = 0;
_Bool right_flag = 0, left_flag = 0, changetr_flag = 0, savewtr_flag = 0, nodel30_flag = 0, woolmode = 0, error_cmd = 0, tmhw_flag = 1, dhw_flag = 0, eesaves_flag = 0;
_Bool w60_flag = 0, w90_flag = 0, pw60_flag = 0, pw90_flag = 0, tmf_flag = 0, tmt_flag = 0, tmmr_flag = 0, tmml_flag = 0, tmr_flag = 0, stayoff = 0;
_Bool tmd_flag = 0, tmv1_flag = 0, tmv2_flag = 0, tmv3_flag = 0, delay_start_flag = 0, twiceturn_flag, pl_flag = 0, fourturn_flag = 0, OpRy_flag = 0;
_Bool lockdoor_flag = 0, m1_flag = 0, m2_flag = 0, m3_flag = 0, m4_flag = 0, m5_flag = 0, fto_flag = 0, ea_flag = 0, autodebug_flag = 0, nosensor_flag = 0, dnose_flag = 0;
_Bool Dlmn_flag, Flmn_flag = 0, Glmn_flag, Hlmn_flag, Ilmn_flag, Jlmn_flag, Klmn_flag, Llmn_flag, Mlmn_flag, Nlmn_flag, Olmn_flag, nospin_flag = 0, E3c_flag = 0;
_Bool Plmn_flag, Qlmn_flag, Rlmn_flag, Slmn_flag, Tlmn_flag, Ulmn_flag, Vlmn_flag, Wlmn_flag, Ylmn_flag, Zlmn_flag, E37_flag = 0, E38_flag = 0, E5lchk_flag = 0;

_Bool firstword_flag = 0, secondword_flag = 0, firstwordE_flag = 0, secondwordE_flag = 0, trig_flag = 0, trig_flag2 = 0, onems_flag = 0, drercl_flag = 0;
_Bool tccnt_flag = 0, Debug_flag = 0, err_num = 0, G2lmn_flag = 0, onwm_flag = 0, RelaeaseOok_flag = 0, sosp_flag = 0;

uint8_t c = 0, z = 0, tc_cnt = 0, tt = 88, pp = 88, num2 = 88, nrcnt = 0, nr3 = 0, ll = 0, rr = 0, CC = 0, aa = 0, vfc = 0, cpi = 0, cpd = 0, cpf = 0, stoffcnt = 0, nba = 0, hydpin_cnt = 0;
uint16_t x = 0, x22 = 0, d = 850, taco_cnt = 0, taco_cnt2 = 0, taco_cnt3 = 0, taco_cnt4 = 0, taco_cnt5 = 0, hyd_cnt = 0, tc = 0, ttt = 0, num3 = 0, mm[40], HHH = 0, ww = 88, ss = 88;
uint16_t Dlmn = 0, Flmn = 0, Glmn = 0, Hlmn = 0, Ilmn = 0, Jlmn = 0, Klmn = 0, Llmn = 0, Mlmn = 600, Nlmn = 0, Olmn = 0, Plmn = 0, Qlmn = 0, Rlmn = 0, Slmn = 0, Tlmn = 0, Ulmn = 0, Vlmn = 0, Wlmn = 0, Ylmn = 0, Zlmn = 0;

uint8_t ready_scnt, pausing_scnt, washing_scnt, rinsing_scnt, spinning_scnt, buzzercnt = 1, fastspin = 0, buzzoff = 0, Ewp, Erns, Eps, Ern, verify_cnt = 0;
uint8_t y = 0, j = 0, washing_cnt = 0, unreg_scnt, taco_stop_tmr = 0, resume_cnt = 1, pause_cnt = 1, dlcntt = 0, eesc = 0, adcnt = 0, LkDrEr = 0, spchcnt = 0, cmm_cnt = 0;
uint8_t rinsemode = 2, hydro_timer = 0, E23_cnt = 0, E24_cnt = 0, E34_cnt = 0, E51_cnt = 0, tm4 = 0, E63_cnt, washing_time = 0, E00_scnt = 1, tct = 0, chgdr_cnt = 11;
uint8_t program_select = 0, HydEr_timer = 0, Error_timer = 0, tm1 = 0, rinsetime_cnt = 0, turn_cnt = 0, wprg = 0, tm5 = 0, wrck = 4, wrckout = 0, relaytmr = 0;
uint8_t timer_show = 0, ttemp2_cnt = 0, maxtaco = 22, mintaco = 21, cc = 0, spintime = 0, tacosafe = 0, prewashing_time = 0, cooling_timer = 0, hydchk = 0, numi = 0;
uint8_t washresttime = 3, washturntime = 20, newresttime = 3, newturntime = 20, min_cnt = 0, fourturn_cnt = 0, fourtime = 0, tcpl = 0;
uint8_t tol = 0, rest_tmr = 0, wtme = 0, dtme = 0, stme = 0, rtme = 0, grab_cnt = 0, plm = 30, plm2 = 0, plm3 = 26, plm4 = 25, doorlock_cnt = 0, bsc = 0, HHl[5];
uint16_t k = 0, k2 = 0, tm2 = 0, mission_timer = 0, spinspeed = 0, E21_timer = 0, adc_rd2 = 0, adcsum[5], tmsb = 1;
uint16_t motor_speed = 600, motor_speed_start = 600, motor_speed_slctL = 600, motor_speed_slctR = 600;
uint16_t EEDATA, EEDATA2, tm3 = 0, hydro_temp = 0, delay_timer = 0, stop_off_timer = 0, washcasetime = 0, wshcnt = 0;
uint16_t reftemp1 = 10, hydro2, hydro3, hydsum[5], hydroempty, hydrofull, hydroempty2, offcnt = 0, wefcnt = 0;
uint16_t yx = 0, yw2 = 0, Beta = 0, Ems, Eml, Emr, EHl, yw = 0, yz = 0, hs = 298, runmotorcnt = 0, cm_cnt = 0;
uint32_t adc_rd3 = 0;
float T = 0, T_temp, T_temp2, T2;
char bufftest[60], bufftest2[10], bufftest3[5], bufftest4[8], EEkey[1];
extern char sb[100], rb[1], cb[100], counter;

uint8_t pscnt = 0;
uint16_t hcnt = 0;
uint32_t Rif = 0;
uint16_t adc_buf[2] = {5000, 5000};
MotorRelayCommand motor_relay_target = MOTOR_RELAY_CMD_OFF;
MotorRelayCommand motor_relay_applied = MOTOR_RELAY_CMD_OFF;
MotorRelayState motor_relay_state = MOTOR_RELAY_STATE_STABLE;
uint16_t motor_relay_timer_ms = 0;
_Bool master_relay_target = 0, master_relay_on = 0, master_relay_off_pending = 0;
uint16_t master_relay_timer_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static void motor_relay_apply_hw(MotorRelayCommand cmd);
static void motor_relay_force_reset(void);
static void motor_relay_request(MotorRelayCommand cmd);
static void motor_relay_service_1ms(void);
static void master_relay_force_reset(void);
static void master_relay_request(_Bool on);
static void master_relay_service_1ms(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void key_func(void);
void onems_func(void);
void ms250_func(void);
void ms1000_func(void);
void washing_func(void);
void mission_func(void);
void stop_func(void);
void spchg_func(void);
void resume_func(void);
void pause_func(void);
void EEwr_func(void);
void EErd_func(void);
void EEsp_func(void);
void EElp_func(void);
void lockdoor_func(void);
void calT(void);
void calBR(void);
void btc_func(void);
void procng(void);
void NumToStr(void);
void eesaves_func(void);
void Debug_func(void);
void str2num(void);
void autodebug(void);
void BeforeSleep(void);
void EraseVar_func(void);
void check_E(void);

#define MOTOR_RELAY_HOLD_OFF_MS 2000U
#define MOTOR_RELAY_DEADTIME_MS 100U
#define MOTOR_RELAY_SETTLE_MS 100U

static void motor_relay_apply_hw(MotorRelayCommand cmd)
{
	GPIO_PinState ui1_state = GPIO_PIN_RESET;
	GPIO_PinState ui2_state = GPIO_PIN_RESET;

	if (cmd == MOTOR_RELAY_CMD_UI1)
		ui1_state = GPIO_PIN_SET;
	else if (cmd == MOTOR_RELAY_CMD_UI2)
		ui2_state = GPIO_PIN_SET;

	HAL_GPIO_WritePin(Motor_Ui1_GPIO_Port, Motor_Ui1_Pin, ui1_state);
	HAL_GPIO_WritePin(Motor_Ui2_GPIO_Port, Motor_Ui2_Pin, ui2_state);
}

static void motor_relay_force_reset(void)
{
	motor_relay_target = MOTOR_RELAY_CMD_OFF;
	motor_relay_applied = MOTOR_RELAY_CMD_OFF;
	motor_relay_state = MOTOR_RELAY_STATE_STABLE;
	motor_relay_timer_ms = 0;
	allow_relay = 0;
	relaytmr = 0;
	motor_relay_apply_hw(MOTOR_RELAY_CMD_OFF);
}

static void master_relay_force_reset(void)
{
	master_relay_target = 0;
	master_relay_on = 0;
	master_relay_off_pending = 0;
	master_relay_timer_ms = 0;
	HAL_GPIO_WritePin(MasterRLY_GPIO_Port, MasterRLY_Pin, GPIO_PIN_RESET);
}

static void master_relay_request(_Bool on)
{
	master_relay_target = on;

	if (on)
	{
		master_relay_off_pending = 0;
		master_relay_timer_ms = 0;
		if (!master_relay_on)
		{
			HAL_GPIO_WritePin(MasterRLY_GPIO_Port, MasterRLY_Pin, GPIO_PIN_SET);
			master_relay_on = 1;
		}
	}
	else if (!master_relay_on)
	{
		master_relay_off_pending = 0;
		master_relay_timer_ms = 0;
	}
	else if (!master_relay_off_pending)
	{
		master_relay_off_pending = 1;
		master_relay_timer_ms = MOTOR_RELAY_HOLD_OFF_MS;
	}
}

static void master_relay_service_1ms(void)
{
	if (master_relay_target)
	{
		master_relay_off_pending = 0;
		master_relay_timer_ms = 0;
		if (!master_relay_on)
		{
			HAL_GPIO_WritePin(MasterRLY_GPIO_Port, MasterRLY_Pin, GPIO_PIN_SET);
			master_relay_on = 1;
		}
		return;
	}

	if (master_relay_off_pending)
	{
		if (master_relay_timer_ms > 0)
			master_relay_timer_ms--;
		if (master_relay_timer_ms == 0)
		{
			HAL_GPIO_WritePin(MasterRLY_GPIO_Port, MasterRLY_Pin, GPIO_PIN_RESET);
			master_relay_on = 0;
			master_relay_off_pending = 0;
		}
	}
}

static void motor_relay_request(MotorRelayCommand cmd)
{
	if (motor_relay_target != cmd)
	{
		motor_relay_target = cmd;
		HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
		allow_relay = 0;
		relaytmr = 0;

		if (motor_relay_state == MOTOR_RELAY_STATE_STABLE)
		{
			if (motor_relay_applied != MOTOR_RELAY_CMD_OFF)
			{
				motor_relay_state = MOTOR_RELAY_STATE_HOLD_CURRENT;
				motor_relay_timer_ms = MOTOR_RELAY_HOLD_OFF_MS;
			}
			else if (motor_relay_target != MOTOR_RELAY_CMD_OFF)
			{
				motor_relay_apply_hw(motor_relay_target);
				motor_relay_applied = motor_relay_target;
				motor_relay_state = MOTOR_RELAY_STATE_SETTLE;
				motor_relay_timer_ms = MOTOR_RELAY_SETTLE_MS;
			}
		}
	}

	if ((motor_relay_state == MOTOR_RELAY_STATE_STABLE) && (motor_relay_applied == motor_relay_target))
		allow_relay = (motor_relay_applied != MOTOR_RELAY_CMD_OFF);
}

static void motor_relay_service_1ms(void)
{
	if ((motor_relay_state == MOTOR_RELAY_STATE_STABLE) && (motor_relay_applied != motor_relay_target))
	{
		HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
		allow_relay = 0;
		relaytmr = 0;

		if (motor_relay_applied != MOTOR_RELAY_CMD_OFF)
		{
			motor_relay_state = MOTOR_RELAY_STATE_HOLD_CURRENT;
			motor_relay_timer_ms = MOTOR_RELAY_HOLD_OFF_MS;
		}
		else if (motor_relay_target != MOTOR_RELAY_CMD_OFF)
		{
			motor_relay_apply_hw(motor_relay_target);
			motor_relay_applied = motor_relay_target;
			motor_relay_state = MOTOR_RELAY_STATE_SETTLE;
			motor_relay_timer_ms = MOTOR_RELAY_SETTLE_MS;
		}
	}

	switch (motor_relay_state)
	{
	case MOTOR_RELAY_STATE_HOLD_CURRENT:
		if (motor_relay_timer_ms > 0)
			motor_relay_timer_ms--;
		if (motor_relay_timer_ms == 0)
		{
			if (motor_relay_target == motor_relay_applied)
			{
				motor_relay_state = MOTOR_RELAY_STATE_SETTLE;
				motor_relay_timer_ms = MOTOR_RELAY_SETTLE_MS;
			}
			else
			{
				motor_relay_apply_hw(MOTOR_RELAY_CMD_OFF);
				motor_relay_applied = MOTOR_RELAY_CMD_OFF;
				if (motor_relay_target == MOTOR_RELAY_CMD_OFF)
				{
					motor_relay_state = MOTOR_RELAY_STATE_STABLE;
				}
				else
				{
					motor_relay_state = MOTOR_RELAY_STATE_DEADTIME;
					motor_relay_timer_ms = MOTOR_RELAY_DEADTIME_MS;
				}
			}
		}
		break;

	case MOTOR_RELAY_STATE_DEADTIME:
		if (motor_relay_timer_ms > 0)
			motor_relay_timer_ms--;
		if (motor_relay_timer_ms == 0)
		{
			if (motor_relay_target == MOTOR_RELAY_CMD_OFF)
			{
				motor_relay_state = MOTOR_RELAY_STATE_STABLE;
			}
			else
			{
				motor_relay_apply_hw(motor_relay_target);
				motor_relay_applied = motor_relay_target;
				motor_relay_state = MOTOR_RELAY_STATE_SETTLE;
				motor_relay_timer_ms = MOTOR_RELAY_SETTLE_MS;
			}
		}
		break;

	case MOTOR_RELAY_STATE_SETTLE:
		if (motor_relay_target != motor_relay_applied)
		{
			HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
			allow_relay = 0;
			relaytmr = 0;
			motor_relay_state = MOTOR_RELAY_STATE_HOLD_CURRENT;
			motor_relay_timer_ms = MOTOR_RELAY_HOLD_OFF_MS;
			break;
		}

		if (motor_relay_timer_ms > 0)
			motor_relay_timer_ms--;
		if (motor_relay_timer_ms == 0)
		{
			motor_relay_state = MOTOR_RELAY_STATE_STABLE;
			allow_relay = (motor_relay_applied != MOTOR_RELAY_CMD_OFF);
		}
		break;

	default:
		if (motor_relay_applied == motor_relay_target)
			allow_relay = (motor_relay_applied != MOTOR_RELAY_CMD_OFF);
		break;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	stop_func();
	EraseVar_func();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	motor_relay_force_reset();
	master_relay_force_reset();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 2);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_SET);
	for (int p = 0; p < 100; p++)
	{
		HAL_Delay(30);
		HAL_IWDG_Refresh(&hiwdg);
	}

	socket_init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	Dlmn = 0;
	Glmn = 0;
	Hlmn = 0;
	Klmn = 200;
	Llmn = 200;
	Mlmn = 600;
	Nlmn = 0;
	Olmn = 0;
	Plmn = 30;
	Tlmn = 0;
	Ulmn = 0;
	Vlmn = 0;
	Wlmn = 0;
	Ylmn = 0;
	Zlmn = 0;
	HAL_IWDG_Refresh(&hiwdg);
	master_relay_request(0);
	onwm_flag = 0;
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(200);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

	fto_flag = 1;
	while (!verify_flag)
	{
		socket_init();
		verify_cnt++;
		if (2 < verify_cnt)
		{
			verify_flag = 1;
			norx_flag = 1;
		}
		socket_send("&*r#\r\n");
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
	}
	if (norx_flag)
	{
		HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_SET);
		norx_flag = 0;
		verify_flag = 0;
		while (!verify_flag)
		{
			socket_init();
			verify_cnt++;
			if (30 < verify_cnt)
			{
				verify_flag = 1;
				norx_flag = 1;
			}
			socket_send("&*r#\r\n");
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
		}
	}
	if (norx_flag)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	}
	if (Hlmn == 0)
	{
		socket_send("*#XXendXX@$\r\n");
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		socket_send("*#XXendXX@$\r\n");
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		socket_send("*#XXendXX@$\r\n");
	}
	fto_flag = 0;
	if (Dlmn == 1)
		hwater_flag=1;
	if (Flmn == 1)
		nosensor_flag = 1;
	if (Glmn != 64)
		E41_flag = 1;
	motor_speed_start = Mlmn;
	motor_speed = Mlmn;
	timer_show = 0;

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);

	socket_init();
	Rif = Nlmn;
	Beta = Olmn;
	if (Rif > 100 && Rif < 30000 && Beta > 2900 && Beta < 5000)
	{
		ee30_flag = 1;
		ee3s_flag = 1;
		// if (!nosensor_flag) E30_flag=1;
	}

	bufftest4[0] = '+';
	bufftest4[1] = '=';
	bufftest4[2] = 'P';
	bufftest4[3] = 'S';
	bufftest4[4] = '0';
	bufftest4[5] = '^';
	bufftest4[6] = '\r';
	bufftest4[7] = '\n';
	adc_rd2 = 0;

	hydroempty = Ilmn - 25;
	hydrofull = Jlmn;
	hydroempty2 = hydroempty;
	hydsum[0] = 6750;
	hydsum[1] = 6750;
	hydsum[2] = 6750;
	hydsum[3] = 6750;
	hydsum[4] = 6750;
	bufftest2[0] = '0';
	if (!hydro_flag)
	{
		bufftest2[1] = '3';
	}
	else
	{
		bufftest2[1] = 'x';
	}
	if (swdr_flag)
	{
		bufftest2[2] = 'v';
	}
	else
	{
		bufftest2[2] = '5';
	}
	bufftest2[3] = '7';
	bufftest2[4] = 'a';
	bufftest2[5] = 'c';
	bufftest2[6] = 'e';
	bufftest2[7] = 'g';
	bufftest2[8] = 'i';

	if ((hydrofull > 5000) && (hydrofull < 6500) && (hydroempty > 5000) && (hydroempty < 6500) && (hydroempty > hydrofull))
	{
		l1ok_flag = 1;
		l2ok_flag = 1;
	}

	plm = 30;
	plm4 = 26;
	plm = Plmn;
	if ((plm < 20) || (plm > 50))
		plm = 30;
	plm3 = (6 * plm) / 8;
	maxtaco = plm3 + 1;
	mintaco = plm3;

	if (Ylmn == 93)
		hydro_flag = 1;
	if (Zlmn == 98)
	{
		swdr_flag = 1;
		OpRy_flag = 0;
	}
	else if (Zlmn == 95)
	{
		swdr_flag = 1;
		OpRy_flag = 1;
	}
	bufftest2[0] = '0';
	if (!hydro_flag)
	{
		bufftest2[1] = '3';
	}
	else
	{
		bufftest2[1] = 'x';
	}
	if (swdr_flag)
	{
		bufftest2[2] = 'v';
	}
	else
	{
		bufftest2[2] = '5';
	}
	bufftest2[3] = '7';
	bufftest2[4] = 'a';
	bufftest2[5] = 'c';
	bufftest2[6] = 'e';
	bufftest2[7] = 'g';
	bufftest2[8] = 'i';
	if (Hlmn != 0)
	{
		HHH = Hlmn;
		HHl[0] = 0;
		HHl[1] = 0;
		HHl[2] = 0;
		HHl[3] = 0;
		HHl[4] = 0;
		ww = Hlmn / 10;
		HHl[0] = Hlmn - (ww * 10);
		ss = ww / 10;
		HHl[1] = ww - (ss * 10);
		ww = ss / 10;
		HHl[2] = ss - (ww * 10);
		ss = ww / 10;
		HHl[3] = ww - (ss * 10);
		HHl[4] = ss;

		ww = HHl[3];
		tt = HHl[2];
		if (tt > 4)
			tt = tt - 5;
		pp = 0;
		procng();
		run_flag = 1;

		master_relay_request(1);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(100);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		if (swdr_flag)
		{
			if (!doorlock_flag)
			{
				if ((swdr_flag) && (!Error_flag))
					master_relay_request(1);
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
				HAL_Delay(20);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			}
			while ((!doorlock_flag) && (offcnt < 1000))
			{
				if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
					doorlock_cnt = 0;
				doorlock_cnt++;
				if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
				{
					doorlock_flag = 1;
					doorlock_cnt = 0;
				}
				else
				{
					doorlock_cnt++;
					if (doorlock_cnt > 200)
						doorlock_flag = 0;
				}
				offcnt++;
				HAL_Delay(1);
				HAL_IWDG_Refresh(&hiwdg);
			}
			if (!doorlock_flag)
			{
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
				HAL_Delay(20);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			}
			if (!doorlock_flag)
			{
				if ((swdr_flag) && (!Error_flag))
					master_relay_request(1);
				HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
				HAL_Delay(20);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
				offcnt = 0;
				doorlock_cnt = 0;
				while ((!doorlock_flag) && (offcnt < 1000))
				{
					if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
						doorlock_cnt = 0;
					doorlock_cnt++;
					if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
					{
						doorlock_flag = 1;
						doorlock_cnt = 0;
					}
					else
					{
						doorlock_cnt++;
						if (doorlock_cnt > 200)
							doorlock_flag = 0;
					}
					offcnt++;
					HAL_Delay(1);
					HAL_IWDG_Refresh(&hiwdg);
				}
				if (!doorlock_flag)
				{
					HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
					HAL_Delay(20);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
				}
				while ((!doorlock_flag) && (offcnt < 1000))
				{
					if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
						doorlock_cnt = 0;
					HAL_IWDG_Refresh(&hiwdg);
					doorlock_cnt++;
					if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
					{
						doorlock_flag = 1;
						doorlock_cnt = 0;
					}
					else
					{
						doorlock_cnt++;
						if (doorlock_cnt > 200)
							doorlock_flag = 0;
					}
					offcnt++;
				}
			}
		}
		else
		{
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
			doorlock_cnt = 0;
			doorlock_flag = 0;
			while ((!doorlock_flag) && (offcnt < 5000))
			{
				if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
					doorlock_cnt = 0;
				doorlock_cnt++;
				if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
				{
					doorlock_flag = 1;
					doorlock_cnt = 0;
				}
				else
				{
					doorlock_cnt++;
					if (doorlock_cnt > 200)
						doorlock_flag = 0;
				}
				offcnt++;
				HAL_Delay(1);
				HAL_IWDG_Refresh(&hiwdg);
			}
		}
		washmode = 0;
		prewashmode = 0;
		if (HHl[1] > 4)
		{
			prewashmode = 1;
			prewashing_time = 26;
			HHl[1] = HHl[1] - 5;
		}
		rinsemode = HHl[1];
		program_select = HHl[0];
		if (HHl[2] > 4)
		{
			washmode = 1;
			HHl[2] = HHl[2] - 5;
		}
		wtme = prewashmode + washmode + rinsemode;
		dtme = prewashmode + washmode + rinsemode;
		if (ww < 8)
			stme = (prewashmode + washmode + rinsemode - 1) * 2;
		if (ww == 9)
		{
			stme = 0;
			spintime = 1;
		}
		rtme = rinsemode * 3;
		onwm_flag = 1;
	}
	first_turn_motor = 0;

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if ((Glmn == 64) && (hydro_flag) && (hydro2 < (hydroempty + hydrofull * 3) / 4))
		{
			DnOpDr_flag = 1;
		}
		else
		{
			DnOpDr_flag = 0;
		}
		HAL_IWDG_Refresh(&hiwdg);
		if (onwm_flag)
		{
			if ((swdr_flag) || (program_select > 0) || (Debug_flag) || (autodebug_flag) || (DnOpDr_flag) || (E11_flag))
			{
				master_relay_request(1);
			}
			else
			{
				master_relay_request(0);
			}
			if (!Debug_flag)
			{
				if (autodebug_flag)
				{
					if (onems_flag)
					{
						HAL_IWDG_Refresh(&hiwdg);
						if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
						{
							doorlock_flag = 1;
							doorlock_cnt = 0;
						}
						else
						{
							doorlock_cnt++;
							if (doorlock_cnt > 200)
								doorlock_flag = 0;
						}
						if (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin))
						{
							wtme++;
							if (wtme > 200)
								BeforeSleep();
						}
						else
						{
							wtme = 0;
							if (ms250_flag)
								autodebug();
						}
						onems_flag = 0;
					}
				}
				else
				{
					if (Glmn != 64)
						E41_flag = 1;
					if (lockdoor_flag)
						lockdoor_func();
					if (onems_flag)
						onems_func();
					if (ms250_flag)
						ms250_func();
					if (ms1000_flag)
						ms1000_func();
					if (key_flag)
						key_func();
					if (stop_flag)
						stop_func();
					if ((mission_flag) && (run_flag))
						mission_func();
					if ((washing_flag) && (run_flag))
						washing_func();
					if ((program_select != 0) && (pause_flag))
						pause_func();
					if ((program_select != 0) && (resume_flag))
						resume_func();
					if ((program_select != 0) && (spchg_flag))
						spchg_func();
					if ((procng_flag) && (program_select < 3))
					{
						if (reg_flag)
						{
							procng();
						}
						else
						{
							socket_send("*#XXunregXX@$\r\n");
							procng_flag = 0;
						}
					}
					else
					{
						procng_flag = 0;
					}
				}
			}
		}
		else
		{
			if (onems_flag)
			{
				HAL_IWDG_Refresh(&hiwdg);
				stop_func();
				EraseVar_func();
				stayoff = 1;
				end_flag = 0;
				E5lchk_flag = 0;
				if ((swdr_flag) || (!DnOpDr_flag))
					master_relay_request(0);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				motor_relay_request(MOTOR_RELAY_CMD_OFF);
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				if ((swdr_flag) || (!DnOpDr_flag))
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
				while (stayoff)
				{
					stoffcnt = 0;
					HAL_Delay(50);
					HAL_IWDG_Refresh(&hiwdg);
					while ((stayoff) && (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin)))
					{
						stoffcnt++;
						if (stoffcnt > 20)
							stayoff = 0;
						HAL_Delay(20);
						HAL_IWDG_Refresh(&hiwdg);
					}
				}
				Debug_flag = 0;
				HAL_Delay(50);
				HAL_IWDG_Refresh(&hiwdg);
				if (!HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin))
				{
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
					socket_send("*#XXdebug@$\r\n");
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
					socket_send("*#XXdebug@$\r\n");
					Debug_flag = 1;
				}
				else
				{
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(ESPRST_GPIO_Port, ESPRST_Pin, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				}
				if (swdr_flag)
					master_relay_request(1);
				while (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin))
				{
					HAL_Delay(50);
					HAL_IWDG_Refresh(&hiwdg);
				}
				if (!norx_flag)
					onwm_flag = 1;
				if (Debug_flag)
				{
					Debug_func();
				}
				else
				{
					numi = wprg;
					NumToStr();
					bufftest4[6] = '\r';
					bufftest4[7] = '\n';
					socket_send(bufftest4);
				}
				onems_flag = 0;
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 620;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 153;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Motor_Ui1_GPIO_Port, Motor_Ui1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Motor_Pin | ESPRST_Pin | LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Buzzer_Pin | Door_Open_Pin | Door_Lock_Pin | Drain_Pomp_Pin | Valve3_Pin | Valve2_Pin | Valve1_Pin | Heater_Pin | MasterRLY_Pin | Motor_Ui2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Motor_Ui1_Pin */
	GPIO_InitStruct.Pin = Motor_Ui1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Motor_Ui1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : pushonkey_Pin Freq_in_Pin Taco_PLS_Pin Hydro_Pin */
	GPIO_InitStruct.Pin = pushonkey_Pin | Freq_in_Pin | Taco_PLS_Pin | Hydro_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Motor_Pin */
	GPIO_InitStruct.Pin = Motor_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(Motor_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Buzzer_Pin Door_Open_Pin Door_Lock_Pin Drain_Pomp_Pin
							 Valve3_Pin Valve2_Pin Valve1_Pin Heater_Pin
							 MasterRLY_Pin Motor_Ui2_Pin */
	GPIO_InitStruct.Pin = Buzzer_Pin | Door_Open_Pin | Door_Lock_Pin | Drain_Pomp_Pin | Valve3_Pin | Valve2_Pin | Valve1_Pin | Heater_Pin | MasterRLY_Pin | Motor_Ui2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : ESPRST_Pin LED_Pin */
	GPIO_InitStruct.Pin = ESPRST_Pin | LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : key1_Pin */
	GPIO_InitStruct.Pin = key1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(key1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
/******************************************************************************/
void key_func()
{
	if ((!ChLk_flag) && (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin)) && (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin)))
	{
		yz++;
		if (yz > 1000)
		{
			end_flag = 1;
			stop_func();
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			if (swdr_flag)
				master_relay_request(1);
			if ((swdr_flag) && (!DnOpDr_flag))
			{
				if (doorlock_flag)
				{
					master_relay_request(1);
					if (OpRy_flag)
					{
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
					}
					else
					{
						HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
					}
					HAL_Delay(20);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
					if (OpRy_flag)
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
				}
				offcnt = 0;
				doorlock_cnt = 0;
				while ((doorlock_flag) && (offcnt < 1000))
				{
					if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
						doorlock_cnt = 0;
					doorlock_cnt++;
					if (doorlock_cnt > 200)
						doorlock_flag = 0;
					offcnt++;
					HAL_Delay(1);
					HAL_IWDG_Refresh(&hiwdg);
				}
				if (doorlock_flag)
				{

					master_relay_request(1);
					if (OpRy_flag)
					{
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
					}
					else
					{
						HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
					}
					HAL_Delay(20);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
					if (OpRy_flag)
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
				}
			}
			stop_func();
			while (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin))
			{
				HAL_Delay(50);
				HAL_IWDG_Refresh(&hiwdg);
			}
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(100);
			HAL_IWDG_Refresh(&hiwdg);
			socket_send("*#XXendXX@$\r\n");
			if (!DnOpDr_flag)
				master_relay_request(0);
			onwm_flag = 0;
		}
	}
	else
	{
		yz = 0;
	}
	if (!HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin))
	{
		if ((E30_flag) || (E31_flag) || (E32_flag) || (E34_flag) || (E39_flag))
		{
			E3c_flag = 1;
			E30_flag = 0;
			E31_flag = 0;
			E32_flag = 0;
			E34_flag = 0;
			E39_flag = 0;
			yw = 0;
		}
		else if ((!Error_flag) && (reg_flag) && (!ChLk_flag) && (!plkey_flag))
		{
			yw++;
		}
		else
		{
			yw = 0;
		}
		if (yw > 50)
		{
			yw = 0;
			plkey_flag = 1;
			if (program_select == 0)
			{
				buzzercnt = 2;
			}
		}
		if ((!Error_flag) && (reg_flag) && (!ChLk_flag) && (program_select == 0) && (!plkey_flag2))
		{
			yw2++;
		}
		else
		{
			yw = 0;
		}
		if (yw2 > 1500)
		{
			yw2 = 0;
			plkey_flag2 = 1;
			if (program_select == 0)
			{
				buzzercnt = 3;
			}
		}
		if ((program_select > 0) && (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin)))
		{
			yx++;
			if (yx > 3000)
			{
				plkey_flag = 0;
				buzzercnt = 4;
				ChLk_flag = !ChLk_flag;
				push_flag = 1;
				yx = 0;
			}
		}
		else if ((program_select > 0) && (!Error_flag) && (reg_flag) && (!push_flag) && (!ChLk_flag))
		{
			push_flag = 1;
			if (run_flag)
			{
				buzzercnt = 2;
				pause_cnt = 3;
				pause_flag = 1;
			}
			else
			{
				buzzercnt = 3;
				resume_flag = 1;
			}
		}
	}
	else
	{
		yw = 0;
		yw2 = 0;
		yx = 0;
		if ((!ChLk_flag) && (plkey_flag) && (!Debug_flag))
		{
			if ((program_select == 0) && (!delay_start_flag))
			{
				if (plkey_flag2)
				{
					progs_flag = 1;
					socket_send("+=PSR^\r\n");
					delay_timer = 0;
					ww = wprg;
					ss = 2;
					if (ww == 3)
						ss = 1;
					if (ww == 4)
						ss = 0;
					if (ww == 9)
						ss = 5;

					tt = 0;
					if ((ww == 0) || (ww == 4))
						tt = 1;
					if ((ww == 2) || (ww == 3) || (ww == 5))
						tt = 2;
					if ((ww == 1) || (ww == 6))
						tt = 3;

					if (ww == 1)
						pp = 1;
					if ((ww == 5) || (ww == 6))
						pp = 2;
					woolmode = 0;
					if (ww == 4)
						woolmode = 1;
					delay_start_flag = 0;
					procng_flag = 1;
				}
				else
				{
					wprg++;
					if (wprg > 9)
						wprg = 0;
					numi = wprg;
					NumToStr();
					bufftest4[6] = '\r';
					bufftest4[7] = '\n';
					socket_send(bufftest4);
				}
			}
		}
		plkey_flag = 0;
		plkey_flag2 = 0;
		push_flag = 0;
	}
	key_flag = 0;
}

/******************************************************************************/
void onems_func()
{
	HAL_IWDG_Refresh(&hiwdg);
	master_relay_service_1ms();
	motor_relay_service_1ms();
	tm3++;
	key_flag = 1;
	if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
	{
		doorlock_flag = 1;
		doorlock_cnt = 0;
	}
	else
	{
		doorlock_cnt++;
		if (doorlock_cnt > 200)
			doorlock_flag = 0;
	}
	if (buzzercnt > 1)
		tm4++;
	if ((tm4 > 100) && (buzzercnt > 1))
	{
		if (buzz)
		{
			buzz = 0;
			buzzercnt--;
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			buzz = 1;
		}
		tm4 = 0;
	}
	if (tm3 > 1000)
	{
		if ((reg_flag) && (!Error_flag) && (program_select == 0) && (!delay_start_flag) && (pscnt > 5) && (!Debug_flag))
		{
			bufftest4[6] = '\r';
			bufftest4[7] = '\n';
			socket_send(bufftest4);
			pscnt = 0;
		}
		if (ready_scnt > 1)
		{
			pscnt = 0;
			if ((program_select != 7) || (!end_flag))
			{
				if (reg_flag)
				{
					socket_send("*#XXreadyXX@$\r\n");
				}
				else
				{
					socket_send("*#XXunregXX@$\r\n");
				}
			}
			ready_scnt--;
		}
		else if (E00_scnt > 1)
		{
			socket_send("*#XXE00XX@$\r\n");
			E00_scnt--;
		}
		else if (pausing_scnt > 1)
		{
			socket_send("*#XXPauseXX@$\r\n");
			pausing_scnt--;
		}
		else if (washing_scnt > 1)
		{
			socket_send("*#XXwashXX@$\r\n");
			washing_scnt--;
		}
		else if (rinsing_scnt > 1)
		{
			socket_send("*#XXrinseXX@$\r\n");
			rinsing_scnt--;
		}
		else if (spinning_scnt > 1)
		{
			socket_send("*#XXspinXX@$\r\n");
			spinning_scnt--;
		}
		else if (resume_cnt > 1)
		{
			socket_send("*#XXplayXX@$\r\n");
			resume_cnt--;
		}
		else if (pause_cnt > 1)
		{
			socket_send("*#XXpauseXX@$\r\n");
			pause_cnt--;
		}
		tm3 = 0;
		pscnt++;
	}
	tm5++;
	if (tm5 > 19)
	{
		if (swdr_flag)
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
		if ((swdr_flag) && ((!doorlock_flag) || (!DnOpDr_flag)))
		{
			if (chgdr_flag)
			{
				master_relay_request(1);

				if ((OpRy_flag) && (doorlock_flag))
				{
					HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
				}
				chgdr_flag = 0;
			}
			else
			{
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
				if (OpRy_flag)
					HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
			}
		}
		tm5 = 0;
	}
	if (run_flag)
	{
		if (spin_mode)
		{
			tcpl++;
			if (tcpl > 249)
			{
				tcpl = 0;
				taco_cnt2 = taco_cnt;
				taco_cnt = 0;
			}
			y++; // mrc
			if (spinspeed > plm * 2)
				y++; // mrc
			if ((mission_timer > 140) && (!sosp_flag))
				y += 2;
			if (y > 100) // mrc
			{
				if ((allow_relay) && (taco_cnt2 < 4 * spinspeed) && (run_motor) && (motor_speed < 2000)) //  150 -> 400rpm   300 -> 800rpm   450 -> 1200rpm
				{
					motor_speed++;
				}
				y = 0;
			}
		}
		else
		{
			if (first_turn_motor)
			{
				k++;
				if (program_select != 7)
					k++;
				if (k > 20) //  ramp incr times (ms)    vvvv
				{
					if ((allow_relay) && ((left_flag)) && (run_motor))
					{
						if (motor_speed_slctL > motor_speed)
							motor_speed++;
						if (motor_speed_slctL < motor_speed)
							motor_speed--;
					}
					else if ((allow_relay) && (right_flag) && (run_motor))
					{
						if (motor_speed_slctR > motor_speed)
							motor_speed++;
						if (motor_speed_slctR < motor_speed)
							motor_speed--;
					}
					k = 0;
				}
			}
			else
			{
				k2 += 2;
				if (program_select != 7)
					k2++;
				if (k2 > 150)
				{
					if ((allow_relay) && (run_motor) && (motor_speed < 2000))
						motor_speed++;
					k2 = 0;
				}
			}
		}
	}
	onems_flag = 0;
}

/******************************************************************************/
void ms250_func()
{
	hydpin_flag = !HAL_GPIO_ReadPin(Hydro_GPIO_Port, Hydro_Pin);
	if (!hydro_flag)
	{
		if (hydpin_flag)
		{
			hydpin2_flag = 1;
			hydpin_cnt = 0;
		}
		else
		{
			hydpin_cnt++;
			if (hydpin_cnt > 20)
				hydpin2_flag = 0;
		}
	}
	if (run_motor)
	{
		spchcnt++;
		if (runmotorc_flag)
			runmotorcnt++;
		if (runmotorcnt > 7)
		{
			first_turn_motor = 0;
			runmotorc_flag = 0;
			runmotorcnt = 0;
		}
	}
	else
	{
		runmotorc_flag = 1;
		runmotorcnt = 0;
	}
	spchg_flag = 1;
	spchcnt = 0;
	if ((run_flag) && (program_select > 1) && (!doorlock_flag) && ((mission_timer < 420) || (program_select != 7)))
	{
		LkDrEr++;
		if (LkDrEr > 5)
		{
			stop_func();
			E11_flag = 1;
			mission_timer = 0;
		}
	}
	else
	{
		LkDrEr = 0;
	}
	if (hydpin_flag)
	{
		wef_flag = 1;
		wefcnt = 0;
	}
	if ((!hydpin_flag) && ((program_select == 5) || (program_select == 2)))
		wefcnt++;
	if (wefcnt > 520)
		wef_flag = 0;
	if ((swdr_flag) && (!Error_flag))
		master_relay_request(1);
	tacosafe = 0;
	if (((hydro_flag) && ((hydrofull + hydroempty) < hydro2 * 2)) && (program_select == 4) && (!run_motor) && (run_flag))
	{
		tct++;
		if (tct > 15)
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
	}
	else
	{
		tct = 0;
	}
	j++;
	if (j > 12)
	{
		if ((Error_flag) || (E30_flag) || (E31_flag) || (E32_flag) || (E34_flag) || (E39_flag))
		{
			if (doorlock_flag)
				E11_flag = 0;
			buzzercnt = 5;
			if (E01_flag)
			{
				socket_send("*#XXE01XX@$\r\n");
			}
			else if (E02_flag)
			{
				socket_send("*#XXE02XX@$\r\n");
			}
			else if (E11_flag)
			{
				socket_send("*#XE11XE11@$\r\n");
			}
			else if (E21_flag)
			{
				socket_send("*#XE21XE21@$\r\n");
			}
			else if (E22_flag)
			{
				socket_send("*#XE22XE22@$\r\n");
			}
			else if (E23_flag)
			{
				socket_send("*#XE23XE23@$\r\n");
			}
			else if (E24_flag)
			{
				socket_send("*#XE24XE24@$\r\n");
			}
			else if (E30_flag)
			{
				socket_send("*#XE30XE30@$\r\n");
			}
			else if (E31_flag)
			{
				socket_send("*#XE31XE31@$\r\n");
			}
			else if (E32_flag)
			{
				socket_send("*#XE32XE32@$\r\n");
			}
			else if (E33_flag)
			{
				socket_send("*#XE33XE33@$\r\n");
			}
			else if (E34_flag)
			{
				socket_send("*#XE34XE34@$\r\n");
			}
			else if (E39_flag)
			{
				socket_send("*#XE34XE39@$\r\n");
			}
			else if (E41_flag)
			{
				socket_send("*#XE41XE41@$\r\n");
			}
			else if (E51_flag)
			{
				socket_send("*#XE51XE51@$\r\n");
			}
			else if (E61_flag)
			{
				socket_send("*#XE61XE61@$\r\n");
			}
			else if (E62_flag)
			{
				socket_send("*#XE62XE62@$\r\n");
			}
			else if (E63_flag)
			{
				socket_send("*#XE63XE63@$\r\n");
			}
		}
		else
		{
			if ((program_select > 0) || (delay_start_flag))
			{
				//				adc_rd=4096-adc_rd2;
				//				adc_rd=10000*adc_rd2/adc_rd;
				if ((nospin_flag) || (ss == 5))
					stme = 0;
				timer_show = wtme + dtme + stme + rtme + (prewashing_time * prewashmode) + (washing_time * washmode) + spintime;
				if ((timer_show == 0) && (doorlock_flag))
					timer_show = 1;
				if (reg_flag)
					sprintf(bufftest, "*#W%2dT%3dD%4dXXXXXXXXXXXCXDX,%fR%dB%d,%d@$\r\n", ww, timer_show, delay_timer, T, Rif, Beta, mission_timer);
				if (((program_select == 7) || (ww == 8)) && (run_flag))
				{
					bufftest[14] = 's';
					bufftest[15] = 'p';
					bufftest[16] = 'i';
					bufftest[17] = 'n';
				}
				else if ((prewashmode) && (run_flag))
				{
					bufftest[14] = 'w';
					bufftest[15] = 'a';
					bufftest[16] = 's';
					bufftest[17] = 'h';
					bufftest[18] = '1';
				}
				else if ((washmode) && (run_flag))
				{
					bufftest[14] = 'w';
					bufftest[15] = 'a';
					bufftest[16] = 's';
					bufftest[17] = 'h';
					bufftest[18] = '2';
				}
				else if (run_flag)
				{
					bufftest[14] = 'r';
					bufftest[15] = 'i';
					bufftest[16] = 'n';
					bufftest[17] = 's';
					bufftest[18] = 'e';
				}
				if (ww < 10)
					bufftest[3] = '0';
				if (timer_show < 10)
				{
					bufftest[6] = '0';
					bufftest[7] = '0';
				}
				else if (timer_show < 100)
				{
					bufftest[6] = '0';
				}
				if (delay_timer < 10)
				{
					bufftest[10] = '0';
					bufftest[11] = '0';
					bufftest[12] = '0';
				}
				else if (delay_timer < 100)
				{
					bufftest[10] = '0';
					bufftest[11] = '0';
				}
				else if (delay_timer < 1000)
				{
					bufftest[10] = '0';
				}
				if (run_flag)
				{
					bufftest[19] = 'p';
					bufftest[20] = 'l';
					bufftest[21] = 'a';
					bufftest[22] = 'y';
					bufftest[23] = 'X';
				}
				else
				{
					bufftest[19] = 'p';
					bufftest[20] = 'a';
					bufftest[21] = 'u';
					bufftest[22] = 's';
					bufftest[23] = 'e';
				}
				if (ChLk_flag)
				{
					bufftest[26] = 'L';
				}
				else
				{
					bufftest[26] = 'O';
				}
				if (doorlock_flag)
				{
					bufftest[28] = 'L';
				}
				else
				{
					bufftest[28] = 'O';
				}
				socket_send(bufftest);
			}
		}
		j = 0;
	}
	else if (j > 11)
	{
		socket_init();
	}
	else if (j > 10)
	{
		if (eesaves_flag)
			eesaves_func();
	}
	if ((j < 11) && (j > 2))
	{
		if ((!left_flag) && (!right_flag))
		{
			if (!run_motor)
			{
				if (hydroread_flag)
				{
					if ((hydro_flag) && ((hyd_cnt > 7500) || (hyd_cnt < 3750)))
					{
						if (HydEr_timer < 11)
							HydEr_timer++;
						if (HydEr_timer > 5)
						{
							if (hyd_cnt < 3750)
							{
								E01_flag = 1;
							}
							else
							{
								E02_flag = 1;
							}
						}
					}
					else
					{
						HydEr_timer = 0;
						hydsum[0] = hydsum[1];
						hydsum[1] = hydsum[2];
						if (hyd_cnt > 4000)
							hydsum[2] = hyd_cnt;
						hydro2 = (hydsum[0] + hydsum[1] + hydsum[2]) / 3;
						E01_flag = 0;
						E02_flag = 0;
					}
				}
				hydroread_flag = 1;
			}
			else
			{
				hydroread_flag = 0;
			}
		}
	}
	hyd_cnt = 0;
	ms250_flag = 0;
}

/******************************************************************************/
void ms1000_func()
{
	//	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	if (nosensor_flag)
	{
		coldwash_flag = 1;
	}
	else
	{
		if ((adc_buf[1] > 200) && (adc_buf[1] < 3900))
		{
			if (firstadc_flag)
			{
				adc_rd2 = adc_buf[1];
				adcsum[0] = adc_buf[1];
				adcsum[1] = adc_buf[1];
				adcsum[2] = adc_buf[1];
				adcsum[3] = adc_buf[1];
				adcsum[4] = adc_buf[1];
				firstadc_flag = 0;
			}
			else
			{
				adcsum[0] = adcsum[1];
				adcsum[1] = adcsum[2];
				adcsum[2] = adcsum[3];
				adcsum[3] = adcsum[4];
				adcsum[4] = adc_buf[1];
				adc_rd2 = adcsum[0] + adcsum[1] + adcsum[2] + adcsum[3] + adcsum[4];
				adc_rd2 = adc_rd2 / 5;
			}
			E34_cnt = 0;
			calT();
		}
		else
		{
			if ((!coldwash_flag) && (program_select > 0))
				E34_cnt++;
			if ((!E3c_flag) && (E34_cnt > 55))
			{
				if (adc_buf[1] < 201)
				{
					E35_flag = 1;
				}
				else
				{
					E34_flag = 1;
				}
			}
		}
	}
	if ((!delay_start_flag) && (program_select == 0))
	{
		stop_off_timer++;
	}
	else
	{
		stop_off_timer = 0;
	}
	if (stop_off_timer > 300)
		BeforeSleep();
	if (delay_start_flag)
	{
		min_cnt++;
		if (min_cnt > 55)
		{
			if (delay_timer > 0)
				delay_timer--;
			min_cnt = 0;
		}
		if (delay_timer == 0)
		{
			program_select = 1;
			delay_start_flag = 0;
			run_flag = 1;
		}
	}
	if ((program_select > 0) && ((mission_timer < 420) || (program_select != 7)))
	{
		if (swdr_flag)
		{
			if (!doorlock_flag)
			{
				chgdr_cnt++;
				if (chgdr_cnt > 11)
				{
					chgdr_flag = 1;
					chgdr_cnt = 0;
				}
			}
			else
			{
				chgdr_cnt = 11;
			}
		}
		else
		{
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
		}
	}
	else
	{
		if ((swdr_flag) && (!DnOpDr_flag))
		{
			if (doorlock_flag)
			{
				chgdr_cnt++;
				if (chgdr_cnt > 11)
				{
					chgdr_flag = 1;
					chgdr_cnt = 0;
				}
			}
			else
			{
				chgdr_cnt = 11;
			}
		}
		else
		{
			if (!DnOpDr_flag)
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
		}
	}
	if ((reg_flag) && ((E30_flag) || (E31_flag) || (E32_flag) || (E34_flag) || (E39_flag)))
		E3c_flag = 1;
	if ((reg_flag) && ((E01_flag) || (E02_flag) || (E11_flag) || (E21_flag) || (E22_flag) || (E23_flag) || (E24_flag) || (E33_flag) || (E41_flag) || (E51_flag) || (E61_flag) || (E62_flag) || (E63_flag)))
	{
		Error_timer++;
		if (!Error_flag)
		{
			if (run_flag)
				ressa = 1;
			if ((E23_flag) || (E24_flag) || (E33_flag) || (E41_flag) || (E51_flag) || (E61_flag) || (E62_flag) || (E63_flag))
			{
				if (run_flag)
					stop_flag = 1;
			}
			else
			{
				if (run_flag)
					pause_flag = 1;
			}
		}
		Error_flag = 1;
		if (Error_timer > 9)
		{
			Error_sflag = 1;
			Error_timer = 0;
		}
		ready_scnt = 1;
		pausing_scnt = 1;
		washing_scnt = 1;
		rinsing_scnt = 1;
		spinning_scnt = 1;
		pause_cnt = 1;
		resume_cnt = 1;
		unreg_scnt = 1;
	}
	else
	{
		Error_timer = 0;
		if (Error_flag)
		{
			E00_scnt = 4;
			if (ressa)
				resume_flag = 1;
			Error_flag = 0;
			Error_timer = 0;
		}
	}
	if (run_flag)
	{
		_Bool wash_drive_active = washst_flag && run_motor && allow_relay && doorlock_flag &&
								  (motor_relay_applied != MOTOR_RELAY_CMD_OFF);

		if (washst_flag)
			washing_cnt++;
		mission_timer++;
		mission_flag = 1;

		/* In wash stage, raise E51 if tacho feedback is missing for 5 seconds while motor drive is active. */
		if (wash_drive_active)
		{
			if (turn_motor)
			{
				turn_motor = 0;
				taco_stop_tmr = 0;
				E51_cnt = 0;
			}
			else
			{
				if (taco_stop_tmr < 255)
					taco_stop_tmr++;
				E51_cnt = taco_stop_tmr;
				if (taco_stop_tmr >= 5)
					E51_flag = 1;
			}
		}
		else
		{
			turn_motor = 0;
			taco_stop_tmr = 0;
			E51_cnt = 0;
		}
		if (washing_cnt > washturntime * 2 + washresttime * 2 - 1)
		{
			left_flag = 0;
			right_flag = 0;
			twiceturn_flag = 1;
			washing_cnt = 0;
		}
	}
	ms1000_flag = 0;
}

/******************************************************************************/
void washing_func()
{
	if (washing_cnt > washresttime * 2 + washturntime - 1)
	{
		run_motor = 1;
		if ((washing_cnt == washturntime * 2 + washresttime * 2 - 1) && ((!first_grab) || (!fourturn_flag)))
		{
			tol++;
			if ((tol < 13) || (!tccnt_flag))
				washing_cnt--;
		}
		left_flag = 1;
		right_flag = 0;
		rest_tmr = 0;
		cc++;
	}
	else if (washing_cnt > washresttime * 2 + washturntime - 2)
	{
		if (first_grab)
		{
			motor_speed = motor_speed_start;
			if ((program_select == 7) && (motor_speed > 100))
				motor_speed -= 100;
		}

		motor_relay_request(MOTOR_RELAY_CMD_UI1);

		run_motor = 0;
		turn_cnt = 0;
		fourturn_flag = 0;
		fourturn_cnt = 0;
		left_flag = 1;
		rest_tmr = 0;
		right_flag = 0;
		if (allow_relay)
			E5lchk_flag = 1;
		cc = 0;
		tol = 0;
	}
	else if (washing_cnt > washturntime + washresttime)
	{
		run_motor = 0;

		motor_relay_request(MOTOR_RELAY_CMD_OFF);
		E5lchk_flag = 0;
		motor_speed = motor_speed_start;
		if ((program_select == 7) && (motor_speed > 100))
			motor_speed -= 100;
		taco_cnt = 0;
		left_flag = 0;
		right_flag = 0;
		rest_tmr++;
		turn_motor = 0;
		tccnt_flag = 0;
		cc = 0;
	}
	else if (washing_cnt > washturntime + washresttime - 1)
	{
		motor_relay_request(MOTOR_RELAY_CMD_OFF);
		E5lchk_flag = 0;
		allow_relay = 0;
		turn_motor = 0;
		left_flag = 0;
		right_flag = 0;
		rest_tmr++;
		cc = 0;
		if (program_select == 3)
		{
			mixok_flag = 1;
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(100);
			HAL_IWDG_Refresh(&hiwdg);
			run_motor = 0;
		}
		if (!first_grab)
			first_turn_motor = 0;
		tol = 0;
	}
	else if (washing_cnt > washresttime - 1)
	{
		run_motor = 1;
		if ((washing_cnt == washturntime + washresttime - 1) && ((!first_grab) || (!fourturn_flag)))
		{
			tol++;
			if ((tol < 13) || (!tccnt_flag))
				washing_cnt--;
		}
		cc++;
		left_flag = 0;
		right_flag = 1;
		rest_tmr = 0;
	}
	else if (washing_cnt > washresttime - 2)
	{
		if (first_grab)
		{
			motor_speed = motor_speed_start;
			if ((program_select == 7) && (motor_speed > 100))
				motor_speed -= 100;
		}
		motor_relay_request(MOTOR_RELAY_CMD_UI2);
		run_motor = 0;
		turn_cnt = 0;
		cc = 0;
		fourturn_flag = 0;
		fourturn_cnt = 0;
		tol = 0;
		left_flag = 0;
		right_flag = 1;
		rest_tmr = 0;
		if (allow_relay)
			E5lchk_flag = 1;
	}
	else if (washing_cnt > 0)
	{
		run_motor = 0;
		motor_relay_request(MOTOR_RELAY_CMD_OFF);
		E5lchk_flag = 0;
		motor_speed = motor_speed_start;
		if ((program_select == 7) && (motor_speed > 100))
			motor_speed -= 100;
		taco_cnt = 0;
		left_flag = 0;
		right_flag = 0;
		turn_motor = 0;
		tccnt_flag = 0;
		rest_tmr++;
		cc = 0;
		if (changetr_flag)
		{
			washresttime = newresttime;
			washturntime = newturntime;
			changetr_flag = 0;
		}
	}
	else
	{
		allow_relay = 0;
		E5lchk_flag = 0;
		motor_relay_request(MOTOR_RELAY_CMD_OFF);
		turn_motor = 0;
		left_flag = 0;
		right_flag = 0;
		if (!first_grab)
			first_turn_motor = 0;
		rest_tmr++;
		cc = 0;
		tol = 0;
	}
	washing_flag = 0;
}

/******************************************************************************/
void mission_func()
{
	if (program_select == 0) // here
	{
		HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
	}
	else
	{
		master_relay_request(1);
		switch (program_select)
		{
		case 1:
			check_E();
			if (mission_timer > 2)
				lockdoor_flag = 1;
			left_flag = 0;
			right_flag = 0;
			progs_flag = 0;
			motor_speed = motor_speed_start;
			first_turn_motor = 0;
			first_grab = 0;
			grab_cnt = 0;
			procng();
			wtme = prewashmode + washmode + rinsemode;
			dtme = prewashmode + washmode + rinsemode;
			if (ww < 8)
				stme = (prewashmode + washmode + rinsemode - 1) * 2;
			rtme = rinsemode * 3;
			if (mission_timer > 19)
			{
				E11_flag = 1;
				mission_timer = 0;
			}
			if (doorlock_flag) // feedback
			{
				if (ww == 8)
				{
					program_select = 5;
				}
				else if ((!woolmode) && (washmode) && (oow_flag) && (hydro2 < hydrofull + (0.15 * (hydroempty - hydrofull))))
				{
					program_select = 4;
					hs = 300;
					if ((!nosensor_flag) && (!coldwash_flag) && (reftemp1 > T) && (washmode) && (((!hydro_flag) && (wef_flag)) || ((hydro_flag) && (hydro2 < (2 * hydrofull + hydroempty) / 3))))
						HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
					washing_cnt = washresttime - 2;
					T_temp = T;
					T_temp2 = T;
					ttemp2_cnt = 0;
					if (wtme > 0)
						wtme--;
				}
				else if ((savewtr_flag) && (ww < 7))
				{
					program_select = 3;
					nodel30_flag = 1;
					changetr_flag = 1;
					washresttime = 6;
					washturntime = 10;
					mission_timer = 22;
				}
				else
				{
#ifdef DRAIN_ENABLED
					program_select = 2;
					nodel30_flag = 0;
					mission_timer = 0;
#else
					program_select = 3;
					nodel30_flag = 1;
					if (ww > 7)
					{
						program_select = 2;
						nodel30_flag = 0;
					}
					mission_timer = 0;
#endif
				}
				washing_scnt = 3;
				resume_cnt = 3;
				eesaves_flag = 1;
				eesc = 0;
				E11_flag = 0;
				right_flag = 0;
				left_flag = 0;
			}
			break;
		case 2: // drain
			check_E();
			HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
			procng();
			wtme = prewashmode + washmode + rinsemode;
			dtme = prewashmode + washmode + rinsemode;
			if (ww < 8)
				stme = (prewashmode + washmode + rinsemode - 1) * 2;
			rtme = rinsemode * 3;
			if ((hydro_flag) && (mission_timer > 120) && (hydro_flag) && (hydrofull + 30 > hydro2))
				E61_flag = 1;
			if (mission_timer > 240)
				E62_flag = 1;
			if ((!hydro_flag) && (!hydpin_flag))
			{
				E61_flag = 0;
			}
			if (((hydro_flag) && (hydro2 > hydroempty - 75)) || ((!wef_flag) && (!hydro_flag)))
			{
				hydro_timer++;
			}
			else
			{
				hydro_timer = 0;
			}
			if (hydro_timer > 4)
			{
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				wef_flag = 0;
				washing_cnt = 0;
				mission_timer = 0;
				mixok_flag = 0;
				first_turn_motor = 0;
				first_grab = 0;
				grab_cnt = 0;
				turn_cnt = 0;
				savewtr_flag = 1;
				program_select = 3;
				left_flag = 0;
				right_flag = 0;
				if (ww == 9)
				{
					program_select = 7;
					first_turn_motor = 0;
					mission_timer = 496;
					if (nospin_flag)
						mission_timer = 500;
					changetr_flag = 1;
					prewashmode = 0;
					washmode = 0;
					rinsemode = 0;
				}
				if (ww == 8)
				{
					mission_timer = 10;
					run_motor = 0;
					turn_motor = 0;
					first_turn_motor = 0;
					first_grab = 0;
					grab_cnt = 0;
					E5lchk_flag = 0;
					spin_mode = 0;
					maxtaco = plm3;
					mintaco = plm3 - 2;
					left_flag = 0;
					right_flag = 0;
					washresttime = 7;
					washturntime = 14;
					cc = 0;
					washing_flag = 0;
					washst_flag = 0;
					washing_cnt = 0;
					program_select = 7;
					if (nospin_flag)
						mission_timer = 500;
					first_grab = 0;
					grab_cnt = 0;
					motor_speed = motor_speed_start;
				}
				eesaves_flag = 1;
				eesc = 4;
			}
			break;
		case 3: // WaterF
			check_E();
			HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
			if ((mission_timer == 6) && (hydro2 > hydroempty))
				hydroempty2 = hydro2;
			oow_flag = 0;
			if (mission_timer > 9)
			{
				if (!washmode)
				{
					left_flag = 0;
					right_flag = 0;
					run_motor = 0;
					if ((rinsemode == 1) && (mission_timer < 30))
						HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_SET);
				}
				else if ((tccnt_flag) && (washst_flag))
				{
					HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
				}
				else
				{
					if ((nosensor_flag) || (coldwash_flag) || (reftemp1 < T) || (!hwater_flag))
					{
						if (prewashmode)
						{
							HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
						}
						else
						{
							HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_SET);
						}
						HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
					}
					else
					{
						HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
					}
				}
			}
			if ((hydro_flag) && (washmode) && (hydro2 > hydroempty2 - 30))
			{
				E21_timer++;
				if (E21_timer > 130)
					E21_flag = 1;
			}
			else
			{
				E21_timer = 0;
			}
			if (mission_timer > 610 - washmode * 200)
				E22_flag = 1;
			if ((!hydro_flag) && (wef_flag) && (!wshc_flag))
			{
				wshcnt = mission_timer * 1.9;
				wshc_flag = 1;
			}
			if ((hydro_flag) && (!mixok_flag) && (washmode) && (hydro2 - ((hydroempty - hydrofull) / 4) < hydrofull))
			{
				washing_flag = 1;
				washst_flag = 1;
			}
			else
			{
				left_flag = 0;
				right_flag = 0;
				washing_flag = 0;
				washst_flag = 0;
				washing_cnt = washresttime - 2;
				tccnt_flag = 0;
			}
			if ((!hydro_flag) && (wef_flag) && ((washmode) || ((!washmode) && (wshcnt < mission_timer))))
			{
				m5_flag = 1;
			}
			else
			{
				m5_flag = 0;
			}
			if (((hydro2 - 35 + (1 - washmode + 1.3 * woolmode * washmode) * ((hydroempty - hydrofull) / 3)) < (hydrofull)) && (hydro_flag))
			{
				m1_flag = 1;
			}
			else
			{
				m1_flag = 0;
			}
			if (mission_timer > 30)
			{
				m2_flag = 1;
			}
			else
			{
				m2_flag = 0;
			}
			if ((!left_flag) && (!right_flag))
			{
				m4_flag = 1;
			}
			else
			{
				m4_flag = 0;
			}
			if ((((m1_flag) && (m4_flag)) || (m5_flag)) && ((m2_flag) || (nodel30_flag)))
			{
				hydchk++;
			}
			else
			{
				hydchk = 0;
			}
			if (hydchk > 11)
			{
				E21_timer = 0;
				wshc_flag = 0;
				rinsetime_cnt = 0;
				motor_speed = motor_speed_start;
				if ((washmode) || (ww == 7))
				{
					first_turn_motor = 0;
					first_grab = 0;
				}
				else
				{
					motor_speed_slctL = motor_speed_start;
					motor_speed_slctR = motor_speed_start;
					first_turn_motor = 1;
					first_grab = 1;
				}
				HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
				mission_timer = 0;
				if ((!nosensor_flag) && (!coldwash_flag) && (reftemp1 > T) && (washmode) && (((!hydro_flag) && (wef_flag)) || ((hydro_flag) && (hydro2 < (2 * hydrofull + hydroempty) / 3))))
					HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
				taco_cnt = 0;
				newresttime = 7;
				newturntime = 20;
				changetr_flag = 1;
				washresttime = 7;
				washturntime = 20;
				washing_cnt = washresttime - 2;
				rinsetime_cnt = 0;
				T_temp = T;
				T_temp2 = T;
				ttemp2_cnt = 0;
				if (wtme > 0)
					wtme--;
				program_select = 4;
				hs = 300;
				mixok_flag = 0;
				rinsetime_cnt = 0;
				washcasetime = 0;
				eesaves_flag = 1;
				eesc = 4;
			}
			break;
		case 4: // washing
			check_E();
			washing_flag = 1;
			washst_flag = 1;
			if ((!OpRy_flag) && (washmode))
			{
				hs++;
				if (hs > 299)
				{
					HAL_GPIO_TogglePin(Door_Open_GPIO_Port, Door_Open_Pin);
					hs = 0;
				}
			}
			washcasetime++;
			if (woolmode)
			{
				newresttime = 15;
				newturntime = 7;
				if (mission_timer == 10)
					changetr_flag = 1;
			}
			else if ((left_flag) && (!right_flag) && (washturntime == 20))
			{
				newresttime = 8;
				newturntime = 13;
				changetr_flag = 1;
			}
			if ((coldwash_flag) || (reftemp1 < T) || (E3c_flag) || (nosensor_flag) || ((!hydro_flag) && (!hydpin2_flag)))
				HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
			if (mission_timer > 59)
			{
				if ((!E3c_flag) && (washmode))
				{
					if ((!nosensor_flag) && (ww != 0))
					{
						if ((!coldwash_flag) && (HAL_GPIO_ReadPin(Heater_GPIO_Port, Heater_Pin)))
						{
							ttemp2_cnt++;
							if (T < 0)
								E39_flag = 1;
							if (ttemp2_cnt > 4)
							{
								if (T_temp2 + 2 > T)
									E31_flag = 1;
								T_temp2 = T;
								ttemp2_cnt = 0;
							}
							if (T > T_temp + 20)
								E32_flag = 1;
							T_temp = T;
						}
						else
						{
							if ((reftemp1 < 72) && (75 < T) && (reftemp1 + 20 < T))
								E33_flag = 1;
							if ((reftemp1 < 95) && (95 < T))
								E33_flag = 1;
						}
					}
				}
				if (prewashmode)
				{
					if (prewashing_time > 0)
						prewashing_time--;
				}
				else if (washmode)
				{
					if (washing_time > 0)
						washing_time--;
				}
				else
				{
					if (rinsetime_cnt < 10)
						rinsetime_cnt++;
					if (rtme > 0)
						rtme--;
				}
				mission_timer = 0;
			}
			if ((!left_flag) && (!right_flag) && (hydro_flag) && (rest_tmr > 3))
			{
				if (hydro2 < (((2.8 - (0.8 * washmode)) * hydrofull) + (((0.8 * washmode) - 1.8) * hydroempty)))
				{
					E23_cnt++;
					if (E23_cnt > 59)
						E23_flag = 1;
				}
				else
				{
					E23_cnt = 0;
				}
				if (hydro2 > ((hydrofull + 3 * hydroempty) / 4))
				{
					E24_cnt++;
					if (E24_cnt > 7)
						E24_flag = 1;
				}
				else
				{
					E24_cnt = 0;
				}
			}
			HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
			if ((((prewashmode) && (prewashing_time == 0)) || ((washmode) && (washing_time == 0)) || (rinsetime_cnt > 2)) && (washcasetime > 179))
			{
				HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
				if ((T > 44) && (hydro_flag) && ((2 * hydrofull - hydroempty) < hydro2) && (cooling_timer < 60))
				{
					E23_cnt = 0;
					cooling_timer++;
					HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_SET);
					HydEr_timer = 0;
				}
				else if ((!washmode) && (rinsemode < 2)) // Islamic wash
				{
					HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_SET);
					cooling_timer++;
					if (cooling_timer > 14)
						HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
					if (cooling_timer > 29)
					{
						cooling_timer = 0;
						washing_cnt = 0;
						washing_flag = 0;
						hydro_timer = 0;
						E5lchk_flag = 0;
						HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
						motor_relay_request(MOTOR_RELAY_CMD_OFF);
						HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
						program_select = 5;
						rinsetime_cnt = 0;
						hydro_temp = hydro2;
						allow_relay = 0;
						washst_flag = 0;
						washing_flag = 0;
						right_flag = 0;
						left_flag = 0;
						if (woolmode)
						{
							newresttime = 15;
							newturntime = 7;
						}
						run_motor = 0;
						mission_timer = 0;
					}
				}
				else
				{
					if (woolmode)
					{
						newresttime = 15;
						newturntime = 7;
					}
					mission_timer = 0;
					program_select = 5;
					rinsetime_cnt = 0;
					cooling_timer = 0;
					washing_cnt = 0;
					washing_flag = 0;
					hydro_timer = 0;
					E5lchk_flag = 0;
					HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
					motor_relay_request(MOTOR_RELAY_CMD_OFF);
					hydro_temp = hydro2;
					allow_relay = 0;
					washst_flag = 0;
					right_flag = 0;
					left_flag = 0;
					run_motor = 0;
					mission_timer = 0;
				}
				eesaves_flag = 1;
				eesc = 4;
			}
			break;
		case 5: // drain
			check_E();
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
			left_flag = 0;
			right_flag = 0;
			savewtr_flag = 0;
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
			if ((hydro_flag) && (mission_timer > 120) && (hydro_flag) && (hydrofull + 30 > hydro2))
				E61_flag = 1;
			if (mission_timer > 240)
				E62_flag = 1;
			if ((!hydro_flag) && (!hydpin_flag))
			{
				E61_flag = 0;
			}
			if ((hydro2 > hydroempty - 75) || (!wef_flag))
			{
				hydro_timer++;
			}
			else
			{
				hydro_timer = 0;
			}
			if (hydro_timer > 4)
			{
				wef_flag = 0;
				hydro_timer = 0;
				washing_cnt = 0;
				washst_flag = 0;
				washing_flag = 0;
				turn_motor = 0;
				spin_mode = 0;
				washcasetime = 0;
				E5lchk_flag = 0;
				HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
				motor_relay_request(MOTOR_RELAY_CMD_OFF);
				allow_relay = 0;
				if (dtme > 0)
					dtme--;
				if ((washmode == 0) && (rinsemode < 2))
				{
					mission_timer = 10;
					run_motor = 0;
					first_turn_motor = 0;
					first_grab = 0;
					grab_cnt = 0;
					maxtaco = plm3;
					mintaco = plm3 - 2;
					left_flag = 0;
					right_flag = 0;
					washresttime = 7;
					washturntime = 14;
					cc = 0;
					washing_flag = 0;
					washst_flag = 0;
					washing_cnt = 0;
					program_select = 7;
					if (nospin_flag)
						mission_timer = 500;
					first_turn_motor = 0;
					first_grab = 0;
					grab_cnt = 0;
					motor_speed = motor_speed_start;
				}
				else
				{
					program_select = 6;
					mission_timer = 25;
					//							if ((washmode==0)&&(rinsemode==2)&&(ww==0)) mission_timer=190;
					if (nospin_flag)
						mission_timer = 190;
					first_turn_motor = 0;
					first_grab = 0;
					grab_cnt = 0;
					motor_speed = motor_speed_start;
				}
				eesaves_flag = 1;
				eesc = 4;
			}
			break;
		case 6: // spin low
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
			check_E();
			if (fastspin == 0)
				mission_timer = 180;
			if (mission_timer < 30)
			{
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
				run_motor = 0;
				washing_flag = 0;
				washst_flag = 0;
				spin_mode = 0;
				washing_cnt = 0;
				turn_cnt = 0;
				twiceturn_flag = 0;
			}
			else if (mission_timer < 110)
			{
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
				if ((mission_timer == 90) && (stme > 0))
				{
					stme--;
				}
				if (!spin_mode)
				{
					spin_mode = 0;
					washing_flag = 1;
					washst_flag = 1;
					if (mission_timer > 105)
						mission_timer = 100;
				}
				if ((left_flag) && (!right_flag) && ((turn_cnt > 2) || (washing_cnt > washturntime * 2 + washresttime * 2 - 3) || (spin_mode)))
				{
					if (twiceturn_flag)
					{
						spin_mode = 1;
						washing_flag = 0;
						washing_cnt = 0;
						washst_flag = 0;
						run_motor = 1;
						spinspeed = 15;
						spin_mode = 1;
					}
				}
			}
			else if (mission_timer < 160)
			{
				run_motor = 1;
				spinspeed = 48;
				spin_mode = 1;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
			}
			else if (mission_timer < 170)
			{
				spinspeed = 0;
				spin_mode = 0;
				run_motor = 0;
				allow_relay = 0;
			}
			else
			{
				washing_cnt = 0;
				washing_flag = 0;
				turn_motor = 0;
				mission_timer = 0;
				E5lchk_flag = 0;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
				motor_relay_request(MOTOR_RELAY_CMD_OFF);
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				allow_relay = 0;
				motor_speed_slctL = motor_speed_start;
				motor_speed_slctR = motor_speed_start;
				if (stme > 0)
					stme--;
				rinsetime_cnt = 0;
				if (prewashmode)
				{
					program_select = 3;
					prewashmode = 0;
					HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				}
				else if (washmode)
				{
					program_select = 3;
					HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
					washmode = 0;
				}
				else if (rinsemode > 1)
				{
					rinsemode--;
					left_flag = 0;
					right_flag = 0;
					program_select = 3;
					HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				}
				else
				{
					mission_timer = 10;
					motor_speed = motor_speed_start;
					first_turn_motor = 0;
					program_select = 7;
					if (nospin_flag)
						mission_timer = 500;
				}
				if (prewashing_time == 0)
					prewashmode = 0;
				if (washing_time == 0)
					washmode = 0;
				eesaves_flag = 1;
				eesc = 4;
			}
			break;
		case 7: // spin fast
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
			if (mission_timer < 15)
			{
				motor_speed = motor_speed_start;
				check_E();
				first_turn_motor = 0;
				wtme = 0;
				dtme = 0;
				stme = 0;
				rtme = 0;
				if (mission_timer > 9)
					HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
				run_motor = 0;
				washing_flag = 0;
				washst_flag = 0;
				spin_mode = 0;
				washing_cnt = 0;
				turn_cnt = 0;
				if (!sosp_flag)
				{
					if (ss == 0)
						spintime = 5;
					if (ss == 1)
						spintime = 6;
					if (ss == 2)
						spintime = 6;
					if (ss == 3)
						spintime = 7;
					if (ss == 4)
						spintime = 8;
					if (ss == 5)
						spintime = 2;
				}
				twiceturn_flag = 0;
			}
			else if (mission_timer < 140)
			{
				if ((mission_timer == 61) && (spintime > 0) && (sosp_flag))
					spintime--;
				if ((mission_timer == 120) && (spintime > 0) && (sosp_flag))
					spintime--;
				if (!spin_mode)
				{
					HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
					washing_flag = 1;
					washst_flag = 1;
					spin_mode = 0;
				}
				if ((left_flag) && (!right_flag) && ((turn_cnt > 2) || (washing_cnt > washturntime * 2 + washresttime * 2 - 3) || (spin_mode)))
				{
					if (twiceturn_flag)
					{
						spin_mode = 1;
						washing_flag = 0;
						washing_cnt = 0;
						washst_flag = 0;
						run_motor = 1;
						spinspeed = 15;
						spin_mode = 1;
					}
				}
			}
			else if (mission_timer < 170) // 140 to 170
			{
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				motor_relay_request(MOTOR_RELAY_CMD_UI1);
				if (allow_relay)
					E5lchk_flag = 1;
				run_motor = 1;
				spinspeed = plm * 1.08;
				spin_mode = 1;
				if ((!sosp_flag) && (mission_timer == 165))
				{
					mission_timer = 2;
					sosp_flag = 1;
					E5lchk_flag = 0;
					motor_relay_request(MOTOR_RELAY_CMD_OFF);
				}
			}
			else if (mission_timer < 225) // 170 to 225		400rpm
			{
				if ((mission_timer == 186) && (spintime > 1))
					spintime--;
				motor_relay_request(MOTOR_RELAY_CMD_UI1);
				if (allow_relay)
					E5lchk_flag = 1;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
				run_motor = 1;
				spinspeed = plm * 1.6;
				spin_mode = 1;
			}
			else if (mission_timer < 285) // 225 to 285		600rpm
			{
				motor_relay_request(MOTOR_RELAY_CMD_UI1);
				if (allow_relay)
					E5lchk_flag = 1;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				run_motor = 1;
				spinspeed = plm * 2.4;
				spin_mode = 1;
				if ((mission_timer == 240) && (spintime > 1))
					spintime--;
				if (fastspin < 20)
					mission_timer = 420;
			}
			else if (mission_timer < 330) // 285 to 330		800rpm
			{
				motor_relay_request(MOTOR_RELAY_CMD_UI1);
				if (allow_relay)
					E5lchk_flag = 1;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
				run_motor = 1;
				spinspeed = plm * 3.2;
				spin_mode = 1;
				if ((mission_timer == 300) && (spintime > 1))
					spintime--;
				if (fastspin < 30)
					mission_timer = 420;
			}
			else if (mission_timer < 375) // 330 to 375		1000rpm
			{
				motor_relay_request(MOTOR_RELAY_CMD_UI1);
				if (allow_relay)
					E5lchk_flag = 1;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				run_motor = 1;
				spinspeed = plm * 4;
				spin_mode = 1;
				if ((mission_timer == 360) && (spintime > 1))
					spintime--;
				if (fastspin < 40)
					mission_timer = 420;
			}

			else if (mission_timer < 420) // 375 to 420		1200rpm
			{
				motor_relay_request(MOTOR_RELAY_CMD_UI1);
				if (allow_relay)
					E5lchk_flag = 1;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				run_motor = 1;
				spinspeed = plm * 4.8;
				spin_mode = 1;
				washst_flag = 0;
				if (fastspin < 50)
					mission_timer = 420;
			}
			else if (mission_timer < 450) // 420 to 450
			{
				if ((mission_timer == 420) && (spintime > 1))
					spintime--;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
				E5lchk_flag = 0;
				if (mission_timer > 420)
				{
					motor_speed = motor_speed_start;
					run_motor = 0;
				}
				spin_mode = 0;
				Hlmn = 0;
				EEkey[0] = 'H';
				EEDATA = 0;
				if (EHl != 0)
				{
					EEwr_func();
					if (Hlmn_flag)
					{
						EHl = 0;
					}
				}
				washing_cnt = 0;
				washst_flag = 0;
				turn_motor = 0;
				newresttime = 4;
				newturntime = 7;
				changetr_flag = 1;
				washresttime = 4;
				washturntime = 7;
				allow_relay = 0;
				spintime = 1;
				first_turn_motor = 0;
				first_grab = 0;
				grab_cnt = 0;
				E5lchk_flag = 0;
				motor_relay_request(MOTOR_RELAY_CMD_OFF);
				motor_speed = motor_speed_start;
				first_turn_motor = 0;
			}
			else if (mission_timer < 472) // 450 to 472
			{
				washing_flag = 1;
				washst_flag = 1;
				run_motor = 1;
				maxtaco = plm3 + 2;
				mintaco = plm3 - 1;
				E5lchk_flag = 0;
				if ((mission_timer == 470) && (spintime > 0))
					spintime--;
			}
			else if (mission_timer < 490) // 472 to 490
			{
				washing_flag = 0;
				washst_flag = 0;
				E5lchk_flag = 0;
				motor_relay_request(MOTOR_RELAY_CMD_OFF);
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
				if (!swdr_flag)
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
				run_motor = 0;
			}
			else if (mission_timer < 595) // 490 to 595
			{
				spintime = 0;
				E5lchk_flag = 0;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				BeforeSleep();
				mission_timer = 510;
				spintime = 0;
			}
			else
			{
				socket_send("*#XXendXX@$\r\n");
				end_flag = 1;
				onwm_flag = 0;
				socket_send("*#XXendXX@$\r\n");
			}
			break;
		}
	}
	mission_flag = 0;
}

/******************************************************************************/
void spchg_func()
{
	if ((allow_relay) && (left_flag) && (run_motor) && (!spin_mode)) //////turn_motor?????
	{
		if ((program_select != 7) && (taco_cnt > maxtaco + 12))
			motor_speed_slctL -= 1;
		if ((program_select != 7) && (taco_cnt > maxtaco + 4))
			motor_speed_slctL -= 4;
		if (taco_cnt > maxtaco)
			motor_speed_slctL -= 1;
		if ((program_select != 7) && (taco_cnt + 10 < mintaco) && (cc > 2) && (cc < 7))
			motor_speed_slctL += 2;
		if (taco_cnt < mintaco)
		{
			motor_speed_slctL += 1; // incr motor speed  vvvv
		}
		if ((taco_cnt <= maxtaco) && (taco_cnt >= mintaco))
		{
			if (grab_cnt < 10)
				grab_cnt++;
			if ((grab_cnt > 3) && (!first_grab))
			{
				first_grab = 1;
				motor_speed_slctR = motor_speed_slctL - 10;
			}
		}
		if (motor_speed_slctL < 50)
			motor_speed_slctL = 50;
		taco_cnt = 0;
	}
	if ((allow_relay) && (right_flag) && (run_motor) && (!spin_mode))
	{
		if ((program_select != 7) && (taco_cnt > maxtaco + 12))
			motor_speed_slctR -= 1;
		if ((program_select != 7) && (taco_cnt > maxtaco + 4))
			motor_speed_slctR -= 1;
		if (taco_cnt > maxtaco)
			motor_speed_slctR -= 1;
		if ((program_select != 7) && (taco_cnt + 10 < mintaco) && (cc > 2) && (cc < 7))
			motor_speed_slctR += 2;
		if (taco_cnt < mintaco)
		{
			motor_speed_slctR += 1; // incr motor speed  vvvv
		}
		if ((taco_cnt <= maxtaco) && (taco_cnt >= mintaco))
		{
			if (grab_cnt < 10)
				grab_cnt++;
			if ((grab_cnt > 3) && (!first_grab))
			{
				first_grab = 1;
				motor_speed_slctL = motor_speed_slctR - 10;
			}
		}
		if (motor_speed_slctR < 50)
			motor_speed_slctR = 50;
		taco_cnt = 0;
	}
	spchg_flag = 0;
}

/******************************************************************************/
void resume_func()
{
	if (!run_flag)
	{
		if (washmode)
		{
			washing_scnt = 3;
		}
		else if (program_select == 7)
		{
			spinning_scnt = 3;
		}
		else
		{
			rinsing_scnt = 2;
		}
		resume_cnt = 3;
	}
	if (((!hydro_flag) && (wef_flag)) || ((hydro_flag) && (hydro2 < (2 * hydrofull + hydroempty) / 3) && (Heater_temp)))
	{
		if ((!nosensor_flag) && (!coldwash_flag))
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
	}
	if (Valve1_temp)
	{
		HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
	}
	if (DrainP_temp)
	{
		HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
	}
	if (!run_flag)
		run_flag = 1;
	if ((!OpRy_flag) && (program_select == 4))
	{
		HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
		hs = 0;
	}
	tm2 = 0;
	hyd_cnt = 0;
	run_flag = 1;
	resume_flag = 0;
}

/******************************************************************************/
void pause_func()
{
	Heater_temp = HAL_GPIO_ReadPin(Heater_GPIO_Port, Heater_Pin);
	Valve1_temp = HAL_GPIO_ReadPin(Valve1_GPIO_Port, Valve1_Pin);
	Valve2_temp = HAL_GPIO_ReadPin(Valve2_GPIO_Port, Valve2_Pin);
	Valve3_temp = HAL_GPIO_ReadPin(Valve3_GPIO_Port, Valve3_Pin);
	DrainP_temp = HAL_GPIO_ReadPin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin);
	HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
	motor_relay_request(MOTOR_RELAY_CMD_OFF);
	if (!OpRy_flag)
		HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
	allow_relay = 0;
	E5lchk_flag = 0;
	if ((mission_timer > 60) && (program_select == 6))
		mission_timer = 61;
	if (program_select > 5)
		mission_timer = 0;
	washing_cnt = 0;
	if (program_select == 7)
	{
		run_motor = 0;
		turn_motor = 0;
		E5lchk_flag = 0;
		first_turn_motor = 0;
		first_grab = 0;
		grab_cnt = 0;
		spin_mode = 0;
		maxtaco = plm3;
		mintaco = plm3 - 2;
		left_flag = 0;
		right_flag = 0;
		washresttime = 7;
		washturntime = 14;
		cc = 0;
		washing_flag = 0;
		washst_flag = 0;
		washing_cnt = 0;
		first_turn_motor = 0;
		first_grab = 0;
		grab_cnt = 0;
		motor_speed = motor_speed_start;
	}
	if (program_select > 5)
		motor_speed = motor_speed_start;
	if (!Error_flag)
	{
		socket_send("*#XXpauseXX@$\r\n");
		pause_cnt = 3;
	}
	if (run_flag)
		run_flag = 0;
	run_flag = 0;
	left_flag = 0;
	right_flag = 0;
	spin_mode = 0;
	turn_cnt = 0;
	pause_flag = 0;
}

/******************************************************************************/
void EraseVar_func()
{
	ms250_flag = 0;
	prewashmode = 0;
	key_flag = 0;
	verify_flag = 0;
	Error_sflag = 0;
	ressa = 0;
	procng_flag = 0;
	EEErase = 0;
	Error_flag = 0;
	oow_flag = 1;
	bufftest4[4] = '0';
	firstadc_flag = 1;
	buzzercnt = 1;
	E01_flag = 0;
	E02_flag = 0;
	E11_flag = 0;
	E21_flag = 0;
	E22_flag = 0;
	E23_flag = 0;
	E24_flag = 0;
	E30_flag = 0;
	E31_flag = 0;
	E32_flag = 0;
	E33_flag = 0;
	E34_flag = 0;
	E37_flag = 0;
	E38_flag = 0;
	E39_flag = 0;
	E41_flag = 0;
	E51_flag = 0;
	E61_flag = 0;
	E62_flag = 0;
	E63_flag = 0;
	E23_cnt = 0;
	E34_cnt = 0;
	E51_cnt = 0;
	E63_cnt = 0;
	tm4 = 0;
	tm5 = 0;
	hydpin_flag = 0;
	wef_flag = 0;
	drm_flag = 0;
	drt_flag = 0;
	dr2_flag = 0;
	dml_flag = 0;
	dmr_flag = 0;
	ddr_flag = 0;
	dv1_flag = 0;
	dv2_flag = 0;
	dv3_flag = 0;
	ddl_flag = 0;
	dhr_flag = 0;
	dwl1_flag = 0;
	dwl2_flag = 0;
	dt30_flag = 0;
	debug_show = 0;
	showfb_flag = 0;
	ChLk_flag = 0;
	plkey_flag = 0;
	plkey_flag2 = 0;
	progs_flag = 0;
	wshc_flag = 0;
	test_flag = 0;
	chgdr_flag = 0;
	savewtr_flag = 0;
	nodel30_flag = 0;
	error_cmd = 0;
	tmhw_flag = 0;
	eesaves_flag = 0;
	tmf_flag = 0;
	tmt_flag = 0;
	tmmr_flag = 0;
	tmml_flag = 0;
	tmr_flag = 0;
	tmd_flag = 0;
	tmv1_flag = 0;
	tmv2_flag = 0;
	tmv3_flag = 0;
	pl_flag = 0;
	fourturn_flag = 0;
	lockdoor_flag = 0;
	m1_flag = 0;
	m2_flag = 0;
	m3_flag = 0;
	m4_flag = 0;
	m5_flag = 0;
	fto_flag = 0;
	ea_flag = 0;
	Dlmn_flag = 0;
	Glmn_flag = 0;
	Hlmn_flag = 0;
	EEkey[0] = 'H';
	EEDATA = 0;
	if (EHl != 0)
	{
		EEwr_func();
		if (Hlmn_flag)
		{
			EHl = 0;
		}
	}
	Ilmn_flag = 0;
	Jlmn_flag = 0;
	Klmn_flag = 0;
	Llmn_flag = 0;
	Mlmn_flag = 0;
	Nlmn_flag = 0;
	Olmn_flag = 0;
	Plmn_flag = 0;
	Qlmn_flag = 0;
	Rlmn_flag = 0;
	Slmn_flag = 0;
	Tlmn_flag = 0;
	Ulmn_flag = 0;
	Vlmn_flag = 0;
	Wlmn_flag = 0;
	Ylmn_flag = 0;
	Zlmn_flag = 0;
	nospin_flag = 0;
	trig_flag = 0;
	trig_flag2 = 0;
	G2lmn_flag = 0;
	c = 0;
	z = 0;
	tc_cnt = 0;
	ww = 88;
	tt = 88;
	ss = 88;
	pp = 88;
	num2 = 88;
	nrcnt = 0;
	nr3 = 0;
	ll = 0;
	rr = 0;
	CC = 0;
	aa = 0;
	vfc = 0;
	cpi = 0;
	cpd = 0;
	cpf = 0;
	dlcntt = 0;
	buzzoff = 0;
	stop_off_timer = 0;
	eesc = 0;
	E00_scnt = 1;
	chgdr_cnt = 11;
	Error_timer = 0;
	turn_cnt = 0;
	wrck = 0;
	min_cnt = 0;
	fourtime = 0;
	tol = 0;
	tmsb = 1;
	doorlock_cnt = 0;
	reftemp1 = 10;
	wefcnt = 0;
	yx = 0;
	yw2 = 0;
	yw = 0;
	yz = 0;
	counter = 0;
	autodebug_flag = 0;
	reg_flag = 1;
}

/******************************************************************************/
void check_E()
{
	Hlmn = (ss * 10) + ww;
	Hlmn = (Hlmn * 10) + tt;
	if (washmode)
		Hlmn = Hlmn + 5;
	Hlmn = (Hlmn * 10) + rinsemode;
	if (prewashmode)
		Hlmn = Hlmn + 5;
	Hlmn = (Hlmn * 10) + program_select;
	EEkey[0] = 'H';
	EEDATA = Hlmn;
	if (EEDATA != EHl)
	{
		EEwr_func();
		if (Hlmn_flag)
		{
			EHl = Hlmn;
		}
	}
}

/******************************************************************************/
void stop_func()
{
	run_flag = 0;
	rinsetime_cnt = 0;
	run_motor = 0;
	turn_motor = 0;
	E5lchk_flag = 0;
	first_turn_motor = 0;
	washcasetime = 0;
	first_grab = 0;
	grab_cnt = 0;
	notsave_flag = 0;
	maxtaco = plm3;
	mintaco = plm3 - 2;
	washresttime = 5;
	washturntime = 20;
	newresttime = 3;
	newturntime = 20;
	right_flag = 0;
	left_flag = 0;
	spinspeed = 0;
	fastspin = 0;
	delay_start_flag = 0;
	delay_timer = 0;
	motor_speed = motor_speed_start;
	washing_time = 0;
	washing_flag = 0;
	washst_flag = 0;
	washing_cnt = 0;
	washmode = 0;
	rinsemode = 0;
	spin_mode = 0;
	spintime = 0;
	mission_flag = 0;
	mission_timer = 0;
	program_select = 0;
	rxw_flag = 0;
	txw_flag = 0;
	spchg_flag = 0;
	allow_relay = 0;
	coldwash_flag = 0;
	firstword_flag = 0;
	secondword_flag = 0;
	firstwordE_flag = 0;
	secondwordE_flag = 0;
	changetr_flag = 0;
	E21_timer = 0;
	E34_cnt = 0;
	E51_cnt = 0;
	E63_cnt = 0;
	timer_show = 0;
	ready_scnt = 1;
	washing_scnt = 1;
	pausing_scnt = 1;
	rinsing_scnt = 1;
	spinning_scnt = 1;
	prewashing_time = 0;
	unreg_scnt = 1;
	pause_cnt = 1;
	resume_cnt = 1;
	hydro_timer = 0;
	HydEr_timer = 0;
	hyd_cnt = 0;
	taco_cnt = 0;
	taco_stop_tmr = 0;
	delay_timer = 0;
	ttemp2_cnt = 0;
	adc_rd2 = 0;
	adc_rd3 = 0;
	hydroread_flag = 1;
	needheat_flag = 0;
	Heater_temp = 0;
	Valve1_temp = 0;
	DrainP_temp = 0;
	E3c_flag = 0;
	sosp_flag = 0;
	nba = 0;
	relaytmr = 0;
	hydroempty2 = hydroempty;
	if ((swdr_flag) && (!DnOpDr_flag) && (EHl != 0))
	{
		if (doorlock_flag)
		{
			if ((swdr_flag) && (!Error_flag))
				master_relay_request(1);
			if (OpRy_flag)
			{
				HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
			}
			HAL_Delay(20);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
		}
		offcnt = 0;
		doorlock_cnt = 0;
		while ((doorlock_flag) && (offcnt < 1000))
		{
			if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
				doorlock_cnt = 0;
			doorlock_cnt++;
			if (doorlock_cnt > 200)
				doorlock_flag = 0;
			offcnt++;
			HAL_Delay(1);
			HAL_IWDG_Refresh(&hiwdg);
		}
		if (doorlock_flag)
		{
			master_relay_request(1);
			if (OpRy_flag)
			{
				HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
			}
			HAL_Delay(20);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
		}
	}
	bufftest2[0] = '0';
	if (!hydro_flag)
	{
		bufftest2[1] = '3';
	}
	else
	{
		bufftest2[1] = 'x';
	}
	if (swdr_flag)
	{
		bufftest2[2] = 'v';
	}
	else
	{
		bufftest2[2] = '5';
	}
	bufftest2[3] = '7';
	bufftest2[4] = 'a';
	bufftest2[5] = 'c';
	bufftest2[6] = 'e';
	bufftest2[7] = 'g';
	bufftest2[8] = 'i';
	E5lchk_flag = 0;
	if ((swdr_flag) || (!DnOpDr_flag))
		HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
	motor_relay_request(MOTOR_RELAY_CMD_OFF);
	HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
	if (!swdr_flag)
		master_relay_request(0);
	if ((onwm_flag) && (!end_flag))
		socket_send("*#XXreadyX@$\r\n");
	pause_flag = 0;
	resume_flag = 0;
	woolmode = 0;
	ww = 0;
	ss = 0;
	tt = 0;
	pp = 0;
	cc = 0;
	y = 0;
	j = 0;
	Hlmn_flag = 0;
	EEkey[0] = 'H';
	EEDATA = 0;
	if (EHl != 0)
	{
		EEwr_func();
		if (Hlmn_flag)
		{
			EHl = 0;
		}
	}
	RelaeaseOok_flag = 0;
	w60_flag = 0;
	w90_flag = 0;
	pw60_flag = 0;
	pw90_flag = 0;
	vaoff_flag = 0;
	mixok_flag = 0;
	cpi = 0;
	cpd = 0;
	cpf = 0;
	bsc = 0;
	rest_tmr = 0;
	tccnt_flag = 0;
	Debug_flag = 0;
	drercl_flag = 0;
	err_num = 0;
	stop_flag = 0;
	x = 0;
	x22 = 0;
	taco_cnt = 0;
	hyd_cnt = 0;
	tc = 0;
	tct = 0;
	ttt = 0;
	num3 = 0;
	k = 0;
	k2 = 0;
	tacosafe = 0;
	cooling_timer = 0;
	hydchk = 0;
	fourturn_cnt = 0;
	oow_flag = 1;
	wtme = 0;
	dtme = 0;
	stme = 0;
	rtme = 0;
}

/******************************************************************************/
void NumToStr()
{
	if (numi == 0)
		bufftest4[4] = '0';
	else if (numi == 1)
		bufftest4[4] = '1';
	else if (numi == 2)
		bufftest4[4] = '2';
	else if (numi == 3)
		bufftest4[4] = '3';
	else if (numi == 4)
		bufftest4[4] = '4';
	else if (numi == 5)
		bufftest4[4] = '5';
	else if (numi == 6)
		bufftest4[4] = '6';
	else if (numi == 7)
		bufftest4[4] = '7';
	else if (numi == 8)
		bufftest4[4] = '8';
	else if (numi == 9)
		bufftest4[4] = '9';
}

/******************************************************************************/
void lockdoor_func()
{
	if (swdr_flag)
	{
		if (!doorlock_flag)
		{
			chgdr_cnt++;

			if (chgdr_cnt > 11)
			{
				chgdr_flag = 1;
				chgdr_cnt = 0;
			}
		}
		else
		{
			chgdr_cnt = 11;
		}
	}
	else
	{
		HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
	}
	lockdoor_flag = 0;
}

/******************************************************************************/
void EEwr_func()
{
	memset(bufftest, '\0', 60);
	sprintf(bufftest, "&*w%c%d#", EEkey[0], EEDATA);
	socket_send(bufftest);
}

/******************************************************************************/
void btc_func()
{
	adc_rd3 = 4096 - adc_rd2;
	adc_rd3 = 10000 * adc_rd2 / adc_rd3;
	if (adc_rd3 > 99999)
		adc_rd3 = 99999;
	if (adc_rd3 > 9999)
		sprintf(bufftest, "*#F%5dR%dMxxxxxxxxx%dDxRxSxR%dB%d", hydro2 * 4, adc_rd3, plm, Rif, Beta);
	else if (adc_rd3 > 999)
		sprintf(bufftest, "*#F%5dR0%dMxxxxxxxxx%dDxRxSxR%dB%d", hydro2 * 4, adc_rd3, plm, Rif, Beta);
	else if (adc_rd3 > 99)
		sprintf(bufftest, "*#F%5dR00%dMxxxxxxxxx%dDxRxSxR%dB%d", hydro2 * 4, adc_rd3, plm, Rif, Beta);
	else if (adc_rd3 > 9)
		sprintf(bufftest, "*#F%5dR000%dMxxxxxxxxx%dDxRxSxR%dB%d", hydro2 * 4, adc_rd3, plm, Rif, Beta);
	else
		sprintf(bufftest, "*#F%5dR0000%dMxxxxxxxxx%dDxRxSxR%dB%d", hydro2 * 4, adc_rd3, plm, Rif, Beta);
	if (hydro_flag)
	{
		if (hydro2 < 2500)
		{
			bufftest[3] = '0';
			bufftest[4] = '0';
			bufftest[5] = '0';
			bufftest[6] = '0';
			bufftest[7] = '0';
		}
		else if ((hydro2 * 4) > 99999)
		{
			bufftest[3] = '9';
			bufftest[4] = '9';
			bufftest[5] = '9';
			bufftest[6] = '9';
			bufftest[7] = '9';
		}
	}
	else
	{
		if (hydpin_flag)
		{
			bufftest[3] = '6';
			bufftest[4] = '6';
			bufftest[5] = '6';
			bufftest[6] = '6';
			bufftest[7] = '6';
		}
		else
		{
			bufftest[3] = '5';
			bufftest[4] = '5';
			bufftest[5] = '5';
			bufftest[6] = '5';
			bufftest[7] = '5';
		}
	}
	bufftest[15] = bufftest2[0];
	bufftest[16] = bufftest2[1];
	bufftest[17] = bufftest2[2];
	bufftest[18] = bufftest2[3];
	bufftest[19] = bufftest2[4];
	bufftest[20] = bufftest2[5];
	bufftest[21] = bufftest2[6];
	bufftest[22] = bufftest2[7];
	bufftest[23] = bufftest2[8];
	if ((dml_flag) || (dmr_flag))
	{
		bufftest[3] = 'X';
		bufftest[4] = 'X';
		bufftest[5] = 'X';
		bufftest[6] = 'X';
		bufftest[7] = 'X';
		bufftest[9] = 'X';
		bufftest[10] = 'X';
		bufftest[11] = 'X';
		bufftest[12] = 'X';
		bufftest[13] = 'X';
	}
	if (doorlock_flag)
	{
		bufftest[27] = 'L';
	}
	else
	{
		bufftest[27] = 'O';
	}
	if (OpRy_flag)
	{
		bufftest[29] = '2';
	}
	else
	{
		bufftest[29] = '1';
	}
	if (nosensor_flag)
	{
		bufftest[31] = '1';
	}
	else
	{
		bufftest[31] = '0';
	}

	calT();

	adc_rd3 = 4096 - adc_buf[0];
	adc_rd3 = 10000 * adc_buf[0] / adc_rd3;
	T2 = (3950 / log(adc_rd3 / 0.01763)) - 273.15;

	if (T > 99)
		T = 99;
	if (notsave_flag)
	{
		sprintf(bufftest, "%sTPErT", bufftest);
	}
	else if (T >= 10)
	{
		sprintf(bufftest, "%sTP%.0f", bufftest, T);
	}
	else
	{
		sprintf(bufftest, "%sTP0%.0f", bufftest, T);
	}
	if ((T2 >= 0) && (T2 < 100))
	{
		if (T2 >= 10)
		{
			sprintf(bufftest, "%sT2P%.0f@$\r\n", bufftest, T2);
		}
		else
		{
			sprintf(bufftest, "%sT2P0%.0f@$\r\n", bufftest, T2);
		}
	}
	if (T2 < 0)
		sprintf(bufftest, "%sT2P0OL@$\r\n", bufftest);
	if (T2 > 99)
		sprintf(bufftest, "%sT2P0SC@$\r\n", bufftest);
}

// TODO DEBUG
/******************************************************************************/
void Debug_func()
{
	while (Debug_flag)
	{
		if (onems_flag)
		{
			HAL_IWDG_Refresh(&hiwdg);
			master_relay_service_1ms();
			motor_relay_service_1ms();
			if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
			{
				doorlock_flag = 1;
				doorlock_cnt = 0;
			}
			else
			{
				doorlock_cnt++;
				if (doorlock_cnt > 200)
					doorlock_flag = 0;
			}
			run_flag = 1;
			if (doorlock_flag)
				doorlock_cnt++;
			if (smt_flag)
			{
				if (first_turn_motor)
				{
					k++;
					if (k > 4) //  ramp incr times (ms)    vvvv
					{
						if ((allow_relay) && (left_flag) && (run_motor))
						{
							if (motor_speed_slctL > motor_speed)
								motor_speed++;
							if (motor_speed_slctL < motor_speed)
								motor_speed--;
						}
						else if ((allow_relay) && (right_flag) && (run_motor))
						{
							if (motor_speed_slctR > motor_speed)
								motor_speed++;
							if (motor_speed_slctR < motor_speed)
								motor_speed--;
						}
						k = 0;
					}
				}
				else
				{
					k2++;
					if (k2 > 24)
					{
						if ((allow_relay) && (run_motor) && (motor_speed < 2000) && ((left_flag) || (right_flag)))
							motor_speed++;
						k2 = 0;
					}
				}
			}
			if (buzzoff < 150)
				buzzoff++;
			if (buzzoff > 100)
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			if (ms250_flag)
			{
				taco_cnt4 = taco_cnt3;
				taco_cnt3 = 0;
				socket_init();
				if (run_motor)
					spchcnt++;
				spchg_func();
				spchcnt = 0;
				adc_rd2 = adc_buf[1];
				if (!hydro_flag)
					hydpin_flag = !HAL_GPIO_ReadPin(Hydro_GPIO_Port, Hydro_Pin);
				wef_flag = hydpin_flag;
				if (EEErase)
				{
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					while (!ea_flag)
					{
						socket_send("&*ea#");
						ee3s_flag = 0;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
					}
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
					socket_send("*#XXendXX@$\r\n");
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					socket_send("*#XXendXX@$\r\n");
					Debug_flag = 0;
					onwm_flag = 0;
				}
				if ((ee30_flag) && ((!hydro_flag) || ((eef_flag) && (eee_flag))))
				{
					wrck++;
					if (wrck > 3)
					{
						socket_send("&*wG64#");
						EEDATA = 64;
						wrck = 0;
					}
					if (Glmn_flag)
					{
						ee30_flag = 0;
						eef_flag = 0;
						eee_flag = 0;
						Glmn_flag = 0;
						wrck = 4;
					}
				}
				master_relay_request(1);
				j++;
				if (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin))
				{
					wrck = 20;
					wrckout = 0;
					end_flag = 1;
					socket_send("*#XXendXX@$\r\n");
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(100);
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					socket_send("*#XXendXX@$\r\n");
					Debug_flag = 0;
					onwm_flag = 0;
				}
				else if (tmsb > 2)
				{
					socket_send(bufftest);
					tmsb--;
				}
				else if (j > 11)
				{
					btc_func();
					socket_send(bufftest);
					j = 0;
				}
				else if (j > 10)
				{
					socket_init();
				}
				else if ((hydro_flag) && (j < 11) && (j > 2))
				{
					if ((!left_flag) && (!right_flag))
					{
						hydsum[0] = hydsum[1];
						hydsum[1] = hydsum[2];
						hydsum[2] = hyd_cnt;
						hydro2 = (hydsum[0] + hydsum[1] + hydsum[2]) / 3;
					}
				}
				hyd_cnt = 0;
				if (debug_show)
				{
					socket_send("*#XXdebug@$\r\n");
					tmsb = 4;
					buzzercnt = 2;
					debug_show = 0;
				}
				if (showfb_flag)
				{
					if (doorlock_flag)
					{
						if (!tmf_flag)
						{
							bufftest2[8] = 'j';
							btc_func();
							socket_send(bufftest);
							tmsb = 4;
							showfb_flag = 0;
							tmf_flag = 1;
							j = 10;
						}
					}
				}
				if (tccnt_flag)
				{
					if (!tmt_flag)
					{
						bufftest2[7] = 'h';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tccnt_flag = 0;
						tmt_flag = 1;
						j = 10;
					}
				}
				if (dml_flag)
				{
					left_flag = 1;
					right_flag = 0;
					tmmr_flag = 0;
					dmr_flag = 0;
					if (!tmml_flag)
					{
						bufftest2[0] = '2';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmml_flag = 1;
						j = 10;
					}
					motor_relay_request(MOTOR_RELAY_CMD_UI2);
					if (allow_relay)
					{
						run_flag = 1;
						run_motor = 1;
						if (!smt_flag)
						{
							cm_cnt++;
							if ((cm_cnt > 8) && (run_motor))
							{
								if (motor_speed > 2000)
									motor_speed = 0;
								cm_cnt = 10;
								if (taco_cnt4 < 3)
								{
									motor_speed += 6;
									cmm_cnt = 0;
								}
								if ((taco_cnt4 > 2) && (taco_cnt4 < 20))
								{
									cmm_cnt++;
									if (cmm_cnt > 40)
									{
										EEkey[0] = 'M';
										EEDATA = motor_speed;
										EEwr_func();
										if (Mlmn_flag)
										{
											run_motor = 0;
											dml_flag = 0;
											dmr_flag = 0;
											left_flag = 0;
											motor_relay_request(MOTOR_RELAY_CMD_OFF);
											motor_speed_start = motor_speed;
											first_turn_motor = 0;
											motor_speed_slctL = motor_speed_start + 50;
											motor_speed_slctR = motor_speed_start + 50;
											buzzercnt = 4;
											smt_flag = 1;
										}
									}
								}
								if ((taco_cnt4 > 19) && (taco_cnt4 < 301))
								{
									if (motor_speed > 1)
										motor_speed -= 2;
									cmm_cnt = 0;
								}
								if ((taco_cnt4 > 300))
								{
									motor_speed += 6;
									cmm_cnt = 0;
								}
							}
						}
					}
					else
					{
						motor_speed = motor_speed_start;
					}
				}
				else if (dmr_flag)
				{
					dml_flag = 0;
					left_flag = 0;
					right_flag = 1;
					tmml_flag = 0;
					if (!tmmr_flag)
					{
						bufftest2[0] = '1';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmmr_flag = 1;
						j = 10;
					}
					motor_relay_request(MOTOR_RELAY_CMD_UI1);
					if (allow_relay)
					{
						run_flag = 1;
						run_motor = 1;
						if ((!smt_flag) && (run_motor))
						{
							cm_cnt++;
							if (cm_cnt > 8)
							{
								if (motor_speed > 2000)
									motor_speed = 0;
								cm_cnt = 10;
								if (taco_cnt4 < 3)
								{
									motor_speed += 6;
									cmm_cnt = 0;
								}
								if ((taco_cnt4 > 2) && (taco_cnt4 < 20))
								{
									cmm_cnt++;
									if (cmm_cnt > 40)
									{
										EEkey[0] = 'M';
										EEDATA = motor_speed;
										EEwr_func();
										if (Mlmn_flag)
										{
											run_motor = 0;
											dml_flag = 0;
											dmr_flag = 0;
											right_flag = 0;
											motor_relay_request(MOTOR_RELAY_CMD_OFF);
											motor_speed_start = motor_speed;
											first_turn_motor = 0;
											motor_speed_slctL = motor_speed_start + 50;
											motor_speed_slctR = motor_speed_start + 50;
											buzzercnt = 4;
											smt_flag = 1;
										}
									}
								}
								if ((taco_cnt4 > 19) && (taco_cnt4 < 301))
								{
									if (motor_speed > 1)
										motor_speed -= 2;
									cmm_cnt = 0;
								}
								if ((taco_cnt4 > 300))
								{
									motor_speed += 6;
									cmm_cnt = 0;
								}
							}
						}
					}
					else
					{
						motor_speed = motor_speed_start;
					}
				}
				else
				{
					left_flag = 0;
					right_flag = 0;
					tccnt_flag = 0;
					first_turn_motor = 0;
					first_grab = 0;
					grab_cnt = 0;
					run_flag = 0;
					run_motor = 0;
					if ((tmmr_flag) || (tmml_flag))
					{
						bufftest2[0] = '0';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmml_flag = 0;
						tmmr_flag = 0;
						j = 10;
					}
					motor_relay_request(MOTOR_RELAY_CMD_OFF);
					motor_speed = motor_speed_start;
					relaytmr = 0;
					allow_relay = 0;
				}
				if ((l2ok_flag) && (dhr_flag) && (((!hydro_flag) && (wef_flag)) || ((hydro_flag) && (hydro2 > 2500) && (hydro2 < hydrofull + 50))))
				{
					HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
				}
				if (ddr_flag)
				{
					HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
					if (!tmd_flag)
					{
						if (!hydro_flag)
						{
							bufftest2[1] = '4';
						}
						else
						{
							bufftest2[1] = 'y';
						}
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmd_flag = 1;
						j = 10;
					}
				}
				else
				{
					HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
					if (tmd_flag)
					{
						if (!hydro_flag)
						{
							bufftest2[1] = '3';
						}
						else
						{
							bufftest2[1] = 'x';
						}
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmd_flag = 0;
						j = 10;
					}
				}
				if (hydp_flag)
				{
					wrck++;
					if (wrck > 3)
					{
						socket_send("&*wY22#");
						EEDATA = 22;
						wrck = 0;
					}
					j = 0;
					if (Ylmn_flag)
					{
						if (ddr_flag)
						{
							bufftest2[1] = '4';
						}
						else
						{
							bufftest2[1] = '3';
						}
						buzzercnt = 3;
						hydro_flag = 0;
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						j = 10;
						Ylmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						hydp_flag = 0;
					}
				}
				if (hyde_flag)
				{
					wrck++;
					if (wrck > 3)
					{
						socket_send("&*wY93#");
						EEDATA = 93;
						wrck = 0;
					}
					j = 0;
					if (Ylmn_flag)
					{
						if (ddr_flag)
						{
							bufftest2[1] = 'y';
						}
						else
						{
							bufftest2[1] = 'x';
						}
						buzzercnt = 3;
						hydro_flag = 1;
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						j = 10;
						Ylmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						hyde_flag = 0;
					}
				}
				if (drm_flag)
				{
					wrck++;
					if (wrck > 3)
					{
						socket_send("&*wZ21#");
						EEDATA = 21;
						wrck = 0;
					}
					j = 0;
					if (Zlmn_flag)
					{
						if (tmr_flag)
						{
							bufftest2[2] = '5';
						}
						else
						{
							bufftest2[2] = '6';
						}
						buzzercnt = 3;
						swdr_flag = 0;
						bufftest[29] = '1';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						j = 10;
						Zlmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						drm_flag = 0;
					}
				}
				if (drt_flag)
				{
					wrck++;
					if (wrck > 3)
					{
						socket_send("&*wZ98#");
						EEDATA = 98;
						wrck = 0;
					}
					j = 0;
					if (Zlmn_flag)
					{
						if (tmr_flag)
						{
							bufftest2[2] = 'v';
						}
						else
						{
							bufftest2[2] = 'w';
						}
						buzzercnt = 3;
						swdr_flag = 1;
						OpRy_flag = 0;
						bufftest[29] = '1';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						j = 10;
						Zlmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						drt_flag = 0;
					}
				}
				if (dr2_flag)
				{
					wrck++;
					if (wrck > 3)
					{
						socket_send("&*wZ95#");
						EEDATA = 95;
						wrck = 0;
					}
					j = 0;
					if (Zlmn_flag)
					{
						if (tmr_flag)
						{
							bufftest2[2] = 'v';
						}
						else
						{
							bufftest2[2] = 'w';
						}
						buzzercnt = 3;
						swdr_flag = 1;
						OpRy_flag = 1;
						bufftest[29] = '2';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						j = 10;
						Zlmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						dr2_flag = 0;
					}
				}
				if (dhw_flag)
				{
					wrck++;
					if (wrck > 3)
					{
						if (tmhw_flag)
						{
							socket_send("&*wD1#");
							EEDATA = 1;
						}
						else
						{
							socket_send("&*wD0#");
							EEDATA = 0;
						}
						wrck = 0;
					}
					j = 0;
					if (Dlmn_flag)
					{
						if (tmhw_flag)
						{
							if (!nosensor_flag)
							{
								bufftest2[3] = '7';
							}
							else
							{
								bufftest2[3] = 's';
							}
						}
						else
						{
							if (!nosensor_flag)
							{
								bufftest2[3] = '8';
							}
							else
							{
								bufftest2[3] = 't';
							}
						}
						buzzercnt = 3;
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						j = 10;
						Dlmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						dhw_flag = 0;
					}
				}
				if (dnose_flag)
				{
					wrck++;
					if (wrck > 3)
					{
						if (nosensor_flag)
						{
							EEDATA = 1;
							socket_send("&*wF1#");
						}
						else
						{
							EEDATA = 0;
							socket_send("&*wF0#");
						}
						wrck = 0;
					}
					j = 0;
					if (Flmn_flag)
					{
						if (tmhw_flag)
						{
							if (!nosensor_flag)
							{
								bufftest2[3] = '7';
							}
							else
							{
								bufftest2[3] = 's';
							}
						}
						else
						{
							if (!nosensor_flag)
							{
								bufftest2[3] = '8';
							}
							else
							{
								bufftest2[3] = 't';
							}
						}
						buzzercnt = 3;
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						j = 10;
						Flmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						dnose_flag = 0;
					}
				}
				if (dv1_flag)
				{
					HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_SET);
					if (!tmv1_flag)
					{
						bufftest2[4] = 'b';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmv1_flag = 1;
						j = 10;
					}
				}
				else
				{
					HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
					if (tmv1_flag)
					{
						bufftest2[4] = 'a';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmv1_flag = 0;
						j = 10;
					}
				}
				if (dv2_flag)
				{
					HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_SET);
					if (!tmv2_flag)
					{
						bufftest2[5] = 'd';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmv2_flag = 1;
						j = 10;
					}
				}
				else
				{
					HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
					if (tmv2_flag)
					{
						bufftest2[5] = 'c';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmv2_flag = 0;
						j = 10;
					}
				}
				if (dv3_flag)
				{
					HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_SET);
					if (!tmv3_flag)
					{
						bufftest2[6] = 'f';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmv3_flag = 1;
						j = 10;
					}
				}
				else
				{
					HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
					if (tmv3_flag)
					{
						bufftest2[6] = 'e';
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmv3_flag = 0;
						j = 10;
					}
				}
				if (ddl_flag)
				{
					if (swdr_flag)
					{
						if (!doorlock_flag)
						{
							chgdr_cnt++;
							if (chgdr_cnt > 11)
							{
								chgdr_flag = 1;
								chgdr_cnt = 0;
							}
						}
						else
						{
							chgdr_cnt = 11;
						}
					}
					else
					{
						HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
					}
					if (!tmr_flag)
					{
						if (swdr_flag)
						{
							bufftest2[2] = 'w';
						}
						else
						{
							bufftest2[2] = '6';
						}
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmr_flag = 1;
						j = 10;
					}
				}
				else
				{
					if (swdr_flag)
					{
						if (doorlock_flag)
						{
							chgdr_cnt++;
							if (chgdr_cnt > 11)
							{
								chgdr_flag = 1;
								chgdr_cnt = 0;
							}
						}
						else
						{
							chgdr_cnt = 11;
						}
					}
					else
					{
						HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
					}
					if (tmr_flag)
					{
						if (swdr_flag)
						{
							bufftest2[2] = 'v';
						}
						else
						{
							bufftest2[2] = '5';
						}
						btc_func();
						socket_send(bufftest);
						tmsb = 4;
						tmr_flag = 0;
						j = 10;
					}
				}
				if (dt30_flag)
				{
					wrck++;
					if (wrck == 4)
					{
						memset(bufftest, '\0', 60);
						EEDATA = Rif;
						EEDATA2 = Beta;
						sprintf(bufftest, "&*wN%d#XXXXXXXX&*wO%d#", EEDATA, EEDATA2);
						socket_send(bufftest);
					}
					if (wrck > 7)
					{

						memset(bufftest, '\0', 60);
						EEDATA = Rif;
						EEDATA2 = Beta;
						sprintf(bufftest, "&*wO%d#XXXXXXXX&*wN%d#", EEDATA2, EEDATA);
						socket_send(bufftest);
						wrck = 0;
					}
					j = 10;
					if ((Nlmn_flag) && (Olmn_flag))
					{
						Nlmn_flag = 0;
						Olmn_flag = 0;
						wrck = 4;
						dt30_flag = 0;
						ee30_flag = 1;
						ee3s_flag = 1;
						buzzercnt = 4;
						btc_func();
					}
				}
				if (((l2ok_flag) && (dwl1_flag) && (hydrofull + 200 < hydro2)) || ((dwl1_flag) && (!l2ok_flag)))
				{
					wrck++;
					if (wrck > 3)
					{
						EEkey[0] = 'I';
						EEDATA = hydro2;
						EEwr_func();
						wrck = 0;
					}
					j = 10;
					if (Ilmn_flag)
					{
						dwl1_flag = 0;
						l1ok_flag = 1;
						eee_flag = 1;
						hydroempty = hydro2;
						Ilmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						buzzercnt = 3;
					}
				}
				if (((l1ok_flag) && (dwl2_flag) && (hydroempty > hydro2 + 200)) || ((dwl2_flag) && (!l1ok_flag)))
				{
					wrck++;
					if (wrck > 3)
					{
						EEkey[0] = 'J';
						EEDATA = hydro2;
						EEwr_func();
						wrck = 0;
					}
					j = 10;
					if (Jlmn_flag)
					{
						dwl2_flag = 0;
						l2ok_flag = 1;
						eef_flag = 1;
						Jlmn_flag = 0;
						wrck = 4;
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						hydrofull = hydro2;
						buzzercnt = 3;
					}
				}
				if ((pl_flag) && (plm > 19) && (plm < 51))
				{
					wrck++;
					if (wrck > 3)
					{
						EEkey[0] = 'P';
						EEDATA = plm;
						EEwr_func();
						wrck = 0;
					}
					j = 0;
					if (Plmn_flag)
					{
						plm3 = (6 * plm) / 8;
						maxtaco = plm3 + 1;
						mintaco = plm3;
						pl_flag = 0;
						Plmn_flag = 0;
						wrck = 4;
						buzzercnt = 4;
						btc_func();
					}
				}
				if (buzzercnt > 1)
				{
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
					buzzoff = 0;
					buzzercnt--;
				}
				if (!HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin))
				{
					adcnt++;
				}
				else
				{
					adcnt = 0;
				}
				ms250_flag = 0;
			}
			tm5++;
			if (tm5 > 19)
			{
				if (swdr_flag)
				{
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
					if (chgdr_flag)
					{
						master_relay_request(1);
						if ((OpRy_flag) && (doorlock_flag))
						{
							HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
						}
						else
						{
							HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
						}
						chgdr_flag = 0;
					}
					else
					{
						HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
					}
				}
				tm5 = 0;
			}
			if (program_select > 1)
			{
				if (first_turn_motor)
				{
					k++;
					if (k > 4) //  ramp incr times (ms)    vvvv
					{
						if ((allow_relay) && (left_flag) && (run_motor))
						{
							if (motor_speed_slctL > motor_speed)
								motor_speed++;
							if (motor_speed_slctL < motor_speed)
								motor_speed--;
						}
						else if ((allow_relay) && (right_flag) && (run_motor))
						{
							if (motor_speed_slctR > motor_speed)
								motor_speed++;
							if (motor_speed_slctR < motor_speed)
								motor_speed--;
						}
						k = 0;
					}
				}
				else
				{
					k2++;
					if (k2 > 24)
					{
						if ((allow_relay) && (run_motor) && (motor_speed < 2000))
							motor_speed++;
						k2 = 0;
					}
				}
			}
			onems_flag = 0;
		}
		if (adcnt > 20)
		{
			autodebug_flag = 1;
			Debug_flag = 0;
			adcnt = 0;
			cpi = 0;
			onwm_flag = 1;
		}
	}
	stop_func();
	while (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin))
	{
		HAL_Delay(50);
		HAL_IWDG_Refresh(&hiwdg);
	}
	if ((onwm_flag) && (test_flag) && (!autodebug_flag))
	{
		prewashmode = 0;
		washmode = 0;
		ww = 7;
		pp = 0;
		tt = 0;
		ss = 2;
		if (swdr_flag)
			master_relay_request(1);
		HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
		motor_relay_request(MOTOR_RELAY_CMD_OFF);
		relaytmr = 0;
		rinsemode = 1;
		maxtaco = plm3 + 1;
		mintaco = plm3;
		procng();
		program_select = 1;
		run_flag = 1;
	}
	else
	{
		master_relay_request(0);
	}
	hwater_flag = 0;
	if (Dlmn == 1)	hwater_flag = 1;

	if ((Glmn != 64) && (!autodebug_flag))
		E41_flag = 1;
}

/******************************************************************************/
void procng()
{
	if ((!run_flag) || (program_select < 3))
	{
		if (ww == 0) // Quick wash
		{
			washing_time = 5;
			washmode = 1;
			rinsemode = 2;
			maxtaco = plm3 - 2;
			mintaco = plm3 - 3;
		}
		if (ww == 1) // Cotton
		{
			washing_time = 50;
			washmode = 1;
			rinsemode = 3;
			maxtaco = plm3 - 1;
			mintaco = plm3 - 2;
		}
		if (ww == 2) // Daily
		{
			washing_time = 47;
			washmode = 1;
			rinsemode = 2;
			maxtaco = plm3 - 2;
			mintaco = plm3 - 3;
		}
		if (ww == 3) // Mix
		{
			washing_time = 35;
			washmode = 1;
			rinsemode = 2;
			maxtaco = (plm3 / 1.5) + 1;
			mintaco = (plm3 / 1.5);
		}
		if (ww == 4) // wool
		{
			washing_time = 18;
			washmode = 1;
			rinsemode = 2;
			maxtaco = (plm3 / 2) + 2;
			mintaco = (plm3 / 2) + 1;
			woolmode = 1;
		}
		if (ww == 5) // zir
		{
			washing_time = 43;
			washmode = 1;
			rinsemode = 2;
			maxtaco = plm3 - 2;
			mintaco = plm3 - 3;
		}
		if (ww == 6) // Baby Care
		{
			washing_time = 56;
			washmode = 1;
			rinsemode = 3;
			maxtaco = plm3 - 2;
			mintaco = plm3 - 3;
		}
		if (ww == 7) // rinse+Spin
		{
			prewashmode = 0;
			washmode = 0;
			rinsemode = 1;
			maxtaco = plm3 + 1;
			mintaco = plm3;
		}
		if (ww == 8) // Spin
		{
			prewashmode = 0;
			delay_start_flag = 0;
			washmode = 0;
			rinsemode = 0;
		}
		if (ww == 9) // drain
		{
			prewashmode = 0;
			washmode = 0;
			rinsemode = 0;
			ss = 5;
		}
		if (pp > 3)
		{
			pp = pp - 4;
		}
		prewashmode = 0;
		if ((pp == 1) || (pp == 3))
		{
			prewashmode = 1;
			prewashing_time = 26;
		}
		if ((pp == 2) || (pp == 3))
			rinsemode++;
		nospin_flag = 0;
		if (ss == 0)
			fastspin = 18; // 400*************************************************************************************
		if (ss == 1)
			fastspin = 28; // 600
		if (ss == 2)
			fastspin = 37; // 800
		if (ss == 3)
			fastspin = 45; // 1000
		if (ss == 4)
			fastspin = 56; // 1200
		if (ss == 5)
		{
			fastspin = 0; // no spin
			nospin_flag = 1;
		}
		if (ss == 0)
			spintime = 5;
		if (ss == 1)
			spintime = 6;
		if (ss == 2)
			spintime = 6;
		if (ss == 3)
			spintime = 7;
		if (ss == 4)
			spintime = 8;
		if (ss == 5)
			spintime = 2;
		coldwash_flag = 0;
		w60_flag = 0;
		w90_flag = 0;
		reftemp1 = 10;
		if (tt == 0)
			coldwash_flag = 1;
		if (tt == 1)
		{
			washing_time += 3;
			reftemp1 = 30;
		}
		else if (tt == 2)
		{
			washing_time += 7;
			reftemp1 = 45;
		}
		else if (tt == 3)
		{
			reftemp1 = 60;
			washing_time += 10;
			w60_flag = 1;
			if (prewashmode)
				pw60_flag = 1;
		}
		else if (tt == 4)
		{
			reftemp1 = 82;
			washing_time += 15;
			w90_flag = 1;
			if (prewashmode)
				pw90_flag = 1;
		}
		else
		{
			coldwash_flag = 1;
		}
		tm3 = 0;
		if (run_flag)
			delay_start_flag = 0;
		if (!delay_start_flag)
		{
			program_select = 1;
			run_flag = 1;
		}
		if ((!run_flag) && (error_cmd))
		{
			program_select = 0;
			delay_start_flag = 0;
			run_flag = 0;
		}
	}
	wtme = prewashmode + washmode + rinsemode;
	dtme = prewashmode + washmode + rinsemode;
	if (ww < 8)
		stme = (prewashmode + washmode + rinsemode - 1) * 2;
	if (ww == 9)
	{
		stme = 0;
		spintime = 1;
	}
	rtme = rinsemode * 3;
	procng_flag = 0;
}

/******************************************************************************/
void eesaves_func()
{
	switch (eesc)
	{
	case 0:
		break;
	case 1:
		EEkey[0] = 'Q';
		EEDATA = ww;
		EEwr_func();
		if (Qlmn_flag)
			eesc++;
		break;
	case 2:
		EEkey[0] = 'R';
		EEDATA = ss;
		EEwr_func();
		if (Rlmn_flag)
			eesc++;
		break;
	case 3:
		EEkey[0] = 'S';
		EEDATA = tt;
		EEwr_func();
		if (Slmn_flag)
			eesc++;
		break;
	case 4:
		EEkey[0] = 'T';
		EEDATA = washmode + prewashmode;
		if (Ewp != EEDATA)
		{
			EEwr_func();
			if (Tlmn_flag)
			{
				eesc++;
				Ewp = washmode + prewashmode;
			}
		}
		else
		{
			eesc++;
		}
		break;
	case 5:
		EEkey[0] = 'U';
		EEDATA = rinsemode;
		if (Erns != EEDATA)
		{
			EEwr_func();
			if (Ulmn_flag)
			{
				eesc++;
				Erns = rinsemode;
			}
		}
		else
		{
			eesc++;
		}
		break;
	case 6:
		EEkey[0] = 'V';
		EEDATA = program_select;
		if (Eps != EEDATA)
		{
			EEwr_func();
			if (Vlmn_flag)
			{
				eesc++;
				Eps = program_select;
			}
		}
		else
		{
			eesc++;
		}
		break;
	case 7:
		EEkey[0] = 'K';		  // ee
		EEDATA = motor_speed; // ee
		if (Ems != EEDATA)	  // ee
		{
			EEwr_func();
			if (Klmn_flag) // ee
			{
				eesc++;
				Ems = motor_speed; // ee
			}
		}
		else
		{
			eesc++;
		}
		break;
	case 8:
		EEkey[0] = 'L';				// ee
		EEDATA = motor_speed_slctL; // ee
		if (Eml != EEDATA)			// ee
		{
			EEwr_func();
			if (Llmn_flag) // ee
			{
				eesc++;
				Eml = motor_speed_slctL; // ee
			}
		}
		else
		{
			eesc++;
		}
		break;
	case 9:
		// EEkey[0]='M';												//ee
		// EEDATA=motor_speed_slctR;						//ee
		//				if (Emr!=EEDATA) 										//ee
		//				{
		//					EEwr_func();
		//					if (Mlmn_flag)										//ee
		//					{
		//						eesc++;
		//						Emr=motor_speed_slctR;					//ee
		//					}
		//				}
		//				else
		//				{
		eesc++;
		//				}
		break;
	case 10:
		EEkey[0] = 'W';	   // ee
		EEDATA = run_flag; // ee
		if (Ern != EEDATA) // ee
		{
			EEwr_func();
			if (Wlmn_flag) // ee
			{
				eesc++;
				Ern = run_flag; // ee
			}
		}
		else
		{
			eesc++;
		}
		break;
	case 11:
		eesc = 0;
		eesaves_flag = 0;
		break;
	}
}

/******************************************************************************/
void autodebug()
{
	hydro2 = hyd_cnt;
	hyd_cnt = 0;
	if (cpi == 0)
	{
		cpf++;
		if (cpf > 3)
		{

			if (doorlock_flag)
			{
				socket_send("*#XE15XE15@$\r\n");
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			}
			else if (E37_flag)
			{
				socket_send("*#XE37XE37@$\r\n");
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			} /////OpRy_flagggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg
			else if (E38_flag)
			{
				socket_send("*#XE38XE38@$\r\n");
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			}
			cpf = 0;
		}
		if ((adc_buf[1] < 200) || (adc_buf[1] > 3900))
		{
			E37_flag = 1;
			socket_send("*#XE37XE37@$\r\n");
		}
		else if ((adc_buf[0] < 200) || (adc_buf[0] > 3900))
		{
			E38_flag = 1;
			socket_send("*#XE38XE38@$\r\n");
		}
		else if (!doorlock_flag)
		{
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
			cpi = 1;
			cpd = 0;
		}
	}
	else if ((!doorlock_flag) && (cpi < 12))
	{
		fourtime++;
		cpf++;
		if (cpf > 3)
		{
			if (!drercl_flag)
				socket_send("calprocess");
			cpf = 0;
		}
		if (fourtime > 24)
		{
			socket_send("*#XE19XE19@$\r\n");
			drercl_flag = 1;
			cpf = 0;
			fourtime = 27;
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		}
		else if (cpd > 5)
		{
			swdr_flag = 0;
			EEDATA = 21;
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
		}
		else if (fourtime > 3)
		{
			swdr_flag = 1;
			EEDATA = 98;
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
			HAL_Delay(20);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			cpd++;
			fourtime = 0;
		}
	}
	else
	{
		switch (cpi)
		{
		case 1:
			if (Zlmn_flag)
			{
				if (((swdr_flag) && (Zlmn == 98)) || ((!swdr_flag) && (Zlmn == 21)))
				{
					cpi = 2;
				}
				Zlmn_flag = 0;
			}
			else
			{
				if (swdr_flag)
				{
					socket_send("&*wZ98#");
				}
				else
				{
					socket_send("&*wZ21#");
				}
			}
			break;
		case 2:
			if (Ylmn_flag)
			{
				if ((hydro_flag) && (Ylmn == 93))
					cpi = 3;
				if ((!hydro_flag) && (Ylmn == 22))
					cpi = 6;
				cpd = 0;
				Ylmn_flag = 0;
			}
			else
			{

				if (hydro2 > 2)
				{
					EEDATA = 93;
					socket_send("&*wY93#");
					hydro_flag = 1;
				}
				else
				{
					EEDATA = 22;
					socket_send("&*wY22#");
					hydro_flag = 0;
				}
			}
			break;
		case 3:
			HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
			if (cpd > 1)
			{
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				cpi = 4;
				cpd = 0;
			}
			cpf++;
			if (cpf > 23)
			{
				if ((hydro2 < hydro3 + 10) && (hydro2 > hydro3 - 10))
				{
					cpd++;
				}
				else
				{
					cpd = 0;
					hydro3 = hydro2;
				}
				cpf = 0;
			}

			break;
		case 4:
			HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
			cpd++;
			if (cpd == 19)
				hydro3 = hydro2;
			if (cpd > 20)
			{
				if ((hydro3 < 7500) && (hydro3 > 3750))
				{
					if (Ilmn_flag)
					{
						cpi = 5;
						hydroempty = hydro3;
						Ilmn_flag = 0;
					}
					else
					{
						EEkey[0] = 'I';
						EEDATA = hydro3;
						EEwr_func();
					}
				}
				else
				{
					socket_send("*#XE09XE09@$\r\n");
				}
			}
			break;
		case 5:
			if (Jlmn_flag)
			{
				cpi = 6;
				E21_flag = 0;
				E22_flag = 0;
				E23_flag = 0;
				mission_timer = 0;
				Jlmn_flag = 0;
			}
			else
			{
				EEkey[0] = 'J';
				EEDATA = hydro3 - 650; ////// level 2 (full)	2400Hz  ->  600
				hydrofull = EEDATA;
				EEwr_func();
			}
			break;
		case 6:
			mission_timer++;
			// if (mission_timer>2400) E23_flag=1;
			if (E22_flag)
			{
				HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				socket_send("*#XE25XE25@$\r\n");
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				socket_send("*#XE25XE25@$\r\n");
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				socket_send("*#XE25XE25@$\r\n");
			}
			else if (E23_flag)
			{
				HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				socket_send("*#XE26XE26@$\r\n");
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				socket_send("*#XE26XE26@$\r\n");
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				socket_send("*#XE26XE26@$\r\n");
			}
			else
			{
				HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_SET);
			}
			hydpin_flag = !HAL_GPIO_ReadPin(Hydro_GPIO_Port, Hydro_Pin);
			if (((hydro_flag) && (hydro2 < hydrofull)) || ((!hydro_flag) && (hydpin_flag)) || ((hydro2 < hydroempty - 200) && (!HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin))))
			{
				HAL_GPIO_WritePin(Valve1_GPIO_Port, Valve1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Valve2_GPIO_Port, Valve2_Pin, GPIO_PIN_RESET);
				cpd = 0;
				if ((!E21_flag) && (!E22_flag))
					cpi = 7;
				cpf = 0;
				E51_flag = 0;
				tccnt_flag = 0;
			}
			break;
		case 7:
			// sprintf(bufftest,"&*T2:%f,%d#",T2,cpf);
			// socket_send(bufftest);
			mission_timer = 0;
			adc_rd3 = 4096 - adc_buf[0];
			adc_rd3 = 10000 * adc_buf[0] / adc_rd3;
			T2 = (3950 / log(adc_rd3 / 0.01763)) - 273.15;
			if ((T2 > 90) || (T2 < 2))
			{
				socket_send("*#XE36XE36@$\r\n");
			}
			else if ((cpf > 16) && ((T2 < 24) || (26 < T2)))
			{
				adc_rd2 = adc_buf[1];
				HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
				motor_relay_request(MOTOR_RELAY_CMD_OFF);
				run_flag = 0;
				run_motor = 0;
				calBR();
				cpi = 8;
			}
			cpd++;
			if (cpd > 3)
			{
				if (cpf < 20)
					cpf++;
				cpd = 0;
			}
			if (cpi == 7)
			{
				HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
				if (cpf > 6)
				{
					run_flag = 1;
					run_motor = 1;
					motor_relay_request(MOTOR_RELAY_CMD_UI2);
					if ((!tccnt_flag) && (motor_speed < 600))
						motor_speed += 10;
				}
				else if (cpf > 5)
				{
					left_flag = 0;
					right_flag = 1;
					motor_relay_request(MOTOR_RELAY_CMD_UI2);
					run_flag = 0;
				}
			}
			break;
		case 8:
			HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
			motor_relay_request(MOTOR_RELAY_CMD_OFF);
			run_flag = 0;
			run_motor = 0;
			EEkey[0] = 'N';
			EEDATA = Rif;
			EEwr_func();
			if (Nlmn_flag)
			{
				cpi = 9;
				Nlmn_flag = 0;
			}
			break;
		case 9:
			EEkey[0] = 'O';
			EEDATA = Beta;
			EEwr_func();
			if (Olmn_flag)
			{
				cpi = 10;
				Olmn_flag = 0;
			}
			break;
		case 10:
			if (hydro_flag)
			{
				cpd++;
				HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_SET);
				if (cpd > 240)
				{
					HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
					cpi = 11;
					cpd = 0;
				}
			}
			else
			{
				cpi = 11;
			}
			break;
		case 11:
			HAL_GPIO_WritePin(Valve3_GPIO_Port, Valve3_Pin, GPIO_PIN_RESET);
			if ((hydro_flag) && (hydro2 + 50 < hydrofull))
			{
				socket_send("&*wD1#");
				EEDATA = 1;
				hwater_flag = 1;
			}
			else
			{
				socket_send("&*wD0#");
				hwater_flag = 0;
				EEDATA = 0;
			}
			if (Dlmn_flag)
			{
				cpi = 12;
				Dlmn_flag = 0;
			}
			break;
		case 12:
			if (!swdr_flag)
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			mission_timer++;
			if (mission_timer > 480)
			{
				mission_timer = 490;
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				socket_send("*#XE65XE65@$\r\n");
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
				socket_send("*#XE65XE65@$\r\n");
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(200);
				HAL_IWDG_Refresh(&hiwdg);
				socket_send("*#XE65XE65@$\r\n");
			}
			else
			{
				HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_SET);
			}
			hydpin_flag = !HAL_GPIO_ReadPin(Hydro_GPIO_Port, Hydro_Pin);
			if (hydpin_flag)
			{
				wef_flag = 1;
				wefcnt = 0;
			}
			wefcnt++;
			if (wefcnt > 520)
				wef_flag = 0;
			if (((hydro_flag) && (hydro2 > hydroempty - 100)) || ((!wef_flag) && (!hydro_flag)))
			{
				hydro_timer++;
			}
			else
			{
				hydro_timer = 0;
			}
			if (hydro_timer > 4)
				cpi = 13;
			if (mission_timer > 480)
				cpi = 12;
			break;
		case 13:
			HAL_GPIO_WritePin(Drain_Pomp_GPIO_Port, Drain_Pomp_Pin, GPIO_PIN_RESET);
			if (swdr_flag)
			{
				if (doorlock_flag)
				{
					chgdr_cnt++;
					if (chgdr_cnt > 11)
					{
						if (nba > 7)
						{
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(100);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
							HAL_Delay(100);
							HAL_IWDG_Refresh(&hiwdg);
							socket_send("*#XE14XE14@$\r\n");
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
							HAL_Delay(100);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
							HAL_Delay(100);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
							HAL_Delay(100);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
							HAL_Delay(100);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
							HAL_Delay(100);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
							socket_send("*#XE14XE14@$\r\n");
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							HAL_Delay(200);
							HAL_IWDG_Refresh(&hiwdg);
							socket_send("*#XE14XE14@$\r\n");
							nba = 9;
						}
						else if (nba > 3)
						{
							HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
						}
						else
						{
							HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
						}
						HAL_Delay(20);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
						chgdr_cnt = 0;
						nba++;
					}
				}
				else
				{
					chgdr_cnt = 11;
				}
			}
			else
			{
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
			}
			if (!doorlock_flag)
			{
				if (nba > 5)
				{
					socket_send("&*wZ95#");
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					socket_send("&*wZ95#");
					HAL_Delay(100);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
				}
				cpi = 14;
			}

			break;
		case 14:
			EEDATA = 64;
			socket_send("&*wG64#");
			if (Glmn_flag)
			{
				cpi = 15;
				Glmn = 64;
				Glmn_flag = 0;
			}
			break;
		case 15:
			if (swdr_flag)
			{
				if (doorlock_flag)
				{
					if ((swdr_flag) && (!Error_flag))
						master_relay_request(1);
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
					HAL_Delay(20);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
				}
				offcnt = 0;
				doorlock_cnt = 0;
				while ((doorlock_flag) && (offcnt < 1000))
				{
					if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
						doorlock_cnt = 0;
					doorlock_cnt++;
					if (doorlock_cnt > 200)
						doorlock_flag = 0;
					offcnt++;
					HAL_Delay(1);
					HAL_IWDG_Refresh(&hiwdg);
				}
				if (doorlock_flag)
				{
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_SET);
					HAL_Delay(20);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
				}
				while ((doorlock_flag) && (offcnt < 1000))
				{
					if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
						doorlock_cnt = 0;
					doorlock_cnt++;
					if (doorlock_cnt > 200)
						doorlock_flag = 0;
					offcnt++;
					HAL_Delay(1);
					HAL_IWDG_Refresh(&hiwdg);
				}
				if (doorlock_flag)
				{
					if ((swdr_flag) && (!Error_flag))
						master_relay_request(1);
					HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
					HAL_Delay(20);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
					offcnt = 0;
					doorlock_cnt = 0;
					while ((doorlock_flag) && (offcnt < 1000))
					{
						if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
							doorlock_cnt = 0;
						doorlock_cnt++;
						if (doorlock_cnt > 200)
							doorlock_flag = 0;
						offcnt++;
						HAL_Delay(1);
						HAL_IWDG_Refresh(&hiwdg);
					}
					if (doorlock_flag)
					{
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_SET);
						HAL_Delay(20);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_GPIO_WritePin(Door_Open_GPIO_Port, Door_Open_Pin, GPIO_PIN_RESET);
					}
					while ((doorlock_flag) && (offcnt < 1000))
					{
						if (HAL_GPIO_ReadPin(Freq_in_GPIO_Port, Freq_in_Pin))
							doorlock_cnt = 0;
						doorlock_cnt++;
						if (doorlock_cnt > 200)
							doorlock_flag = 0;
						offcnt++;
						HAL_Delay(1);
						HAL_IWDG_Refresh(&hiwdg);
					}
					if (!doorlock_flag)
					{
						socket_send("&*wZ95#");
						HAL_Delay(100);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						socket_send("&*wZ95#");
						HAL_Delay(100);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
						HAL_Delay(200);
						HAL_IWDG_Refresh(&hiwdg);
					}
				}
				socket_send("*#XXendXX@$\r\n");
				onwm_flag = 0;
			}
			else
			{
				offcnt = 0;
				socket_send("*#XXendXX@$\r\n");
				doorlock_cnt = 0;
				HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
				while ((doorlock_flag) && (offcnt < 60))
				{
					offcnt++;
					if (!HAL_GPIO_ReadPin(pushonkey_GPIO_Port, pushonkey_Pin))
						break;
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
					HAL_Delay(200);
					HAL_IWDG_Refresh(&hiwdg);
				}
				socket_send("*#XXendXX@$\r\n");
				onwm_flag = 0;
			}
			master_relay_request(0);
			break;
		}
	}
	if (!onwm_flag)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		socket_send("*#XXendXX@$\r\n");
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		socket_send("*#XXendXX@$\r\n");
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	}
	ms250_flag = 0;
}

/******************************************************************************/
void BeforeSleep()
{
	bsc++;
	if (swdr_flag)
	{
		if ((!doorlock_flag) || (bsc > 5) || (DnOpDr_flag))
		{
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			socket_send("*#XXendXX@$\r\n");
			end_flag = 1;
			onwm_flag = 0;
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			socket_send("*#XXendXX@$\r\n");
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(200);
			HAL_IWDG_Refresh(&hiwdg);
			socket_send("*#XXendXX@$\r\n");
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		}
		else if (doorlock_flag)
		{
			chgdr_cnt++;
			if (chgdr_cnt > 11)
			{
				dlcntt++;
				if (dlcntt < 4)
				{
					chgdr_flag = 1;
				}
				else
				{
					dlcntt = 0;
				}
				chgdr_cnt = 0;
			}
		}
		else
		{
			chgdr_cnt = 11;
		}
	}
	else
	{
		if (!DnOpDr_flag)
			HAL_GPIO_WritePin(Door_Lock_GPIO_Port, Door_Lock_Pin, GPIO_PIN_RESET);
	}
	if ((!doorlock_flag) || (bsc > 60))
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		socket_send("*#XXendXX@$\r\n");
		end_flag = 1;
		onwm_flag = 0;
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		socket_send("*#XXendXX@$\r\n");
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		socket_send("*#XXendXX@$\r\n");
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	}
}

/******************************************************************************/
void calT()
{
	adc_rd3 = 4096 - adc_rd2;
	adc_rd3 = 10000 * adc_rd2 / adc_rd3;
	float rif = (float)Rif / 100000;
	T = (Beta / log(adc_rd3 / rif)) - 273.15;
}

/******************************************************************************/
void calBR()
{
	int r0Array[14] = {5000, 6800, 10000, 15000, 20000, 23000, 25000, 30000, 33000, 40000, 47000, 50000, 68000, 100000};
	float t;
	float delta;
	float minDelta = 100000000;
	int minDeltaInd = 0, minBeta = 0;
	int rr = 5000;
	float rif;
	float mrmr = 0;
	for (int iii = 0; iii < 14; iii++)
	{
		for (int jjj = 0; jjj < 200; jjj++)
		{
			Beta = 3000 + jjj * 10;
			mrmr = Beta / -298.15;
			rif = r0Array[iii] * exp(mrmr);
			adc_rd3 = 4096 - adc_rd2;
			adc_rd3 = 10000 * adc_rd2 / adc_rd3;
			t = (Beta / log(adc_rd3 / rif)) - 273.15;
			if (t > T2)
			{
				delta = t - T2;
			}
			else
			{
				delta = T2 - t;
			}
			if (delta < minDelta)
			{
				minDelta = delta;
				minDeltaInd = iii;
				minBeta = Beta;
			}
		}
	}

	if (minDelta > 1.0)
	{
		if (minDeltaInd > 0)
			rr = r0Array[minDeltaInd - 1];
		while (rr < r0Array[minDeltaInd + 1])
		{
			rr = rr + 100;
			HAL_IWDG_Refresh(&hiwdg);
			for (int jjj = 0; jjj < 200; jjj++)
			{
				Beta = 3000 + jjj * 10;

				mrmr = Beta / -298.15;
				rif = rr * exp(mrmr);
				adc_rd3 = 4096 - adc_rd2;
				adc_rd3 = 10000 * adc_rd2 / adc_rd3;
				t = (Beta / (log(adc_rd3 / rif))) - 273.15;
				if (t > T2)
				{
					delta = t - T2;
				}
				else
				{
					delta = T2 - t;
				}
				if (delta < minDelta)
				{
					minDelta = delta;
					minBeta = Beta;
				}
			}
		}
	}
	Beta = minBeta;
	mrmr = Beta / -298.15;

	rif = r0Array[minDeltaInd] * exp(mrmr);
	Rif = 100000 * rif;
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4)
	{
		HAL_IncTick();
		onems_flag = 1;
		nrcnt++;
		hcnt++;
		tm1++;
		if (tm1 > 249)
		{
			ms250_flag = 1;
			tm1 = 0;
		}
		tm2++;
		if (tm2 > 999)
		{
			ms1000_flag = 1;
			tm2 = 0;
		}
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM3) //	5us
	{
		if (trig_flag)
			x++;
		if (trig_flag2)
			x22++;
		if (motor_speed < 2000)
		{
			d = 2001 - motor_speed;
			if (x > d)
			{
				c = 0;
				if ((run_flag) && (run_motor) && (allow_relay) && (motor_relay_applied != MOTOR_RELAY_CMD_OFF))
					HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_SET);
				if (trig_flag2)
				{
					x = x22;
					trig_flag2 = 0;
				}
				else
				{
					trig_flag = 0;
					x = 0;
				}
				x22 = 0;
			}
		}
		c++;
		if (c > 20)
			HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin, GPIO_PIN_RESET);
	}
	/* USER CODE END Callback 1 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Freq_in_Pin)
	{
		if (nr3 < 3)
		{
			nr3++;
			if (nr3 > 1)
			{
				// Reset when nrcnt is outside the valid [9,11] window.
				if ((nrcnt < 9) || (nrcnt > 11))
					nr3 = 0;
			}
		}
		else if ((nrcnt > 8) && (nrcnt < 12))
		{
			if (trig_flag)
			{
				trig_flag2 = 1;
				x22 = 0;
			}
			else
			{
				trig_flag = 1;
				// HAL_GPIO_WritePin(Motor_GPIO_Port, Motor_Pin,GPIO_PIN_RESET);
				x = 0;
			}
		}
		else
		{
			//   HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		}
		nrcnt = 0;
	}
	if (GPIO_Pin == Hydro_Pin)
	{
		if (hcnt)
			hyd_cnt++;
	}
	if (GPIO_Pin == Taco_PLS_Pin)
	{
		turn_motor = 1;
		taco_stop_tmr = 0;
		E51_cnt = 0;
		tacosafe++;
		if ((tacosafe > 50) && (washst_flag))
		{
			run_motor = 0;
			motor_speed = motor_speed_start;
			first_turn_motor = 0;
			tacosafe = 0;
		}
		runmotorc_flag = 0;
		runmotorcnt = 0;
		if (run_motor)
			tc++;
		taco_cnt++;
		taco_cnt3++;
		if (tc > 3)
		{
			taco_stop_tmr = 0;
			if (!first_turn_motor)
			{
				motor_speed_slctL = motor_speed;
				motor_speed_slctR = motor_speed;
			}
			first_turn_motor = 1;
			if (!fourturn_flag)
				fourturn_cnt++;
			if (fourturn_cnt > 95)
			{
				fourturn_flag = 1;
			}
			tc_cnt++;
			if (tc_cnt > 23)
			{
				tccnt_flag = 1;
				turn_cnt++;
				tc_cnt = 0;
			}
			tc = 0;
		}
	}
	if (GPIO_Pin == pushonkey_Pin)
	{
		if (!onwm_flag)
			HAL_PWR_DisableSleepOnExit();
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (secondword_flag == 1 && firstword_flag == 1)
	{
		cb[counter] = rb[0];
		counter++;
	}
	if ((rb[0] == 'z') || (rb[0] == 'E')) ////////////////zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzEEEEEEEEEEEEEEE
	{
		if ((cb[0] == 'D') && (cb[1] == 'e') && (cb[2] == 'b') && (cb[3] == 'u') && (cb[4] == 'g'))
		{
			debug_show = 1;
		}
		if ((cb[0] == 'u') && (cb[1] == 'n') && (cb[2] == 'r') && (cb[3] == 'g'))
		{
			unreg_flag = 1;
		}
		if ((cb[0] == 'D') && (cb[1] == 'o') && (cb[2] == 'n') && (cb[3] == 'e'))
		{
			if ((ready_scnt > 1) || (washing_scnt > 1) || (rinsing_scnt > 1) || (spinning_scnt > 1) || (pausing_scnt > 1))
			{
				ready_scnt = 1;
				washing_scnt = 1;
				rinsing_scnt = 1;
				spinning_scnt = 1;
				pausing_scnt = 1;
			}
			else
			{
				resume_cnt = 1;
				pause_cnt = 1;
			}
		}
		if ((cb[0] == 'e') && (cb[1] == 'e') && (cb[2] == 'P'))
		{
			EEErase = 1;
		}
		if (((cb[0] == 'R') && (cb[1] == 'e') && (cb[2] == 'a') && (cb[3] == 'd') && (cb[4] == 'y')) || ((cb[0] == 'u') && (cb[1] == 'p') && (cb[2] == 'd') && (cb[3] == 'a') && (cb[4] == 't') && (cb[5] == 'e')))
		{
			if (!Error_flag)
			{
				if (program_select == 0)
				{
					if (!delay_start_flag)
						ready_scnt = 3;
				}
				else
				{
					if (!run_flag)
					{
						pausing_scnt = 5;
					}
					else if (washmode)
					{
						washing_scnt = 5;
					}
					else if (program_select == 7)
					{
						spinning_scnt = 5;
					}
					else
					{
						rinsing_scnt = 5;
					}
					if (run_flag)
					{
						resume_cnt = 3;
					}
					else
					{
						pause_cnt = 3;
					}
				}
			}
			tm3 = 0;
		}
		if ((cb[0] == 'w') && (cb[3] == 's') && (cb[5] == 'T') && (cb[7] == 't') && (cb[12] == 'p'))
		{
			if (!run_flag)
				buzzercnt = 3;
			if (reg_flag)
			{
				ttt = 0;
				num2 = 99;
				cb[99] = cb[1];
				str2num();
				ww = 10 * num2;
				num2 = 99;
				cb[99] = cb[2];
				str2num();
				ww = ww + num2;
				if (ww > 9)
				{
					error_cmd = 1;
					buzzercnt = 1;
				}
				num2 = 99;
				cb[99] = cb[4];
				str2num();
				if (num2 > 5)
				{
					error_cmd = 1;
					buzzercnt = 1;
				}
				else
				{
					ss = num2;
				}
				num2 = 99;
				cb[99] = cb[6];
				str2num();
				if (num2 > 4)
				{
					error_cmd = 1;
					buzzercnt = 1;
				}
				else
				{
					tt = num2;
				}
				num2 = 99;
				cb[99] = cb[8];
				str2num();
				ttt = 10 * num2;
				num2 = 99;
				cb[99] = cb[9];
				str2num();
				ttt = ttt + num2;
				ttt = ttt * 10;
				num2 = 99;
				cb[99] = cb[10];
				str2num();
				ttt = ttt + num2;
				ttt = ttt * 10;
				num2 = 99;
				cb[99] = cb[11];
				str2num();
				ttt = ttt + num2;
				num2 = 99;
				cb[99] = cb[13];
				str2num();
				pp = num2;
				woolmode = 0;
				delay_timer = ttt;
				delay_start_flag = 0;
				if ((!run_flag) && (ttt > 0))
					delay_start_flag = 1;
				procng_flag = 1;
			}
		}
		if ((cb[0] == 'r') && (cb[1] == 'e') && (cb[2] == 's') && (cb[3] == 'u') && (cb[4] == 'm') && (cb[5] == 'e'))
		{
			if ((!run_flag))
			{
				buzzercnt = 3;
				resume_flag = 1;
			}
		}
		if ((cb[0] == 'p') && (cb[1] == 'a') && (cb[2] == 'u') && (cb[3] == 's') && (cb[4] == 'e'))
		{
			if (run_flag)
			{
				buzzercnt = 2;
				pause_cnt = 3;
				pause_flag = 1;
			}
		}
		if ((cb[0] == 's') && (cb[1] == 't') && (cb[2] == 'o') && (cb[3] == 'p'))
		{
			buzzercnt = 2;
			stop_flag = 1;
		}
		if (Debug_flag)
		{
			if ((cb[0] == 'P') && (cb[1] == 'L'))
			{
				buzzercnt = 2;
				pl_flag = 1;
				cb[99] = cb[2];
				str2num();
				plm = 10 * num2;
				cb[99] = cb[3];
				str2num();
				plm = plm + num2;
				if ((plm < 20) || (plm > 50))
				{
					//  pl_flag=0;
					buzzercnt = 1;
				}
			}
			if ((cb[0] == 'T') && (cb[1] == 'e') && (cb[2] == 's') && (cb[3] == 't') && (!dmr_flag) && (!dml_flag) && (eef_flag) && (eee_flag))
			{
				test_flag = 1;
				Debug_flag = 0;
			}
			if ((cb[0] == 'M') && (cb[1] == 'L') && (cb[2] == 'o') && (cb[3] == 'n') && (!dmr_flag) && (!dml_flag))
			{
				dml_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'M') && (cb[1] == 'L') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (dml_flag))
			{
				dml_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'M') && (cb[1] == 'R') && (cb[2] == 'o') && (cb[3] == 'n') && (!dml_flag) && (!dmr_flag))
			{
				dmr_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'M') && (cb[1] == 'R') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (dmr_flag))
			{
				dmr_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'D') && (cb[1] == 'R') && (cb[2] == 'o') && (cb[3] == 'n') && (!ddr_flag))
			{
				ddr_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'H') && (cb[1] == 'Y') && (cb[2] == 'D') && (cb[3] == 'P'))
			{
				hydp_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'H') && (cb[1] == 'Y') && (cb[2] == 'D') && (cb[3] == 'E'))
			{
				hyde_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'S') && (cb[1] == 'W') && (cb[2] == 'D') && (cb[3] == 'R'))
			{
				drt_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'S') && (cb[1] == 'W') && (cb[2] == '2') && (cb[3] == 'R'))
			{
				dr2_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'S') && (cb[1] == 'W') && (cb[2] == 'D') && (cb[3] == 'F'))
			{
				drm_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'N') && (cb[1] == 'O') && (cb[2] == 'S') && (cb[3] == 'E'))
			{
				nosensor_flag = 1;
				dnose_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'W') && (cb[1] == 'T') && (cb[2] == 'S') && (cb[3] == 'E'))
			{
				nosensor_flag = 0;
				dnose_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'D') && (cb[1] == 'R') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (ddr_flag))
			{
				ddr_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'V') && (cb[1] == '1') && (cb[2] == 'o') && (cb[3] == 'n') && (!dv1_flag))
			{
				dv1_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'V') && (cb[1] == '1') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (dv1_flag))
			{
				dv1_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'V') && (cb[1] == '2') && (cb[2] == 'o') && (cb[3] == 'n') && (!dv2_flag))
			{
				dv2_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'V') && (cb[1] == '2') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (dv2_flag))
			{
				dv2_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'V') && (cb[1] == '3') && (cb[2] == 'o') && (cb[3] == 'n') && (!dv3_flag))
			{
				dv3_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'V') && (cb[1] == '3') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (dv3_flag))
			{
				dv3_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'D') && (cb[1] == 'L') && (cb[2] == 'o') && (cb[3] == 'n') && (!ddl_flag))
			{
				ddl_flag = 1;
				buzzercnt = 2;
				showfb_flag = 1;
			}
			if ((cb[0] == 'D') && (cb[1] == 'L') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (ddl_flag))
			{
				ddl_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'H') && (cb[1] == 'R') && (cb[2] == 'o') && (cb[3] == 'n') && (!dhr_flag))
			{
				dhr_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'H') && (cb[1] == 'R') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f') && (dhr_flag))
			{
				dhr_flag = 0;
				buzzercnt = 2;
			}
			if ((cb[0] == 'H') && (cb[1] == 'W') && (cb[2] == 'o') && (cb[3] == 'n'))
			{
				tmhw_flag = 1;
				dhw_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'H') && (cb[1] == 'W') && (cb[2] == 'o') && (cb[3] == 'f') && (cb[4] == 'f'))
			{
				tmhw_flag = 0;
				dhw_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'R') && (cb[1] == 'i') && (cb[7] == 'B'))
			{
				Rif = ((cb[2] - 0x30) * 10000) + ((cb[3] - 0x30) * 1000) + ((cb[4] - 0x30) * 100) + ((cb[5] - 0x30) * 10) + (cb[6] - 0x30);
				Beta = ((cb[8] - 0x30) * 1000) + ((cb[9] - 0x30) * 100) + ((cb[10] - 0x30) * 10) + (cb[11] - 0x30);
				buzzercnt = 3;
				if (Rif != 0)
					dt30_flag = 1;
				notsave_flag = 0;
			}
			if ((cb[0] == 'W') && (cb[1] == 'L') && (cb[2] == '1') && (!dwl1_flag) && (hydro_flag))
			{
				dwl1_flag = 1;
				buzzercnt = 2;
			}
			if ((cb[0] == 'W') && (cb[1] == 'L') && (cb[2] == '2') && (!dwl2_flag) && (hydro_flag))
			{
				dwl2_flag = 1;
				buzzercnt = 2;
			}
		}
		secondword_flag = 0;
		firstword_flag = 0;
		memset(cb, '\0', 100);
	}
	if (secondwordE_flag == 1 && firstwordE_flag == 1)
	{
		cb[counter] = rb[0];
		counter++;
		if (rb[0] == '#')
		{
			rr = 0;
			if ((fto_flag) && (cb[0] == 'A') && (cb[1] == '0'))
			{
				verify_flag = 1;
				norx_flag = 0;
			}

			if ((cb[0] == 'e') && (cb[1] == 'a'))
				ea_flag = 1;

			for (int ll = 0; ll < 100; ll++)
			{
				if ((cb[ll] == 'D') || (cb[ll] == 'F') || (cb[ll] == 'G') || (cb[ll] == 'H') || (cb[ll] == 'I') || (cb[ll] == 'J') || (cb[ll] == 'K') || (cb[ll] == 'L') || (cb[ll] == 'M') || (cb[ll] == 'N') || (cb[ll] == 'O') || (cb[ll] == 'P') || (cb[ll] == 'Q') || (cb[ll] == 'R') || (cb[ll] == 'S') || (cb[ll] == 'T') || (cb[ll] == 'U') || (cb[ll] == 'V') || (cb[ll] == 'W') || (cb[ll] == 'X') || (cb[ll] == 'Y') || (cb[ll] == 'Z'))
				{
					mm[rr] = ll;
					rr++;
				}
			}
			mm[rr] = counter - 1;
			for (int ll = 0; ll < rr; ll++)
			{
				CC = mm[ll];
				num3 = 0;
				for (int aa = CC + 1; aa < mm[ll + 1]; aa++)
				{
					cb[99] = cb[aa];
					str2num();
					if (err_num)
						aa = mm[ll + 1];
					num3 = num2 + num3 * 10;
				}
				if ((CC + 1 < mm[ll + 1]) && (!err_num))
				{
					if (cb[CC] == 'D')
					{
						Dlmn = num3;
						if (Dlmn == 1)	hwater_flag = 1;
						if (EEDATA == num3)
							Dlmn_flag = 1;
					}
					if (cb[CC] == 'F')
					{
						Flmn = num3;
						if (EEDATA == num3)
							Flmn_flag = 1;
					}
					if (cb[CC] == 'G')
					{
						Glmn = num3;
						if (EEDATA == num3)
							Glmn_flag = 1;
						if (fto_flag)
							Glmn_flag = 1;
						if (Glmn == 64)
							G2lmn_flag = 1;
					}
					if (cb[CC] == 'H')
					{
						Hlmn = num3;
						if (EEDATA == num3)
							Hlmn_flag = 1;
						if (fto_flag)
							Hlmn_flag = 1;
					}
					if (cb[CC] == 'I')
					{
						Ilmn = num3;
						if (EEDATA == num3)
							Ilmn_flag = 1;
						if (fto_flag)
							Ilmn_flag = 1;
					}
					if (cb[CC] == 'J')
					{
						Jlmn = num3;
						if (EEDATA == num3)
							Jlmn_flag = 1;
						if (fto_flag)
							Jlmn_flag = 1;
					}
					if (cb[CC] == 'K')
					{
						Klmn = num3;
						if (EEDATA == num3)
							Klmn_flag = 1;
					}
					if (cb[CC] == 'L')
					{
						Llmn = num3;
						if (EEDATA == num3)
							Llmn_flag = 1;
					}
					if (cb[CC] == 'M')
					{
						Mlmn = num3;
						if (EEDATA == num3)
							Mlmn_flag = 1;
					}
					if (cb[CC] == 'N')
					{
						Nlmn = num3;
						if (EEDATA == num3)
							Nlmn_flag = 1;
					}
					if (cb[CC] == 'O')
					{
						Olmn = num3;
						if (EEDATA2 == num3)
							Olmn_flag = 1;
					}
					if (cb[CC] == 'P')
					{
						Plmn = num3;
						if (EEDATA == num3)
							Plmn_flag = 1;
					}
					if (cb[CC] == 'Q')
					{
						Qlmn = num3;
						if (EEDATA == num3)
							Qlmn_flag = 1;
					}
					if (cb[CC] == 'R')
					{
						Rlmn = num3;
						if (EEDATA == num3)
							Rlmn_flag = 1;
					}
					if (cb[CC] == 'S')
					{
						Slmn = num3;
						if (EEDATA == num3)
							Slmn_flag = 1;
					}
					if (cb[CC] == 'T')
					{
						Tlmn = num3;
						if (EEDATA == num3)
							Tlmn_flag = 1;
					}
					if (cb[CC] == 'U')
					{
						Ulmn = num3;
						if (EEDATA == num3)
							Ulmn_flag = 1;
					}
					if (cb[CC] == 'V')
					{
						Vlmn = num3;
						if (EEDATA == num3)
							Vlmn_flag = 1;
					}
					if (cb[CC] == 'W')
					{
						Wlmn = num3;
						if (EEDATA == num3)
							Wlmn_flag = 1;
					}
					if (cb[CC] == 'Y')
					{
						Ylmn = num3;
						if (EEDATA == num3)
							Ylmn_flag = 1;
					}
					if (cb[CC] == 'Z')
					{
						Zlmn = num3;
						if (EEDATA == num3)
							Zlmn_flag = 1;
					}
				}
			}

			if (fto_flag)
			{
				vfc++;
				if (vfc > 4)
				{
					verify_flag = 1;
					norx_flag = 0;
				}
				if ((Glmn_flag) && (!G2lmn_flag))
				{
					verify_flag = 1;
					norx_flag = 0;
				}
				if ((G2lmn_flag) && (Ilmn_flag) && (Jlmn_flag))
				{
					verify_flag = 1;
					norx_flag = 0;
				}
			}
			secondwordE_flag = 0;
			firstwordE_flag = 0;
			memset(cb, '\0', 100);
		}
	}
	if (rb[0] == 'A')
	{
		firstword_flag = 1;
		secondword_flag = 0;
		cb[1] = '0';
	}
	if ((rb[0] == 'B') && (cb[1] != 'i'))
	{
		secondword_flag = 1;
		counter = 0;
		firstwordE_flag = 0;
		secondwordE_flag = 0;
		memset(cb, '\0', 100);
	}
	if (rb[0] == '*')
	{
		firstwordE_flag = 1;
		secondwordE_flag = 0;
	}
	if (rb[0] == '&')
	{
		secondwordE_flag = 1;
		counter = 0;
		firstword_flag = 0;
		secondword_flag = 0;
		memset(cb, '\0', 100);
	}
}

void str2num()
{
	err_num = 0;
	if (cb[99] == '0')
		num2 = 0;
	else if (cb[99] == '1')
		num2 = 1;
	else if (cb[99] == '2')
		num2 = 2;
	else if (cb[99] == '3')
		num2 = 3;
	else if (cb[99] == '4')
		num2 = 4;
	else if (cb[99] == '5')
		num2 = 5;
	else if (cb[99] == '6')
		num2 = 6;
	else if (cb[99] == '7')
		num2 = 7;
	else if (cb[99] == '8')
		num2 = 8;
	else if (cb[99] == '9')
		num2 = 9;
	else
		err_num = 1;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

