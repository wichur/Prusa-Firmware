//mmu.cpp

#include "mmu.h"
#include "planner.h"
#include "language.h"
#include "lcd.h"
#include "uart2.h"
#include "temperature.h"
#include "Configuration_prusa.h"
#include "fsensor.h"
#include "cardreader.h"
#include "ultralcd.h"
#include "sound.h"
#include "printers.h"
#include <avr/pgmspace.h>

#ifdef TMC2130
#include "tmc2130.h"
#endif //TMC2130

#define CHECK_FINDA ((IS_SD_PRINTING || is_usb_printing) && (mcode_in_progress != 600) && !saved_printing && e_active())

#define MMU_TODELAY 100
#define MMU_TIMEOUT 10
#define MMU_CMD_TIMEOUT 300000ul  //300000ul //5min timeout for mmu commands (except P0)
#define MMU_P0_TIMEOUT 3000ul //timeout for P0 command: 3seconds
#define OCTO_NOTIFICATIONS_ON

#ifdef MMU_HWRESET
#define MMU_RST_PIN 76
#endif //MMU_HWRESET

bool mmu_enabled = false;
bool mmu_ready = false;
bool last_conf = false; // For comms debug
bool isMMUPrintPaused = false;
bool mmuFSensorLoading = false;
int lastLoadedFilament = -10;
int8_t mmu_state = 0;
int8_t last_state = -10;
uint8_t mmu_cmd = 0;
int toolChanges = 0;
bool ack_received = false;
uint8_t mmu_extruder = 0;

//! This variable probably has no meaning and is planed to be removed
uint8_t tmp_extruder = 0;
bool mmu_finda = false;
int16_t mmu_version = -1;
int16_t mmu_buildnr = -1;
uint32_t mmu_last_request = 0;
uint32_t mmu_last_response = 0;

//initialize mmu2 unit - first part - should be done at begining of startup process
void mmu_init(void)
{
#ifdef MMU_HWRESET
    digitalWrite(MMU_RST_PIN, HIGH);
    pinMode(MMU_RST_PIN, OUTPUT);              //setup reset pin
#endif //MMU_HWRESET
    uart2_init();                              //init uart2
    _delay_ms(10);                             //wait 10ms for sure
    mmu_reset();                               //reset mmu (HW or SW), do not wait for response
    mmu_state = -1;
}

//mmu main loop - state machine processing
/**
 * MMU States
 * -1 >> -2 MMURX Start, respond with S1 (30s timeout to disabled state)
 * -2 >> -3 MMURX ok, respond with S2
 * -3 >> -4 MMURX ok, respond with P0(READ FINDA) if MK3 and goto -4
 *       -5           respond with M1(MODE STEALTH) if MK2.5 and goto -5
 * -4 >>  1 MMURX ok, mmu_ready
 * -5 >>  1 MMURX ok, mmu_ready
 //## DEPRICATED##//*  1 >> 10 MMU CMD Request from MK3
 *  1 >>  3 MMU CMD Request from MK3
 *  2 >>  1 MMURX ok, Finda State
 *  3 >>  1 MMURX ok, mmu commands response
 //## DEPRICATED##//* 10 >>  3 MMUECHO, confirm receipt of cmd
 */

void mmu_loop(void)
{
    int filament = 0;
#ifdef MMU_DEBUG
    if ((last_state != mmu_state) || (last_conf != confirmedPayload)) printf_P(PSTR("MMU loop, state=%d CPL=%d\n"), mmu_state, confirmedPayload);
    last_conf = confirmedPayload; // for comms debug
    last_state = mmu_state;
#endif //MMU_DEBUG

    unsigned char tData1 = rxData1;                  // Copy volitale vars as local
    unsigned char tData2 = rxData2;
    unsigned char tData3 = rxData3;
    unsigned char  tCSUM =  rxCSUM;
    bool     confPayload = confirmedPayload;
    if (txRESEND) {
        txRESEND         = false;
        confirmedPayload = false;
        startRxFlag      = false;
        uart2_txPayload(lastTxPayload);
        return;
    }
    if ((confPayload && !(tCSUM == 0x2D)) || txNAKNext) { // If confirmed with bad CSUM or NACK return has been requested
        confirmedPayload = false;
        startRxFlag      = false;
        uart2_txACK(false); // Send NACK Byte
#ifdef MMU_DEBUG
        puts_P(PSTR("Non-Conf Payload"));
#endif //MMU_DEBUG
    } else if (confPayload) {
        confirmedPayload = false;
        startRxFlag      = false;
        uart2_txACK();      // Send  ACK Byte
    }
    mmu_last_response = millis(); // Update last response counter      
#ifdef MMU_DEBUG
          printf_P(PSTR("MMU Conf Payload, state=%d\n"), mmu_state);
#endif //MMU_DEBUG
    if (mmu_state == -1) {
          if (((tData1 == 'S') && (tData2 == 'T') && (tData3 == 'R')) && (confPayload)) {
#ifdef MMU_DEBUG
              puts_P(PSTR("MMU => MK3 'start'"));
              puts_P(PSTR("MK3 => MMU 'S1'"));
#endif //MMU_DEBUG
              uart2_txPayload("S1-");
              mmu_state = -2;
          } else if (millis() > 30000) { //30sec after reset disable mmu
              puts_P(PSTR("MMU not responding - DISABLED"));
              mmu_state = 0;
          } // End of if STR
    } else if ((mmu_state == -2) && (confPayload)) {
          mmu_version = ((tData1 << 8) | (tData2));
          printf_P(PSTR("MMU => MK3 '%dok'\n"), mmu_version);
          puts_P(PSTR("MK3 => MMU 'S2'"));
          uart2_txPayload("S2-");
          mmu_state = -3;
    } else if ((mmu_state == -3) && (confPayload)) {
          mmu_buildnr = ((tData1 << 8) | (tData2));
          printf_P(PSTR("MMU => MK3 '%dok'\n"), mmu_buildnr);
          bool version_valid = mmu_check_version();
          if (!version_valid) mmu_show_warning();
          else puts_P(PSTR("MMU version valid"));
          if ((PRINTER_TYPE == PRINTER_MK3) || (PRINTER_TYPE == PRINTER_MK3_SNMM)) {
#ifdef MMU_DEBUG
              puts_P(PSTR("MK3 => MMU 'P0'"));
#endif //MMU_DEBUG
              uart2_txPayload("P0-");
              mmu_state = -4;
          } else {
#ifdef MMU_DEBUG
              puts_P(PSTR("MK3 => MMU 'M1'"));
#endif //MMU_DEBUG
              uart2_txPayload("M1-");
              mmu_state = -5;
          }
      } else if ((mmu_state == -4) && (confPayload)) {
          mmu_finda = (tData3 != 0x00) ? true : false;
#ifdef MMU_DEBUG
          printf_P(PSTR("MMU => MK3 '%dok'\n"), mmu_finda);
#endif //MMU_DEBUG
          puts_P(PSTR("MMU - ENABLED"));
          fsensor_enable();
          mmu_enabled = true;
          mmu_state = 1;
      } else if ((mmu_state == -5) && (confPayload)) {
#ifdef MMU_DEBUG
          puts_P(PSTR("MMU => MK3 'P0'"));
#endif //MMU_DEBUG
          uart2_txPayload("P0-");
          mmu_state = -4;
      } else if (mmu_state == 1) {
          if (mmu_cmd) {
              if ((mmu_cmd >= MMU_CMD_T0) && (mmu_cmd <= MMU_CMD_T4))
              {
                  filament = mmu_cmd - MMU_CMD_T0;
                  if (lastLoadedFilament != filament) {
                      toolChanges++;
                      printf_P(PSTR("MK3 => MMU 'T%d @toolChange:%d'\n"), filament, toolChanges);
                      unsigned char tempTxCMD[3] = {'T', filament, BLK};
                      uart2_txPayload(tempTxCMD);
                      mmu_state = 3; // wait for response
                  } else {
                      unsigned char tempTxCMD[3] = {'T', filament, BLK};
                      uart2_txPayload(tempTxCMD);
                      mmu_state = 3;
                  }
                  lastLoadedFilament = filament;
              } else if ((mmu_cmd >= MMU_CMD_L0) && (mmu_cmd <= MMU_CMD_L4))
              {
                  filament = mmu_cmd - MMU_CMD_L0;
                  printf_P(PSTR("MK3 => MMU 'L%d'\n"), filament);
                  unsigned char tempLxCMD[3] = {'L', filament, BLK};
                  uart2_txPayload(tempLxCMD);
                  mmu_state = 3; // wait for response
              } else if (mmu_cmd == MMU_CMD_C0)
              {
                  printf_P(PSTR("MK3 => MMU 'C0'\n"));
                  uart2_txPayload("C0-");
                  mmu_state = 3;
              } else if (mmu_cmd == MMU_CMD_U0)
              {
                  printf_P(PSTR("MK3 => MMU 'U0'\n"));
                  uart2_txPayload("U0-");
                  toolChanges = 0;
                  lastLoadedFilament = -10;
                  mmu_state = 3;
              } else if ((mmu_cmd >= MMU_CMD_E0) && (mmu_cmd <= MMU_CMD_E4))
              {
                  filament = mmu_cmd - MMU_CMD_E0;
                  printf_P(PSTR("MK3 => MMU 'E%d'\n"), filament);
                  unsigned char tempExCMD[3] = {'E', filament, BLK};
                  uart2_txPayload(tempExCMD);
                  mmu_state = 3; // wait for response
              } else if (mmu_cmd == MMU_CMD_R0)
              {
                  printf_P(PSTR("MK3 => MMU 'R0'\n"));
                  uart2_txPayload("R0-");
                  mmu_state = 3; // wait for response
              } else if (mmu_cmd == MMU_CMD_FS)
              {
                  printf_P(PSTR("MK3 => MMU 'Filament seen at extruder'\n"));
                  uart2_txPayload("FS-");
                  mmu_state = 3; // wait for response
              }
              mmu_cmd = 0;
          } else if (((mmu_last_response + 500) < millis()) && !mmuFSensorLoading) { //request every 500ms
              uart2_txPayload("P0-");
              mmu_state = 2;
          } else if (((mmu_last_response + 500) < millis()) && mmuFSensorLoading) {
              if (!fsensor_enabled) fsensor_enable();
          } // end of if mmu_cmd
      } else if (mmu_state == 2) {
          mmu_finda = (0xFF & tData3);
          if (!mmu_finda && CHECK_FINDA && fsensor_enabled) {
              fsensor_stop_and_save_print();
              enquecommand_front_P(PSTR("FSENSOR_RECOVER")); //then recover
              if (lcd_autoDepleteEnabled()) enquecommand_front_P(PSTR("M600 AUTO")); //save print and run M600 command
              else enquecommand_front_P(PSTR("M600")); //save print and run M600 command
          }
          mmu_state = 1;
          if ((mmu_last_request + MMU_P0_TIMEOUT) < millis())
          {   //resend request after timeout (30s)
              mmu_state = 1;
          }
      } else if ((mmu_state == 3) && (confPayload)) {
          if ((tData1 == 'F') && (tData2 == 'S')) {
              printf_P(PSTR("MMU => MK3 'waiting for filament @ MK3 Sensor'\n"));
              if (!fsensor_enabled) fsensor_enable();
              mmuFSensorLoading = true;
              fsensor_autoload_enabled = true;
              fsensor_autoload_check_stop();
              mmu_state = 1;
          } else if ((tData1 == 'O') && (tData2 == 'K')) {
              if (mmuFSensorLoading == true) {
                  mmuFSensorLoading = false;
                  printf_P(PSTR("MMU => MK3 'ok :)'\n"));
              }
              printf_P(PSTR("MMU => MK3 'ok'\n"));
              mmu_ready = true;
              mmu_state = 1;
          }
    } // End of mmu_state if statement
}


void mmu_reset(void)
{
#ifdef MMU_HWRESET                             //HW - pulse reset pin
    digitalWrite(MMU_RST_PIN, LOW);
    _delay_us(100);
    digitalWrite(MMU_RST_PIN, HIGH);
#else                                          //SW - send X0 command
    // TODO:RMM
    //mmu_puts_P(PSTR("X0\n"));
#endif
}

int8_t mmu_set_filament_type(uint8_t extruder, uint8_t filament)
{
    printf_P(PSTR("MMU <= 'F%d %d'\n"), extruder, filament);
    char tempFx[3] = {'F', extruder, filament};
    uart2_txPayload(tempFx);
}

void mmu_command(uint8_t cmd)
{
    mmu_cmd = cmd;
    mmu_ready = false;
}

bool mmu_get_response(void)
{
    KEEPALIVE_STATE(IN_PROCESS);
    while (mmu_cmd != 0)
    {
        delay_keep_alive(100);
    }
    while (!mmu_ready)
    {
        if (((mmu_state == 3) || (mmu_state == 10) || (mmuFSensorLoading)) && ((mmu_last_request + MMU_CMD_TIMEOUT) > millis())) {
            delay_keep_alive(100);
        } else {
            break;
        }
    }
    bool ret = mmu_ready;
    mmu_ready = false;
    return ret;
}

void manage_response(bool move_axes, bool turn_off_nozzle)
{
    bool response = false;
    mmu_print_saved = false;
    bool lcd_update_was_enabled = false;
    float hotend_temp_bckp = degTargetHotend(active_extruder);
    float z_position_bckp = current_position[Z_AXIS];
    float x_position_bckp = current_position[X_AXIS];
    float y_position_bckp = current_position[Y_AXIS];
    uint8_t screen = 0; //used for showing multiscreen messages
    while(!response)
    {
        response = mmu_get_response(); //wait for "ok" from mmu
        if (!response) { //no "ok" was received in reserved time frame, user will fix the issue on mmu unit
            if (!mmu_print_saved) { //first occurence, we are saving current position, park print head in certain position and disable nozzle heater
                if (lcd_update_enabled) {
                    lcd_update_was_enabled = true;
                    lcd_update_enable(false);
                }
                st_synchronize();
                mmu_print_saved = true;
#ifdef OCTO_NOTIFICATIONS_ON
                printf_P(PSTR("// action:mmuAttention\n"));
#endif // OCTO_NOTIFICATIONS_ON
                printf_P(PSTR("MMU not responding\n"));
                hotend_temp_bckp = degTargetHotend(active_extruder);
                if (move_axes) {
                    z_position_bckp = current_position[Z_AXIS];
                    x_position_bckp = current_position[X_AXIS];
                    y_position_bckp = current_position[Y_AXIS];

                    //lift z
                    current_position[Z_AXIS] += Z_PAUSE_LIFT;
                    if (current_position[Z_AXIS] > Z_MAX_POS) current_position[Z_AXIS] = Z_MAX_POS;
                    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, active_extruder);
                    st_synchronize();

                    //Move XY to side
                    current_position[X_AXIS] = X_PAUSE_POS;
                    current_position[Y_AXIS] = Y_PAUSE_POS;
                    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);
                    st_synchronize();
                }
                if (turn_off_nozzle) {
                    //set nozzle target temperature to 0
                    setAllTargetHotends(0);
                }
            }

            //first three lines are used for printing multiscreen message; last line contains measured and target nozzle temperature
            if (screen == 0) { //screen 0
                lcd_display_message_fullscreen_P(_i("MMU needs user attention."));
                screen++;
            }
            else if (screen == 1) {  //screen 1
                if((degTargetHotend(active_extruder) == 0) && turn_off_nozzle) lcd_display_message_fullscreen_P(_i("Press the knob to resume nozzle temperature."));
                else lcd_display_message_fullscreen_P(_i("Fix the issue and then press button on MMU unit."));
                screen = 0;
            }

            lcd_set_degree();
            lcd_set_cursor(0, 4); //line 4
            //Print the hotend temperature (9 chars total) and fill rest of the line with space
            int chars = lcd_printf_P(_N("%c%3d/%d%c"), LCD_STR_THERMOMETER[0],(int)(degHotend(active_extruder) + 0.5), (int)(degTargetHotend(active_extruder) + 0.5), LCD_STR_DEGREE[0]);
            lcd_space(9 - chars);


            //5 seconds delay
            for (uint8_t i = 0; i < 50; i++) {
                if (lcd_clicked()) {
                    setTargetHotend(hotend_temp_bckp, active_extruder);
                    if (mmuFSensorLoading) {
                        if (!fsensor_enabled) fsensor_enable();
                        if (!fsensor_autoload_enabled) fsensor_autoload_enabled  = true;
                        fsensor_autoload_check_stop();
                    }
                    break;
                }
                delay_keep_alive(100);
            }
        }
        else if (mmu_print_saved) {
            printf_P(PSTR("MMU starts responding\n"));
            if (turn_off_nozzle)
            {
                lcd_clear();
                setTargetHotend(hotend_temp_bckp, active_extruder);
                if (((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)) {
                    lcd_display_message_fullscreen_P(_i("MMU OK. Resuming temperature..."));
                    delay_keep_alive(3000);
                }
                while ((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)
                {
                    delay_keep_alive(1000);
                    lcd_wait_for_heater();
                }
            }
            if (move_axes) {
                lcd_clear();
                lcd_display_message_fullscreen_P(_i("MMU OK. Resuming position..."));
                current_position[X_AXIS] = x_position_bckp;
                current_position[Y_AXIS] = y_position_bckp;
                plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 50, active_extruder);
                st_synchronize();
                current_position[Z_AXIS] = z_position_bckp;
                plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, active_extruder);
                st_synchronize();
            } else {
                lcd_clear();
                lcd_display_message_fullscreen_P(_i("MMU OK. Resuming..."));
                delay_keep_alive(1000); //delay just for showing MMU OK message for a while in case that there are no xyz movements
            }
        }
    }
    if (lcd_update_was_enabled) lcd_update_enable(true);
#ifdef TMC2130
    //enable extruder motor (disabled in mmu_command, start of T-code processing)
    tmc2130_set_pwr(E_AXIS, 1);
    //printf_P(PSTR("E-axis enabled\n"));
#endif //TMC2130
}

//! @brief load filament to nozzle of multimaterial printer
//!
//! This function is used only only after T? (user select filament) and M600 (change filament).
//! It is not used after T0 .. T4 command (select filament), in such case, gcode is responsible for loading
//! filament to nozzle.
//!
void mmu_load_to_nozzle()
{
    st_synchronize();

    bool saved_e_relative_mode = axis_relative_modes[E_AXIS];
    if (!saved_e_relative_mode) axis_relative_modes[E_AXIS] = true;
    current_position[E_AXIS] += 7.2f;
    float feedrate = 562;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
    st_synchronize();
    current_position[E_AXIS] += 14.4f;
    feedrate = 871;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
    st_synchronize();
    current_position[E_AXIS] += 36.0f;
    feedrate = 1393;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
    st_synchronize();
    current_position[E_AXIS] += 14.4f;
    feedrate = 871;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
    st_synchronize();
    if (!saved_e_relative_mode) axis_relative_modes[E_AXIS] = false;
}

void mmu_M600_wait_and_beep() {
    //Beep and wait for user to remove old filament and prepare new filament for load

    KEEPALIVE_STATE(PAUSED_FOR_USER);
#ifdef OCTO_NOTIFICATIONS_ON
    printf_P(PSTR("// action:m600\n"));
#endif // OCTO_NOTIFICATIONS_ON

    int counterBeep = 0;
    lcd_display_message_fullscreen_P(_i("Remove old filament and press the knob to start loading new filament."));
    bool bFirst=true;

    while (!lcd_clicked()) {
        manage_heater();
        manage_inactivity(true);

#if BEEPER > 0
        if (counterBeep == 500) {
            counterBeep = 0;
        }
        SET_OUTPUT(BEEPER);
        if (counterBeep == 0) {
            if((eSoundMode==e_SOUND_MODE_LOUD)||((eSoundMode==e_SOUND_MODE_ONCE)&&bFirst))
            {
                bFirst=false;
                WRITE(BEEPER, HIGH);
            }
        }
        if (counterBeep == 20) {
            WRITE(BEEPER, LOW);
        }

        counterBeep++;
#endif //BEEPER > 0

        delay_keep_alive(4);
    }
    WRITE(BEEPER, LOW);
}

void mmu_M600_load_filament(bool automatic)
{
    //load filament for mmu v2
    tmp_extruder = mmu_extruder;
    lastLoadedFilament = -10;
    if (!automatic) {
#ifdef MMU_M600_SWITCH_EXTRUDER
        bool yes = lcd_show_fullscreen_message_yes_no_and_wait_P(_i("Do you want to switch extruder?"), false);
        if(yes) tmp_extruder = choose_extruder_menu();
#endif //MMU_M600_SWITCH_EXTRUDER
    }
    else {
        tmp_extruder = (tmp_extruder+1)%5;
    }
    lcd_update_enable(false);
    lcd_clear();
    lcd_set_cursor(0, 1);
    lcd_puts_P(_T(MSG_LOADING_FILAMENT));
    lcd_print(" ");
    lcd_print(tmp_extruder + 1);
    snmm_filaments_used |= (1 << tmp_extruder); //for stop print

//      printf_P(PSTR("T code: %d \n"), tmp_extruder);
//      mmu_printf_P(PSTR("T%d\n"), tmp_extruder);
    mmu_command(MMU_CMD_T0 + tmp_extruder);

    manage_response(false, true);
    mmu_command(MMU_CMD_C0);
    mmu_extruder = tmp_extruder; //filament change is finished
    mmu_load_to_nozzle();
    load_filament_final_feed();
    st_synchronize();
}


#ifdef SNMM
void extr_mov(float shift, float feed_rate)
{   //move extruder no matter what the current heater temperature is
    set_extrude_min_temp(.0);
    current_position[E_AXIS] += shift;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feed_rate, active_extruder);
    set_extrude_min_temp(EXTRUDE_MINTEMP);
}
#endif //SNMM


void change_extr(int
#ifdef SNMM
                 extr
#endif //SNMM
                ) { //switches multiplexer for extruders
#ifdef SNMM
    st_synchronize();
    delay(100);

    disable_e0();
    disable_e1();
    disable_e2();

    mmu_extruder = extr;

    pinMode(E_MUX0_PIN, OUTPUT);
    pinMode(E_MUX1_PIN, OUTPUT);

    switch (extr) {
    case 1:
        WRITE(E_MUX0_PIN, HIGH);
        WRITE(E_MUX1_PIN, LOW);

        break;
    case 2:
        WRITE(E_MUX0_PIN, LOW);
        WRITE(E_MUX1_PIN, HIGH);

        break;
    case 3:
        WRITE(E_MUX0_PIN, HIGH);
        WRITE(E_MUX1_PIN, HIGH);

        break;
    default:
        WRITE(E_MUX0_PIN, LOW);
        WRITE(E_MUX1_PIN, LOW);

        break;
    }
    delay(100);
#endif
}

int get_ext_nr()
{   //reads multiplexer input pins and return current extruder number (counted from 0)
#ifndef SNMM
    return(mmu_extruder); //update needed
#else
    return(2 * READ(E_MUX1_PIN) + READ(E_MUX0_PIN));
#endif
}


void display_loading()
{
    switch (mmu_extruder)
    {
    case 1:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T1));
        break;
    case 2:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T2));
        break;
    case 3:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T3));
        break;
    default:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T0));
        break;
    }
}

void extr_adj(int extruder) //loading filament for SNMM
{
#ifndef SNMM
    uint8_t cmd = MMU_CMD_L0 + extruder;
    if (cmd > MMU_CMD_L4)
    {
        printf_P(PSTR("Filament out of range %d \n"),extruder);
        return;
    }
    mmu_command(cmd);

    //show which filament is currently loaded

    lcd_update_enable(false);
    lcd_clear();
    lcd_set_cursor(0, 1);
    lcd_puts_P(_T(MSG_LOADING_FILAMENT));
    //if(strlen(_T(MSG_LOADING_FILAMENT))>18) lcd.setCursor(0, 1);
    //else lcd.print(" ");
    lcd_print(" ");
    lcd_print(extruder + 1);

    // get response
    manage_response(false, false);

    lcd_update_enable(true);


    //lcd_return_to_status();
#else

    bool correct;
    max_feedrate[E_AXIS] =80;
    //max_feedrate[E_AXIS] = 50;
START:
    lcd_clear();
    lcd_set_cursor(0, 0);
    switch (extruder) {
    case 1:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T1));
        break;
    case 2:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T2));
        break;
    case 3:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T3));
        break;
    default:
        lcd_display_message_fullscreen_P(_T(MSG_FILAMENT_LOADING_T0));
        break;
    }
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    do {
        extr_mov(0.001,1000);
        delay_keep_alive(2);
    } while (!lcd_clicked());
    //delay_keep_alive(500);
    KEEPALIVE_STATE(IN_HANDLER);
    st_synchronize();
    //correct = lcd_show_fullscreen_message_yes_no_and_wait_P(MSG_FIL_LOADED_CHECK, false);
    //if (!correct) goto  START;
    //extr_mov(BOWDEN_LENGTH/2.f, 500); //dividing by 2 is there because of max. extrusion length limitation (x_max + y_max)
    //extr_mov(BOWDEN_LENGTH/2.f, 500);
    extr_mov(bowden_length[extruder], 500);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_puts_P(_T(MSG_LOADING_FILAMENT));
    if(strlen(_T(MSG_LOADING_FILAMENT))>18) lcd_set_cursor(0, 1);
    else lcd_print(" ");
    lcd_print(mmu_extruder + 1);
    lcd_set_cursor(0, 2);
    lcd_puts_P(_T(MSG_PLEASE_WAIT));
    st_synchronize();
    max_feedrate[E_AXIS] = 50;
    lcd_update_enable(true);
    lcd_return_to_status();
    lcdDrawUpdate = 2;
#endif
}

struct E_step
{
    float extrude;   //!< extrude distance in mm
    float feed_rate; //!< feed rate in mm/s
};
static const E_step ramming_sequence[] PROGMEM =
{
    {1.0,   1000.0/60},
    {1.0,   1500.0/60},
    {2.0,   2000.0/60},
    {1.5,   3000.0/60},
    {2.5,   4000.0/60},
    {-15.0, 5000.0/60},
    {-14.0, 1200.0/60},
    {-6.0,  600.0/60},
    {10.0,  700.0/60},
    {-10.0, 400.0/60},
    {-50.0, 2000.0/60},
};

//! @brief Unload sequence to optimize shape of the tip of the unloaded filament
static void filament_ramming()
{
    for(uint8_t i = 0; i < (sizeof(ramming_sequence)/sizeof(E_step)); ++i)
    {
        current_position[E_AXIS] += pgm_read_float(&(ramming_sequence[i].extrude));
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
                         current_position[E_AXIS], pgm_read_float(&(ramming_sequence[i].feed_rate)), active_extruder);
        st_synchronize();
    }
}

void extr_unload()
{   //unload just current filament for multimaterial printers
#ifdef SNMM
    float tmp_motor[3] = DEFAULT_PWM_MOTOR_CURRENT;
    float tmp_motor_loud[3] = DEFAULT_PWM_MOTOR_CURRENT_LOUD;
    uint8_t SilentMode = eeprom_read_byte((uint8_t*)EEPROM_SILENT);
#endif

    if (degHotend0() > EXTRUDE_MINTEMP)
    {
#ifndef SNMM
        st_synchronize();

        //show which filament is currently unloaded
        lcd_update_enable(false);
        lcd_clear();
        lcd_set_cursor(0, 1);
        lcd_puts_P(_T(MSG_UNLOADING_FILAMENT));
        lcd_print(" ");
        lcd_print(mmu_extruder + 1);

        filament_ramming();

        mmu_command(MMU_CMD_U0);
        // get response
        manage_response(false, true);

        lcd_update_enable(true);
#else //SNMM

        lcd_clear();
        lcd_display_message_fullscreen_P(PSTR(""));
        max_feedrate[E_AXIS] = 50;
        lcd_set_cursor(0, 0);
        lcd_puts_P(_T(MSG_UNLOADING_FILAMENT));
        lcd_print(" ");
        lcd_print(mmu_extruder + 1);
        lcd_set_cursor(0, 2);
        lcd_puts_P(_T(MSG_PLEASE_WAIT));
        if (current_position[Z_AXIS] < 15) {
            current_position[Z_AXIS] += 15; //lifting in Z direction to make space for extrusion
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 25, active_extruder);
        }

        current_position[E_AXIS] += 10; //extrusion
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 10, active_extruder);
        st_current_set(2, E_MOTOR_HIGH_CURRENT);
        if (current_temperature[0] < 230) { //PLA & all other filaments
            current_position[E_AXIS] += 5.4;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 2800 / 60, active_extruder);
            current_position[E_AXIS] += 3.2;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 3000 / 60, active_extruder);
            current_position[E_AXIS] += 3;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 3400 / 60, active_extruder);
        }
        else { //ABS
            current_position[E_AXIS] += 3.1;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 2000 / 60, active_extruder);
            current_position[E_AXIS] += 3.1;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 2500 / 60, active_extruder);
            current_position[E_AXIS] += 4;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 3000 / 60, active_extruder);
            delay_keep_alive(4700);
        }

        max_feedrate[E_AXIS] = 80;
        current_position[E_AXIS] -= (bowden_length[mmu_extruder] + 60 + FIL_LOAD_LENGTH) / 2;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 500, active_extruder);
        current_position[E_AXIS] -= (bowden_length[mmu_extruder] + 60 + FIL_LOAD_LENGTH) / 2;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 500, active_extruder);
        st_synchronize();
        //st_current_init();
        if (SilentMode != SILENT_MODE_OFF) st_current_set(2, tmp_motor[2]); //set back to normal operation currents
        else st_current_set(2, tmp_motor_loud[2]);
        lcd_update_enable(true);
        lcd_return_to_status();
        max_feedrate[E_AXIS] = 50;
#endif //SNMM
    }
    else
    {
        show_preheat_nozzle_warning();
    }
    //lcd_return_to_status();
}

//wrapper functions for loading filament
void extr_adj_0()
{
#ifndef SNMM
    enquecommand_P(PSTR("M701 E0"));
#else
    change_extr(0);
    extr_adj(0);
#endif
}

void extr_adj_1()
{
#ifndef SNMM
    enquecommand_P(PSTR("M701 E1"));
#else
    change_extr(1);
    extr_adj(1);
#endif
}

void extr_adj_2()
{
#ifndef SNMM
    enquecommand_P(PSTR("M701 E2"));
#else
    change_extr(2);
    extr_adj(2);
#endif
}

void extr_adj_3()
{
#ifndef SNMM
    enquecommand_P(PSTR("M701 E3"));
#else
    change_extr(3);
    extr_adj(3);
#endif
}

void extr_adj_4()
{
#ifndef SNMM
    enquecommand_P(PSTR("M701 E4"));
#else
    change_extr(4);
    extr_adj(4);
#endif
}

void mmu_load_to_nozzle_0()
{
    lcd_mmu_load_to_nozzle(0);
}

void mmu_load_to_nozzle_1()
{
    lcd_mmu_load_to_nozzle(1);
}

void mmu_load_to_nozzle_2()
{
    lcd_mmu_load_to_nozzle(2);
}

void mmu_load_to_nozzle_3()
{
    lcd_mmu_load_to_nozzle(3);
}

void mmu_load_to_nozzle_4()
{
    lcd_mmu_load_to_nozzle(4);
}

void mmu_eject_fil_0()
{
    mmu_eject_filament(0, true);
}

void mmu_eject_fil_1()
{
    mmu_eject_filament(1, true);
}

void mmu_eject_fil_2()
{
    mmu_eject_filament(2, true);
}

void mmu_eject_fil_3()
{
    mmu_eject_filament(3, true);
}

void mmu_eject_fil_4()
{
    mmu_eject_filament(4, true);
}

void load_all()
{
#ifndef SNMM
    enquecommand_P(PSTR("M701 E0"));
    enquecommand_P(PSTR("M701 E1"));
    enquecommand_P(PSTR("M701 E2"));
    enquecommand_P(PSTR("M701 E3"));
    enquecommand_P(PSTR("M701 E4"));
#else
    for (int i = 0; i < 4; i++)
    {
        change_extr(i);
        extr_adj(i);
    }
#endif
}

//wrapper functions for changing extruders
void extr_change_0()
{
    change_extr(0);
    lcd_return_to_status();
}

void extr_change_1()
{
    change_extr(1);
    lcd_return_to_status();
}

void extr_change_2()
{
    change_extr(2);
    lcd_return_to_status();
}

void extr_change_3()
{
    change_extr(3);
    lcd_return_to_status();
}

#ifdef SNMM
//wrapper functions for unloading filament
void extr_unload_all()
{
    if (degHotend0() > EXTRUDE_MINTEMP)
    {
        for (int i = 0; i < 4; i++)
        {
            change_extr(i);
            extr_unload();
        }
    }
    else
    {
        show_preheat_nozzle_warning();
        lcd_return_to_status();
    }
}

//unloading just used filament (for snmm)
void extr_unload_used()
{
    if (degHotend0() > EXTRUDE_MINTEMP) {
        for (int i = 0; i < 4; i++) {
            if (snmm_filaments_used & (1 << i)) {
                change_extr(i);
                extr_unload();
            }
        }
        snmm_filaments_used = 0;
    }
    else {
        show_preheat_nozzle_warning();
        lcd_return_to_status();
    }
}
#endif //SNMM

void extr_unload_0()
{
    change_extr(0);
    extr_unload();
}

void extr_unload_1()
{
    change_extr(1);
    extr_unload();
}

void extr_unload_2()
{
    change_extr(2);
    extr_unload();
}

void extr_unload_3()
{
    change_extr(3);
    extr_unload();
}

void extr_unload_4()
{
    change_extr(4);
    extr_unload();
}

bool mmu_check_version()
{
    return (mmu_buildnr >= MMU_REQUIRED_FW_BUILDNR);
}

void mmu_show_warning()
{
    printf_P(PSTR("MMU2 firmware version invalid. Required version: build number %d or higher."), MMU_REQUIRED_FW_BUILDNR);
    kill(_i("Please update firmware in your MMU2. Waiting for reset."));
}

void lcd_mmu_load_to_nozzle(uint8_t filament_nr)
{
    if (degHotend0() > EXTRUDE_MINTEMP)
    {
        tmp_extruder = filament_nr;
        lcd_update_enable(false);
        lcd_clear();
        lcd_set_cursor(0, 1);
        lcd_puts_P(_T(MSG_LOADING_FILAMENT));
        lcd_print(" ");
        lcd_print(tmp_extruder + 1);
        mmu_command(MMU_CMD_T0 + tmp_extruder);
        manage_response(true, true);
        mmu_command(MMU_CMD_C0);
        mmu_extruder = tmp_extruder; //filament change is finished
        mmu_load_to_nozzle();
        load_filament_final_feed();
        st_synchronize();
        custom_message_type = CUSTOM_MSG_TYPE_F_LOAD;
        lcd_setstatuspgm(_T(MSG_LOADING_FILAMENT));
        lcd_return_to_status();
        lcd_update_enable(true);
        lcd_load_filament_color_check();
        lcd_setstatuspgm(_T(WELCOME_MSG));
        custom_message_type = CUSTOM_MSG_TYPE_STATUS;
    }
    else
    {
        show_preheat_nozzle_warning();
    }
}

void mmu_eject_filament(uint8_t filament, bool recover)
{
    if (filament < 5)
    {

        if (degHotend0() > EXTRUDE_MINTEMP)
        {
            st_synchronize();

            {
                LcdUpdateDisabler disableLcdUpdate;
                lcd_clear();
                lcd_set_cursor(0, 1);
                lcd_puts_P(_i("Ejecting filament"));
                current_position[E_AXIS] -= 80;
                plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 2500 / 60, active_extruder);
                st_synchronize();
                mmu_command(MMU_CMD_E0 + filament);
                manage_response(false, false);
                if (recover)
                {
                    lcd_show_fullscreen_message_and_wait_P(_i("Please remove filament and then press the knob."));
                    mmu_command(MMU_CMD_R0);
                    manage_response(false, false);
                }

            }
        }
        else
        {
            show_preheat_nozzle_warning();
        }
    }
    else
    {
        puts_P(PSTR("Filament nr out of range!"));
    }
}
