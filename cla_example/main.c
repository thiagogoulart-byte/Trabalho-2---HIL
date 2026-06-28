
//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "src/scicomm.h"
#include "math.h"

#define L 3e-3f
#define PI 3.1415f

// termos do modelo do conversor
float XR = 30.0f;
float R;
float Ts;
float a11;
float b00;
float b11;

float vinv, iL0, iL1, u0, u1;
float Vdc = 400.0f;
// float vg = 220.0f;

// Chaves
volatile bool S1;
volatile bool S2;
volatile bool S3;
volatile bool S4;


uint32_t ePwm_TimeBaseA;
uint32_t ePwm_TimeBaseB;


// float amplitude = 13.0f;
float fs = 20000.0f;     // 
#define freq 60.0f     //
// #define fsck 15e5f


#pragma DATA_SECTION(vg, "CpuToCla1MsgRAM");
volatile float vg = 0;

#pragma DATA_SECTION(AMP, "CpuToCla1MsgRAM");
volatile float AMP = 13.0f;

#pragma DATA_SECTION(iL, "CpuToCla1MsgRAM");
float iL;

#pragma DATA_SECTION(u, "Cla1ToCpuMsgRAM");
float u;

#pragma DATA_SECTION(fVal,"CpuToCla1MsgRAM");
float fVal;
#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
float fResult;

#pragma DATA_SECTION(ePwm_TimeBaseA,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(ePwm_TimeBaseB,"CpuToCla1MsgRAM");

#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
extern uint32_t CAMPA; 

uint16_t DAC_iL;

float ang;

#define TAM_BUFFER 500

float buffer_iL[TAM_BUFFER];
// float buffer_vinv[TAM_BUFFER];
float buffer_u[TAM_BUFFER];

// float buffer_pwm1A[TAM_BUFFER];  
// float buffer_pwm1B[TAM_BUFFER];  
// float buffer_pwm3A[TAM_BUFFER]; 
// float buffer_pwm3B[TAM_BUFFER];

uint16_t idx_buffer = 0;

volatile bool g_new_step_ready = true;

uint32_t PULAR_PASSO = 1000;
uint32_t contar_passos = 0;
uint32_t temp_delay = 25;
extern float theta;
#pragma DATA_SECTION(vg_ant, "CpuToCla1MsgRAM");
float vg_ant;

void main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    ePwm_TimeBaseA = EPWM_getTimeBasePeriod(myEPWM0_BASE);
    ePwm_TimeBaseB = EPWM_getTimeBasePeriod(myEPWM1_BASE);

    Ts = 1.0f / 15000000.0f;
    R = 2.0f*PI*freq*L/XR;

    a11 = ((2.0f*L/Ts) - R) / ((2.0f*L/Ts) + R);
    b00 = 1.0f / ((2.0f*L/Ts) + R);
    b11 = b00;

    EINT;
    ERTM;

    for(;;)
    {

        if (g_new_step_ready)
        {
            g_new_step_ready = false;

            // ang += 2.0f * 3.1415f * 60.0 * Ts; //para fazer o vg senoidal //talvez pegar o theta da cla
            // if(ang >= 2.0f * 3.1415f)
            // {
            //     ang -= 2.0f * 3.1415f;     
            // }
            

            if (S1 == 1 && S4 == 1) 
            {
                vinv = Vdc; 
            }
            else if (S2 == 1 && S3 == 1) 
            {
                vinv = -Vdc; 
            }
            else 
            {
                vinv = 0.0f; 
            }  
            

            u0 = vinv - vg_ant;

            iL = a11*iL0 + b00*u0 + b11*u1;
            contar_passos++;
            if (contar_passos>= PULAR_PASSO)
            {
                contar_passos = 0;
                buffer_iL[idx_buffer] = iL;
                buffer_u[idx_buffer] = u*10.0f;
                
                idx_buffer++;

                if(idx_buffer >= TAM_BUFFER)
                {
                    idx_buffer = 0;      // volta para o início
                }
    
            }
            // buffer_iL[idx_buffer] = iL;
            // buffer_vinv[idx_buffer] = vinv*0.05f;
            // buffer_u[idx_buffer] = u*20.0f;
            // buffer_pwm1A[idx_buffer] = S1 * 10.0f;
            // buffer_pwm1B[idx_buffer] = S2 * 10.0f;
            // buffer_pwm3A[idx_buffer] = S3 * 10.0f;
            // buffer_pwm3B[idx_buffer] = S4 * 10.0f;

        
            u1  = u0;
            iL0 = iL;
            vg = 311*sinf(theta);
            vg_ant = vg;
            if(iL > 20.0f)
            {
                iL = 20.0f;
            }
            if(iL < -20.0f)
            {
                iL = -20.0f;
            }

            DAC_iL = (uint16_t) ((iL + 20.0f)*(102.375f));
            DAC_setShadowValue(DAC_iL_BASE, (uint16_t) (DAC_iL));
            CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);
            //DEVICE_DELAY_US(temp_delay);
        }
    }
}


__interrupt void cla1Isr1 ()
{
    fVal = fResult;
    Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);
}

__interrupt void INT_myCPUTIMER0_ISR(void)
{
    // Sinaliza para o loop principal que deve simular o próximo passo
    g_new_step_ready = true;
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}

__interrupt void INT_GPIO_S1_XINT_ISR(void)
{
    S1 = GPIO_readPin(GPIO_S1);   
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void INT_GPIO_S2_XINT_ISR(void)
{
    S2 = GPIO_readPin(GPIO_S2);   
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}


__interrupt void INT_GPIO_S3_XINT_ISR(void)
{
    S3 = GPIO_readPin(GPIO_S3);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void INT_GPIO_S4_XINT_ISR(void)
{
    S4 = GPIO_readPin(GPIO_S4);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}
