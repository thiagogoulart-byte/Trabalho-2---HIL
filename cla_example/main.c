
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
// float vg = 0.0f;

// Chaves
volatile bool S1;
volatile bool S2;
volatile bool S3;
volatile bool S4;



// float a1 = -1.999733f;
// float a2 = 1.0f;

// float b0 = 24.97e-6f;
// float b1 = -17.76e-9f;
// float b2 = -24.99e-6f;

// float kp = 40.0f;
// float ki = 55850.0f;

// #define a1    (-1.99431780052123f)
// #define a2    ( 1.00000000000000f)

// #define b0    ( 9.82054592384524e-5f)
// #define b1    (-1.13213776461546e-6f)
// #define b2    (-9.93375970030678e-5f)
// #define kp    (10.471975f)
// #define ki    (3655.409f)


uint32_t ePwm_TimeBase;
// uint32_t ePwm_MinDuty;
// uint32_t ePwm_MaxDuty;
// uint32_t ePwm_curDuty;


// float theta = 0.0f;

// float amplitude = 13.0f;
float fs = 5000.0f;     // 
#define freq 60.0f     //
// #define fsck 15e5f


// Definições de Constantes
//
#define F_PWM                  5000.0f     // Frequência de chaveamento (Hz)
#define T_PWM                  (1.0f / F_PWM) // Período de chaveamento (s)
#define DT_SIM                 0.000001f    // Passo de simulação (5 us)
#define N_STEPS_PER_CYCLE      (uint32_t)(T_PWM / DT_SIM) // Passos por ciclo PWM

// #define NORM_DAC 4095.0f/(15.0f)

// #pragma DATA_SECTION(a1, a2, b0, b1, b2, kp, ki, "CpuToCla1MsgRAM");

#pragma DATA_SECTION(vg, "CpuToCla1MsgRAM");
float vg = 0;

// #pragma DATA_SECTION(Iref, "Cla1ToCpuMsgRAM");
// extern float Iref;
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

#pragma DATA_SECTION(ePwm_TimeBase,"CpuToCla1MsgRAM");

#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
extern uint32_t CAMPA; 

uint16_t DAC_iL;

// float ang;

#define TAM_BUFFER 100

float buffer_iL[TAM_BUFFER];
float buffer_vinv[TAM_BUFFER];
float buffer_u[TAM_BUFFER];

float buffer_pwm1A[TAM_BUFFER];  
float buffer_pwm1B[TAM_BUFFER];  
float buffer_pwm3A[TAM_BUFFER]; 
float buffer_pwm3B[TAM_BUFFER];

uint16_t idx_buffer = 0;
// volatile uint16_t g_step_counter = 0;
volatile bool g_new_step_ready = true;

void main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();


    ePwm_TimeBase = EPWM_getTimeBasePeriod(myEPWM0_BASE);
    // ePwm_MinDuty = (uint32_t) (0.95f * (float) ePwm_TimeBase);
    // ePwm_MaxDuty = (uint32_t) (0.05f * (float) ePwm_TimeBase);

    Ts = 1/fs;
    R = 2.0f*PI*freq*L/XR;

    a11 = ((2.0f*L/Ts) - R) / ((2.0f*L/Ts) + R);
    b00 = 1.0f / ((2.0f*L/Ts) + R);
    b11 = b00;

    EINT;
    ERTM;

    // CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);


    for(;;)
    {


        // if (g_new_step_ready)
        // {
        //     g_new_step_ready = false;

        //     if(S1 == 1 && S4 == 1){
        //         vinv = Vdc;
        //         // iL = 1001.0f;
        //     }
        //     else if(S2 == 1 && S3 == 1){
        //         vinv = -Vdc;
        //         // iL = 0110.0f;
        //     }
        //     else
        //     {
        //         vinv = 0.0;
        //         // iL = 0.0f;
        //     }    
                
            

        //     u0 = vinv - vg;

        //     iL = a11*iL0 + b00*u0 + b11*u1;
        //     buffer_iL[idx_buffer] = iL;
        //     buffer_vinv[idx_buffer] = vinv/20;
        //     buffer_u[idx_buffer] = u*20.0f;
        //     buffer_pwm1A[idx_buffer] = S1 * 10.0f;
        //     buffer_pwm1B[idx_buffer] = S2 * 10.0f;
        //     buffer_pwm3A[idx_buffer] = S3 * 10.0f;
        //     buffer_pwm3B[idx_buffer] = S4 * 10.0f;
        //     idx_buffer++;

        //     if(idx_buffer >= TAM_BUFFER)
        //     {
        //         idx_buffer = 0;      // volta para o início
        //     }
        

        //     // ang += 2.0f * 3.1415f * 60.0 / 40000.0f; //para fazer o vg senoidal //talvez pegar o theta da cla
        //     // if(ang >= 2.0f * 3.1415f)
        //     // {
        //     //     ang -= 2.0f * 3.1415f;     
        //     // }
        //     // vg = 180*sinf(ang);

        //     u1  = u0;
        //     iL0 = iL;
        //     if(iL > 20.0f)
        //     {
        //         iL = 20.0f;
        //     }
        //     if(iL < -20.0f)
        //     {
        //         iL = -20.0f;
        //     }

        //     DAC_iL = (uint16_t) ((iL + 20.0f)*(4095.0f/40.0f));
        
        //     DAC_setShadowValue(DAC_iL_BASE, (uint16_t) (DAC_iL));

        //     CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);
            
        //     // DEVICE_DELAY_US(10000);
        // }
    
    }
}




__interrupt void cla1Isr1 ()
{
    
    fVal = fResult;
    Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);
}

__interrupt void INT_readGPIO_ISR(void)
{

    // Atualiza contador
    // g_step_counter++;

    // // Reinicia no fim do ciclo PWM
    // if (g_step_counter >= N_STEPS_PER_CYCLE)
    //     g_step_counter = 0;

    // Sinaliza para o loop principal que deve simular o pr�ximo passo
    g_new_step_ready = true;

     if (S1 == 1 && S2 == 0 && S3 == 0 && S4 == 1) 
        {
            // Braço 1 em +Vdc/2 e Braço 2 em -Vdc/2 -> Vinv = +Vdc
            vinv = Vdc; 
        }
        else if (S1 == 0 && S2 == 1 && S3 == 1 && S4 == 0) 
        {
            // Braço 1 em -Vdc/2 e Braço 2 em +Vdc/2 -> Vinv = -Vdc
            vinv = -Vdc; 
        }
        else 
        {
            // Qualquer outra combinação (estados nulos ou períodos de dead-band)
            vinv = 0.0f; 
        }  
                        
            

            u0 = vinv - vg;

            iL = a11*iL0 + b00*u0 + b11*u1;
            buffer_iL[idx_buffer] = iL;
            buffer_vinv[idx_buffer] = vinv/20;
            buffer_u[idx_buffer] = u*20.0f;
            buffer_pwm1A[idx_buffer] = S1 * 10.0f;
            buffer_pwm1B[idx_buffer] = S2 * 10.0f;
            buffer_pwm3A[idx_buffer] = S3 * 10.0f;
            buffer_pwm3B[idx_buffer] = S4 * 10.0f;
            idx_buffer++;

            if(idx_buffer >= TAM_BUFFER)
            {
                idx_buffer = 0;      // volta para o início
            }
        

            // ang += 2.0f * 3.1415f * 60.0 / 40000.0f; //para fazer o vg senoidal //talvez pegar o theta da cla
            // if(ang >= 2.0f * 3.1415f)
            // {
            //     ang -= 2.0f * 3.1415f;     
            // }
            // vg = 180*sinf(ang);

            u1  = u0;
            iL0 = iL;
            if(iL > 20.0f)
            {
                iL = 20.0f;
            }
            if(iL < -20.0f)
            {
                iL = -20.0f;
            }

            DAC_iL = (uint16_t) ((iL + 20.0f)*(4095.0f/40.0f));
        
            DAC_setShadowValue(DAC_iL_BASE, (uint16_t) (DAC_iL));

            CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);

    Interrupt_clearACKGroup(INT_readGPIO_INTERRUPT_ACK_GROUP);

}

__interrupt void INT_GPIO_S1_XINT_ISR(void)
{
    S1 = GPIO_readPin(GPIO_S1);

    S2 = GPIO_readPin(GPIO_S2);
    S4 = GPIO_readPin(GPIO_S4);
    S3 = GPIO_readPin(GPIO_S3);
    
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void INT_GPIO_S2_XINT_ISR(void)
{
    S2 = GPIO_readPin(GPIO_S2);

    S1 = GPIO_readPin(GPIO_S1);
    S4 = GPIO_readPin(GPIO_S4);
    S3 = GPIO_readPin(GPIO_S3);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


__interrupt void INT_GPIO_S3_XINT_ISR(void)
{
    S1 = GPIO_readPin(GPIO_S1);
    S2 = GPIO_readPin(GPIO_S2);
    S4 = GPIO_readPin(GPIO_S4);

    S3 = GPIO_readPin(GPIO_S3);
    

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void INT_GPIO_S4_XINT_ISR(void)
{
    S1 = GPIO_readPin(GPIO_S1);
    S2 = GPIO_readPin(GPIO_S2);
    S3 = GPIO_readPin(GPIO_S3);
    
    S4 = GPIO_readPin(GPIO_S4);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
