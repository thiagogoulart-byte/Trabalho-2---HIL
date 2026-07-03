
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
#define FS = 20000.0f     // 
#define FREQ 60.0f   

// termos do modelo do conversor
float XR = 30.0f;
float R;
float Ts;
float a11;
float b00;
float b11;

float vinv, iL1, u1;
float iL0 = 0.0f;
float u0 = 0.0f;
float Vdc = 400.0f;

#pragma DATA_SECTION(iL, "CpuToCla1MsgRAM");
volatile float iL;

uint16_t DAC_iL;
float teste_iL_dac;

// Chaves
volatile bool S1;
volatile bool S2;
volatile bool S3;
volatile bool S4;


// Variáveis para calcular a tensão da rede
float ang;
volatile float vg = 0;
float vg_ant;
// Compartilhar para usar o no Feedforward do controlador
#pragma DATA_SECTION(vg, "CpuToCla1MsgRAM");
#pragma DATA_SECTION(vg_ant, "CpuToCla1MsgRAM");


#pragma DATA_SECTION(ePwm_TimeBaseA,"CpuToCla1MsgRAM");
uint32_t ePwm_TimeBaseA;
#pragma DATA_SECTION(ePwm_TimeBaseB,"CpuToCla1MsgRAM");
uint32_t ePwm_TimeBaseB;


// Variáveis para buffer
#define TAM_BUFFER 1000
uint16_t idx_buffer = 0;

// Buffers
float buffer_iL[TAM_BUFFER];
// float buffer_vinv[TAM_BUFFER];
// float buffer_u[TAM_BUFFER];
// float buffer_pwm1A[TAM_BUFFER];  
// float buffer_pwm1B[TAM_BUFFER];  
// float buffer_pwm3A[TAM_BUFFER]; 
// float buffer_pwm3B[TAM_BUFFER];

// Variável para rodar a planta
volatile bool g_new_step_ready = true;

int n_cont = 0;
extern bool mudou_amp;

void main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    ePwm_TimeBaseA = EPWM_getTimeBasePeriod(myEPWM0_BASE);
    ePwm_TimeBaseB = EPWM_getTimeBasePeriod(myEPWM1_BASE);

    Ts = 1.0f / (40.0e4f);
    R = 2.0f*PI*FREQ*L/XR;

    // Constantes do modelo
    float den = (2.0f * L / Ts) + R;
    a11 = ((2.0f * L / Ts) - R) / den;
    b00 = 1.0f / den;
    b11 = b00;

    EINT;
    ERTM;

    for(;;)
    {
        if (g_new_step_ready)
        {
            g_new_step_ready = false;
            // Calcular a tensão da rede
            ang += 2.0f * 3.1415f * 60.0 * Ts; 
            if(ang >= 6.283185f)  ang -= 6.283185f;     
            vg = 311.1269f*sinf(ang); // Tensão da rede
            
            // Chaveamento
            if (S1 == 1 && S4 == 1)       vinv = Vdc; 
            else if (S2 == 1 && S3 == 1)  vinv = -Vdc; 
            else                          vinv = 0.0f;
            // Planta
            u0 = vinv - vg_ant;
        
            iL = a11*iL0 + b00*u0 + b11*u1;

            // Buffers
            buffer_iL[idx_buffer] = iL;
            //// Colocando os buffers na mesma escala
            // buffer_u[idx_buffer] = u*10.0f;
            // buffer_vinv[idx_buffer] = vinv*0.05f;
            // buffer_pwm1A[idx_buffer] = S1 * 10.0f;
            // buffer_pwm1B[idx_buffer] = S2 * 10.0f;
            // buffer_pwm3A[idx_buffer] = S3 * 10.0f;
            // buffer_pwm3B[idx_buffer] = S4 * 10.0f;
            
            idx_buffer++;
            if(idx_buffer >= TAM_BUFFER)  idx_buffer = 0;    
            
            // Ideia para pegar a mudança de potência
            // if (mudou_amp) // Quando mudar na task começa a contar aqui
            // {
            //     n_cont++; 
            // }
            // if (n_cont >= 500) // até 500 porque é metade do buffer
            // {
            //     n_cont = 0; //só para colocar um breakpoint e pegar o print
            // }
    
            // Atualização das variávies 
            u1  = u0;
            iL0 = iL;
            vg_ant = vg;

            // Normalização do DAC em 20A
            // DAC_iL = (uint16_t) ((teste_iL_dac + 20.0f)*(102.375f));
            DAC_iL = (uint16_t) ((iL + 20.0f)*(102.375f)); //4095/40
            DAC_setShadowValue(DAC_iL_BASE, (uint16_t) (DAC_iL));
            // CLA_forceTasks(myCLA0_BASE,CLA_TASKFLAG_1);
        }
    }
}

__interrupt void cla1Isr1 ()
{
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
