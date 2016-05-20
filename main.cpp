//#include "mbed.h"
#include "MPU6050.h"
#include "nRF24L01P.h"
#include "pid.h"
#define TRANSFER_SIZE   4


DigitalOut myledR(LED_RED);
DigitalOut myledG(LED_GREEN);
DigitalOut myledB(LED_BLUE);
//#include "motor.h"
//#include "imu.h"
Timer t;//declare timer variable
float A_XYZ[3],G_XYZ[3];
int16_t ax, ay, az,gx, gy, gz;
int i;
float X_angle,Y_angle,Z_angle;
double arx, ary, arz, grx, gry, grz, rx, ry, rz;
double timeStep;
    char rxData[4];
    char recvd;
    double txData[TRANSFER_SIZE];
    int txDatacnt= 0;
    int rxDataCnt = 0;
    //global variable for setpoint
double setpR=0.0,setpY=0.0,setpP=0.0,setpT=0.0;

// Global system variables
#define NRF_MOSI  PTD6
#define NRF_MISO  PTD7
#define NRF_SCK   PTD5
#define NRF_CSN   PTD4 
#define NRF_CE    PTB20
#define NRF_IRQ   PTC18
//Declare components
MPU6050 MPU6050(I2C_SDA, I2C_SCL);//i2c_sda is pin24 or D15
nRF24L01P my_nrf24l01p(NRF_MOSI, NRF_MISO, NRF_SCK, NRF_CSN, NRF_CE, NRF_IRQ);    // mosi, miso, sck, csn, ce, irq
Serial pc(USBTX, USBRX);
struct pid roll,pitch,yaw,height;
double yawF=0.0,rollF=0.0,pitchF=0.0,heightF=0.3;
    float ACCEL_XANGLE;
    float ACCEL_YANGLE;
    float ACCEL_XYZ[3];
float GYRO_XYZ[3];
float ACCEL_ERROR[3];
float X_ERROR=0.0,Y_ERROR=0.0;
float Z_ERROR=0.0;
    
 PwmOut pwm1(D7);
 PwmOut pwm2(D6);
 PwmOut pwm3(D5);
 PwmOut pwm4(D3);
int throttle;
static float motor1power=0.0f;
static float motor2power=0.0f;
static float motor3power=0.0f;
static float motor4power=0.0f;

/*
 * this function sets motor-power proportions for different yaw,pitch and roll
 * angles as value for a particular altitude
 * Inputs:
 *          rollOffset-from pid controller for roll
 *          yawoffset -from yaw pid algorithm
 *          pitchoffset-from pitch
 *          throttle- defined for a particular height
 * Outputs:
 *          motorpower for different rotor combination
 *          i.e
 *          for
 *      pitch:
 *          motor1power increased by pitchoffset and motor3power reduced by
 *          the same amount
 *      yaw:
 *          Both motor1 and motor3 increase by yawoffset while both motor2 and motor4 reduced
 *          by the same amount
 *      roll:
 *          motor2 increased by rolloffset and motor4 reduced by the same amount
 *      altitude:
 *          power for all motors increased or reduced by amount equal to altitudeOffset
 */
void pwmsignals(double throttle,double rolloffset,double yawoffset,double pitchoffset)
{
   
/*************************************************************************************************
 * calculate total output for each motor from the values                                         *
 * obtained from the pid algorithms for each degree of freedom and, compare each resultant value *
 *  with the output limits: outmin and outmax.                                                   *                                                        *
 *                                                                                               *
 * If a value exceed the high output limit,set it to the high limit if a value is lower than the *
 * allowed low output,set it to the low limit                                                    *                                                        *
 *************************************************************************************************/
   double outmax=0.90;//80% of the total power
double outmin=0.00f;//50% of the total power

    motor1power=throttle-pitchoffset/2-rolloffset/2+yawoffset/2;
    //check for motor1power
 if(motor1power>outmax)
    {
        motor1power=outmax;
    }else if(motor1power<outmin)
    {
        motor1power=outmin;
    }
    //set dutyratio for pwm1
    pwm1.write(motor1power);

    //check limits for motor2power
    motor2power=throttle-pitchoffset/2+rolloffset/2-yawoffset/2;
    if(motor2power>outmax)
    {
        motor2power=outmax;
    }else if(motor2power<outmin)
    {
        motor2power=outmin;
    }
    pwm2.write(motor2power);
    //check for motor3power
    motor3power=throttle+pitchoffset/2+rolloffset/2+yawoffset/2;
    if(motor3power>outmax)
    {
        motor3power=outmax;
    }else if(motor3power<outmin)
    {
        motor3power=outmin;
    }
    pwm3.write(motor3power);
    //check for motor4power
    motor4power=throttle+pitchoffset/2-rolloffset/2-yawoffset/2;
 if(motor4power>outmax)
    {
        motor4power=outmax;
    }else if(motor4power<outmin)
    {
        motor4power=outmin;
    }
    pwm4.write(motor4power);
    //pc.printf("pwm func! ");//for debuging purpose
}

void setup(){
    MPU6050.setBW(MPU6050_BW_256);
    MPU6050.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    MPU6050.setGyroRange(MPU6050_GYRO_RANGE_250);
    my_nrf24l01p.powerUp();
    pc.baud(38400);
    
    // Display the (default) setup of the nRF24L01+ chip
    pc.printf("nRF24L01+ Frequency    : %d MHz\r\n", my_nrf24l01p.getRfFrequency());
    pc.printf("nRF24L01+ Output power : %d dBm\r\n", my_nrf24l01p.getRfOutputPower());
    pc.printf("nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate());
    pc.printf("nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress());
    pc.printf("nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress());

    pc.printf("Type keys to test transfers:\r\n  (transfers are grouped into %d characters)\r\n", TRANSFER_SIZE);
    my_nrf24l01p.setTransferSize(TRANSFER_SIZE);
    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();

    i=1;
    }
 
void getRawValues()
{
       //set up time for integration
    t.start();
    MPU6050.getAccelero(A_XYZ);
    MPU6050.getGyro(G_XYZ);
    t.stop();
    timeStep=t.read();
    ax=A_XYZ[0];
    ay=A_XYZ[1];
    az=A_XYZ[2];
    
    gx=(double)G_XYZ[0]*57.2958;
    gy=(double)G_XYZ[1]*57.2958;
    gz=(double)G_XYZ[2]*57.2958;
    //calcualte accelerometer angles
    X_angle=(double)57.295*atan((float)ax/sqrt(pow((float)ay,2)+pow((float)az,2)));
    //Rotation on Y is Roll i.e Accel_Yangle
    Y_angle=(double)57.295*atan((float)ay/sqrt(pow((float)ax,2)+pow((float)az,2)));
    //Rotation on Z
    Z_angle=(double)57.295*atan((float)sqrt(pow((float)ay,2)+pow((float)ax,2)))/(float)az;
   // set initial values equal to accel values
    if (i == 1) {
        grx = X_angle;
        gry = Y_angle;
        grz = Z_angle;
        }
        //integrate to find the gyro angles
        else{
            grx = grx + (timeStep * gx);
            gry = gry + (timeStep * gy);
            grz = grz + (timeStep * gz);
            }
            //apply filter
            rx= (0.01 * arx) + (0.99 * grx);
            ry= (0.01 * ary) + (0.99 * gry);
            rz= (0.01 * arz) + (0.99 * grz);
        t.reset();
    }


//function to initialize pids
void init_pids()
{
    pid_init(&roll);
    pid_init(&yaw);
    pid_init(&pitch);
    pid_init(&height);
    }

void transCeiver()
    {
        }
int main()
{
    
    //initialize pids
    init_pids();
    
    //initialize pwm
    pwm1.period_ms(100);
    pwm2.period_ms(100);
    pwm3.period_ms(100);
    pwm4.period_ms(100);
    pwm1.write(0.00f);
    pwm2.write(0.00f);
    pwm3.write(0.00f);
    pwm4.write(0.00f);
    bool connection=0;
    setup();//Prepare Mpu6050 For Communication
   
    if(MPU6050.testConnection())
    {
        pc.printf("MPU6050 connected...\r\n");
        connection=1;
       // myledR=1;
        myledG=0;//connection led
    }
      else 
    {
        pc.printf("MPU6050 not connected...\r\n");
        connection=0;
        //myledR=1;
        myledG=0;
    }
    while (connection)
    {
        if(MPU6050.testConnection())
        {


            getRawValues();
           //transCeiver();
           
         
         // ...add orientation data to the transmit buffer
            txData[0] =rx;
            txData[1] =ry;
            txData[2] =rz;
            txData[3] =heightF;
            // If the transmit buffer is full
          //pc.printf("Function executed \r\n");
                my_nrf24l01p.disable();
                my_nrf24l01p.setTransmitMode();
                my_nrf24l01p.enable();
                // Send the transmitbuffer via the nRF24L01+
                //wait(0.2f);
               txDatacnt=my_nrf24l01p.write(NRF24L01P_PIPE_P0, txData, sizeof(txData));                           
                my_nrf24l01p.disable();
                my_nrf24l01p.setReceiveMode();
                my_nrf24l01p.enable();
            if (my_nrf24l01p.readable())
            {
           // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read(NRF24L01P_PIPE_P0, rxData, sizeof(rxData));

                recvd=rxData[0];

            // Display the receive buffer contents via the host serial link
          switch(recvd)
              {
                    case 'F':
                    myledR=1;
                    setpP-=1;
                    setpR=0;
                   // pc.printf("moving forward... \r\n");
                    rxData[0]='0';
                    break;
                    
                    case 'B':
                    myledR=1;
                    setpP+=1;
                    setpR=0;
                    //pc.printf("moving Backward...\r\n");
                    rxData[0]='0';
                    break;
                    
                    case 'S':
                    myledR=1;
                    setpP=0;
                    setpR=0;
                    //pc.printf("stoping... \r\n");
                    rxData[0]='0';
                    break;
                    
                    case 'R':
                    setpP=0;
                    setpR-=1;
                    //pc.printf("Rolling...right \r\n");
                    rxData[0]='0';
                    break;
                    
                    case 'L':
                    myledR=1;
                    setpP=0;
                    setpR+=1;
                    //pc.printf("Rolling...left \r\n");
                    rxData[0]='0';
                    break;
                   //default :
                    //pc.printf("Waiting... \r");
                    //pc.printf("Waiting... %c\r",recvd);
                    //break;
                }
           // pc.printf("recvd %c\r\n",recvd);
           }
           
             //calculate orientation and determine motor power      
             ACCEL_YANGLE=ry;ACCEL_XANGLE=rx; 
             rollF=pid_out(&roll,ACCEL_YANGLE,setpR);
             pitchF=pid_out(&pitch,ACCEL_XANGLE,setpP);
             pwmsignals((float)heightF,(float)rollF,(float)yawF,(float)pitchF);//call function for esc
            // pc.printf("Motor1:%.2f\tMotor2:%.2f\tMotor3:%.2f\tMotor4:%.2f\r",motor1power,motor2power,motor3power,motor4power);
            //writing current orientation
            pc.printf("\rPitch %.4f\tRoll %.2f\tZ_angle %.4f\t\r",rx,ry,rz);
        }
            else 
        {
                pc.printf("No connection..\r\n");
        } 
        
    }
}