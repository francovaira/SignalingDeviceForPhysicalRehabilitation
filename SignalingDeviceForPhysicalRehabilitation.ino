#include <Servo.h>

// servo bilateral: 0 ... 90 ... 180
// servo unilateral: 0 ... 90 

#define RAND_PIN_SEED A3
#define LED           13
#define LED_WHITE     5
#define LED_RED       6
#define BUTTON_PEDAL  12
#define BAT_LEVEL     A0
#define POTE          A1
#define SERVO_A       10
#define SERVO_B       11
#define ECHO          8
#define TRIGGER       7

#define LOW_BATTERY_LIMIT 11.0

#define PEDAL_TIME_ON_OFF 80 // tiempo que debe presionarse el pedal para encender
#define PEDAL_TIME_OK     15 // tiempo que debe presionarse el pedal para confirmar

#define DISTANCE_TIME_ACTION  0 // cuanto tiempo debe estar "cerca" para que sea detectado como true

#define DELAY_TIME_BETWEEN_MOVES 350 // tiempo en segundos*100 entre movimientos

#define LOWER_LIMIT_POTE  130  // in [cm] distancias que ajusta el pote
#define UPPER_LIMIT_POTE  350 // in [cm]

#define SENSOR_MAX_DISTANCE 360

#define SERVO_A_INIT_POS  0 // in degrees -- UNILATERAL
#define SERVO_A_MIN_POS   0
#define SERVO_A_MAX_POS   90
#define SERVO_A_TIME_UP   75 // tiempo que permanece arriba

#define SERVO_B_INIT_POS  90 // in degrees -- BILATERAL
#define SERVO_B_MIN_POS   0
#define SERVO_B_MAX_POS   180
#define SERVO_B_TIME_UP   75 // tiempo que permanece arriba

#define ST_APAGADO    0
#define ST_READY      1
#define ST_WORKING    2
#define ST_APAGADO    3

Servo servo_A;
Servo servo_B;

uint8_t state=ST_APAGADO;
uint8_t substate=0;

uint16_t pedal_time_count=0;
uint16_t i=0;

uint16_t delay_time_between_moves=0;

uint8_t servo_A_current_pos=0;
uint16_t servo_A_time_up_count=0;
uint8_t servo_B_current_pos=0;
uint16_t servo_B_time_up_count=0;

uint16_t distance_sense=0;
uint16_t distance_set=0;
uint16_t distance_selected=LOWER_LIMIT_POTE;
uint16_t distance_time_count=0;

void setup()
{
  Serial.begin(9600);
  gpio_init();

  servo_A.attach(SERVO_A);
  servo_B.attach(SERVO_B);
}

void loop()
{
  if(state != ST_APAGADO)
  {
    handle_battery_level();    
  }
  
  // main state machine
  switch(state)
  {
    case ST_APAGADO:
    {
      if(!substate)
      {
        Serial.println("apagado");
        
        analogWrite(LED_WHITE, 0);
        analogWrite(LED_RED, 0);

        servo_A.detach();
        servo_B.detach();
        pedal_time_count=0;
        substate++;
      }

      if(se_pulso_pedal())
      {
        pedal_time_count++;

        if(pedal_time_count >= PEDAL_TIME_ON_OFF)
        {
          pedal_time_count=0;
          substate=0;
          state = ST_READY;   
        }
      }
      else
      {
        pedal_time_count=0;
      }

      break;
    }

    case ST_READY:
    {
      if(!substate)
      {
        analogWrite(LED_WHITE, i++);
        if(i>=255)
        {
          Serial.println("encendido");
          analogWrite(LED_WHITE, 0);
          i=0;
          
          pedal_time_count=0;
          distance_selected=LOWER_LIMIT_POTE;

          servo_A.attach(SERVO_A);
          servo_B.attach(SERVO_B);
          
          servo_A_current_pos = SERVO_A_INIT_POS;
          servo_B_current_pos = SERVO_B_INIT_POS;
          servo_A.write(servo_A_current_pos);
          servo_B.write(servo_B_current_pos);
          while(se_pulso_pedal()) {};
          substate++;

          // jumps to working state just after setting servos
          state = ST_WORKING;
          substate = 0;
          break;
        }
        else
        {
          break;  
        }
      }
    }

    case ST_WORKING:
    {
      if(!substate)
      {
        i=0;
        pedal_time_count=0;
        distance_time_count=0;

        randomSeed(analogRead(RAND_PIN_SEED));
        while(se_pulso_pedal()){};
        substate++;
      }

      // check if has to shut down
      if(se_pulso_pedal())
      {
        pedal_time_count++;

        if(pedal_time_count >= PEDAL_TIME_ON_OFF)
        {
          analogWrite(LED_RED, 255);
          pedal_time_count=0;
          substate=0;
          state = ST_APAGADO;
          while(se_pulso_pedal()) {};   
        }
      }
      else
      {
        pedal_time_count = 0;
      }

      if(delay_time_between_moves)
      {
        delay_time_between_moves--;
        break; // avoids the execution of code below
      }

      // if servos have returned to stand-by position
      if(!servo_A_time_up_count && !servo_B_time_up_count)
      {
        // distance adjust handling
        distance_selected = map(analogRead(POTE), 0, 1023, LOWER_LIMIT_POTE, UPPER_LIMIT_POTE);
        distance_sense = get_distancia();

        // person detection - is it closer than threshold?
        if(distance_sense <= distance_selected)
        {
          // detection delay
          distance_time_count++;
          if(distance_time_count >= DISTANCE_TIME_ACTION)
          {
            // selects a servo by parity of a random number
            if(random()%2==0) // SERVO A (unilateral)
            {
              servo_A_current_pos = SERVO_A_MAX_POS;
              servo_A_time_up_count = SERVO_A_TIME_UP;
              servo_A.write(servo_A_current_pos);
            }
            else // SERVO B (bilateral)
            {
              if(random()%2==0) // selects direction of bidirectional servo 
              {
                servo_B_current_pos = SERVO_B_MAX_POS; 
              }
              else
              {
                servo_B_current_pos = SERVO_B_MIN_POS; 
              }
              servo_B_time_up_count = SERVO_B_TIME_UP;
              servo_B.write(servo_B_current_pos);
            }
            
            analogWrite(LED_WHITE, 255);
            distance_time_count = 0;
          }
        }
      }
      else
      {
        distance_time_count=0;
        
        if(servo_A_time_up_count)
        {
          servo_A_time_up_count--;
          if(!servo_A_time_up_count)
          {
            // return to stand-by position
            servo_A_current_pos = SERVO_A_INIT_POS;
            servo_A.write(servo_A_current_pos);
            delay_time_between_moves = DELAY_TIME_BETWEEN_MOVES;
            analogWrite(LED_WHITE, 0);  
          }
        }

        if(servo_B_time_up_count)
        {
          servo_B_time_up_count--;
          if(!servo_B_time_up_count)
          {
            // return to stand-by position
            servo_B_current_pos = SERVO_B_INIT_POS;
            servo_B.write(servo_B_current_pos);
            delay_time_between_moves = DELAY_TIME_BETWEEN_MOVES;
            analogWrite(LED_WHITE, 0);
          }
        }
      }
            
      break;  
    }

    default:
    {
      state=ST_APAGADO;
      substate=0;
      break;  
    }
  }

  delay(10);
}

bool se_pulso_pedal()
{
  if(digitalRead(BUTTON_PEDAL)==false)
  {
    delay(20);
    if(digitalRead(BUTTON_PEDAL)==false)
    {
      return true;
    }
  }
  return false;
}

void gpio_init()
{
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PEDAL, INPUT_PULLUP);
  
  analogWrite(LED_WHITE, 128);
  analogWrite(LED_RED, 128);
}

uint16_t get_distancia()
{
  long t; //timepo que demora en llegar el eco
  uint16_t dist;
  
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(TRIGGER, LOW);
  
  t = pulseIn(ECHO, HIGH); //obtenemos el ancho del pulso
  dist = t/58;
  if(dist>=SENSOR_MAX_DISTANCE)
  {
    return SENSOR_MAX_DISTANCE;
  }
  return t/58; // devolvemos el tiempo a una distancia en cm
}

void handle_battery_level()
{
  static float batt_voltage;
  static int count=0;

  if(count%10 == 0)
  {
    batt_voltage = (analogRead(BAT_LEVEL) * (23.0/1024.0))*0.3 + batt_voltage*0.7;
  }

  if(count++>200)
  {
    //Serial.print("BAT:");
    //Serial.println(batt_voltage);
    if(batt_voltage <= LOW_BATTERY_LIMIT)
    {
      analogWrite(LED_RED, 255);
    }
    else
    {
      analogWrite(LED_RED, 0);
    }
    count=0;
  }
}
