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

#define PEDAL_TIME_ON_OFF 80
#define PEDAL_TIME_OK     15

#define LED_TIME_TOGGLE   7

#define DISTANCE_TIME_ACTION  2 // cuanto tiempo debe estar "cerca" para que sea detectado como true

#define DELAY_TIME_BETWEEN_MOVES 500

#define LOWER_LIMIT_POTE  50  // in [cm]
#define UPPER_LIMIT_POTE  230 // in [cm]

#define SENSOR_MAX_DISTANCE 300

#define SERVO_A_INIT_POS  0 // in degrees -- unilateral
#define SERVO_A_MIN_POS   0
#define SERVO_A_MAX_POS   90
#define SERVO_A_INCREMENT 2
#define SERVO_A_TIME_UP   125

#define SERVO_B_INIT_POS  90 // in degrees -- bilateral
#define SERVO_B_MIN_POS   0
#define SERVO_B_MAX_POS   180
#define SERVO_B_INCREMENT 2
#define SERVO_B_TIME_UP   125

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
uint16_t led_time_count=0;
uint8_t led_toggle_byte=0;

uint16_t delay_time_between_moves=0;

uint8_t servo_A_move_count=0;
uint8_t servo_A_current_pos=0;
uint16_t servo_A_time_up_count=0;
uint8_t servo_B_move_count=0;
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
  handle_battery_level();
  
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
        
        // set servos to start position
        //servo_A.write(SERVO_A_INIT_POS);
        //servo_B.write(SERVO_B_INIT_POS);
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
          i=0;
          led_time_count=0;
          led_toggle_byte=0;
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
        }
        else
        {
          break;  
        }
      }

      // distance handling
      distance_sense = get_distancia();
      distance_set = map(analogRead(POTE), 0, 1023, LOWER_LIMIT_POTE, UPPER_LIMIT_POTE);
      Serial.print("dist_sense: ");
      Serial.println(distance_sense);
      Serial.print("dist_set: ");
      Serial.println(distance_set);

      // start blinking white to notify distance set
      if(distance_sense <= distance_set)
      {
        led_time_count++;
        if(led_time_count >= LED_TIME_TOGGLE)
        {
          led_time_count=0;
          led_toggle_byte = ~led_toggle_byte;
          analogWrite(LED_WHITE, led_toggle_byte);
        }
      }
      else
      {
        led_time_count=0;
        analogWrite(LED_WHITE, 255);  
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
          while(se_pulso_pedal());   
        }
      }
      else
      {
        // check if distance has been selected
        if(pedal_time_count >= PEDAL_TIME_OK)
        {
          Serial.println("working...");
          distance_selected = distance_set;
          state = ST_WORKING;
          i=0;
          substate=0;  
        }
        pedal_time_count=0;
      }

      break;
    }

    case ST_WORKING:
    {
      if(!substate)
      {
        pedal_time_count=0;
        distance_time_count=0;
        servo_A_move_count=0;
        servo_B_move_count=0;
        servo_A_current_pos = SERVO_A_INIT_POS;
        servo_B_current_pos = SERVO_B_INIT_POS;
        servo_A.write(servo_A_current_pos);
        servo_B.write(servo_B_current_pos);

        if(i<150)
        {
          i++;
          analogWrite(LED_RED, 255);
          break;
        }
        else
        {
          analogWrite(LED_RED, 0);
          i=0;
          randomSeed(analogRead(RAND_PIN_SEED));
          while(se_pulso_pedal()){};
          substate++;
        }
      }

      if(se_pulso_pedal())
      {
        analogWrite(LED_WHITE, 0);
        while(se_pulso_pedal()) {};
        Serial.println("ready...");
        state = ST_READY;
        substate=0;
      }

      if(delay_time_between_moves)
      {
        delay_time_between_moves--;
        break;  
      }

      distance_sense = get_distancia();
      if(distance_sense <= distance_selected && !servo_A_time_up_count && !servo_B_time_up_count)
      {
        distance_time_count++;

        if(distance_time_count >= DISTANCE_TIME_ACTION)
        {
          // aca deberia seleccionar uno de dos
          randomSeed(analogRead(RAND_PIN_SEED));
          if(random()%2==0)
          {
            servo_A_current_pos = SERVO_A_MAX_POS;
            servo_A_time_up_count = SERVO_A_TIME_UP;
            servo_A.write(servo_A_current_pos);
          }
          else
          {
            randomSeed(analogRead(RAND_PIN_SEED));
            if(random()%2==0)
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
            servo_A_move_count = SERVO_A_INIT_POS;
            servo_A_current_pos = SERVO_A_INIT_POS;
            servo_A.write(servo_A_current_pos);
            delay_time_between_moves = DELAY_TIME_BETWEEN_MOVES;
          }
        }

        if(servo_B_time_up_count)
        {
          servo_B_time_up_count--;
          if(!servo_B_time_up_count)
          {
            servo_B_move_count = SERVO_B_INIT_POS;
            servo_B_current_pos = SERVO_B_INIT_POS;
            servo_B.write(servo_B_current_pos);
            delay_time_between_moves = DELAY_TIME_BETWEEN_MOVES;
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
