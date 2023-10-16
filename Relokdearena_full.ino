#include "Arduino.h"
#include "LedControl.h"
#include "Delay.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#include <SPI.h>


#define  MATRIX_A  1
#define  MATRIX_B  0

// Values are 260/330/400
#define ACC_THRESHOLD_LOW 300
#define ACC_THRESHOLD_HIGH 360

// Matrix
#define PIN_DATAIN 11   // DIN
#define PIN_CLK 13      // CLK
#define PIN_LOAD 10     // CS

#define PIN_BUZZER 13

// Esto tiene en cuenta cómo se montan las matrices.
#define ROTATION_OFFSET 90

// in milliseconds
#define DEBOUNCE_THRESHOLD 500

#define DELAY_FRAME 100        //Velocidad de caida de particulas

#define DEBUG_OUTPUT 1

#define MODE_HOURGLASS 0
#define MODE_SETMINUTES 1
#define MODE_SETHOURS 2

byte delayHours = 0;
byte delayMinutes = 1;
int mode = MODE_HOURGLASS;
int gravity;
LedControl lc = LedControl(PIN_DATAIN, PIN_CLK, PIN_LOAD, 2);
NonBlockDelay d;
int resetCounter = 0;
bool alarmWentOff = false;
//MPU6050 
MPU6050 sensor;
//Valores RAW
int ax, ay, az;
int gx, gy, gz;


/**
 * Obtener retraso entre caídas de partículas (en segundos)
 */
long getDelayDrop() {
  // como tenemos exactamente 60 partículas, no tenemos que multiplicar por 60 y luego dividir por el número de partículas nuevamente
  return delayMinutes + delayHours * 60;
}


#if DEBUG_OUTPUT
void printmatrix() {
  Serial.println(" 0123-4567 ");
  for (int y = 0; y<8; y++) {
    if (y == 4) {
      Serial.println("|----|----|");
    }
    Serial.print(y);
    for (int x = 0; x<8; x++) {
      if (x == 4) {
        Serial.print("|");
      }
      Serial.print(lc.getXY(0,x,y) ? "X" :" ");
    }
    Serial.println("|");
  }
  Serial.println("-----------");
}
#endif



coord getDown(int x, int y) {
  coord xy;
  xy.x = x-1;
  xy.y = y+1;
  return xy;
}
coord getLeft(int x, int y) {
  coord xy;
  xy.x = x-1;
  xy.y = y;
  return xy;
}
coord getRight(int x, int y) {
  coord xy;
  xy.x = x;
  xy.y = y+1;
  return xy;
}


bool canGoLeft(int addr, int x, int y) {
  if (x == 0) return false; // not available
  return !lc.getXY(addr, getLeft(x, y));              // you can go there if this is empty
}
bool canGoRight(int addr, int x, int y) {
  if (y == 7) return false; // not available
  return !lc.getXY(addr, getRight(x, y));             // you can go there if this is empty
}
bool canGoDown(int addr, int x, int y) {
  if (y == 7) return false; // not available
  if (x == 0) return false; // not available
  if (!canGoLeft(addr, x, y)) return false;
  if (!canGoRight(addr, x, y)) return false;
  return !lc.getXY(addr, getDown(x, y));              // you can go there if this is empty
}



void goDown(int addr, int x, int y) {
  lc.setXY(addr, x, y, false);
  lc.setXY(addr, getDown(x,y), true);
}
void goLeft(int addr, int x, int y) {
  lc.setXY(addr, x, y, false);
  lc.setXY(addr, getLeft(x,y), true);
}
void goRight(int addr, int x, int y) {
  lc.setXY(addr, x, y, false);
  lc.setXY(addr, getRight(x,y), true);
}


int countParticles(int addr) {
  int c = 0;
  for (byte y=0; y<=8; y++) {
    for (byte x=0; x<=8; x++) {
      if (lc.getXY(addr, x, y)) {
        c++;
      }
    }
  }
  return c;
}


bool moveParticle(int addr, int x, int y) {
  if (!lc.getXY(addr,x,y)) {
    return false;
  }

  bool can_GoLeft = canGoLeft(addr, x, y);
  bool can_GoRight = canGoRight(addr, x, y);

  if (!can_GoLeft && !can_GoRight) {
    return false; // we're stuck
  }

  bool can_GoDown = canGoDown(addr, x, y);

  if (can_GoDown) {
    goDown(addr, x, y);
  } else if (can_GoLeft&& !can_GoRight) {
    goLeft(addr, x, y);
  } else if (can_GoRight && !can_GoLeft) {
    goRight(addr, x, y);
  } else if (random(2) == 1) {                  // Podemos ir a izquierda y derecha, pero no abajo
    goLeft(addr, x, y);
  } else {
    goRight(addr, x, y);
  }
  return true;
}


void fill(int addr, int maxcount) {
  int n = 8;
  byte x,y;
  int count = 0;
  for (byte slice = 0; slice < 2*n-1; ++slice) {
    byte z = slice<n ? 0 : slice-n + 1;
    for (byte j = z; j <= slice-z; ++j) {
      y = 7-j;
      x = (slice-j);
      lc.setXY(addr, x, y, (++count <= maxcount));
    }
  }
}

/*
void setupAcelerometro(){
  Serial.begin(9600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  Con esta función, se puede inicializar el acelerómetro MP6030
}
*/

// Detectar la orientación usando el acelerómetro

int getGravity() {
  Serial.begin(9600);    //Iniciando puerto serial en la baud 9600
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
 


  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  Serial.print("a[x y z] g[x y z]:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz); //Estos valores los imprime en consola, para saber si en realidad nuestro acelerometro sirve

  delay(100);     

  if (ax >= 20 & az <= 5  )  { return 90;   }                 //Arena hacia la derecha 
  if (ax >= -10 & az > 0 ) { return 180; }                     //Funciona como reloj de arena arriba - abajo
  if (ax >= -40000 & az <= -15000  )  { return 0;   }                //Funciona como reloj de arena abajo - arriba 
  if (ax <= -20 & az >= -5) { return 270; }                   //Arena hacia la izquierda  
    //Basandonos en los valores en consola, logramos modificar los valores originales para adaptarlos a nuestro Hardware
        
}


int getTopMatrix() {
  return (getGravity() == 90) ? MATRIX_A : MATRIX_B;
}
int getBottomMatrix() {
  return (getGravity() != 90) ? MATRIX_A : MATRIX_B;
}



void resetTime() {
  for (byte i=0; i<2; i++) {
    lc.clearDisplay(i);
  }
  fill(getTopMatrix(), 60);           // cantidad de particulas en la Matriz, si se define un intervalo de minutos u horas el valor debe ser 60
  d.Delay(getDelayDrop() * 1000);
}



/**
 * Atraviese la matriz y compruebe si es necesario mover las partículas
 */
bool updateMatrix() {
  int n = 8;
  bool somethingMoved = false;
  byte x,y;
  bool direction;
  for (byte slice = 0; slice < 2*n-1; ++slice) {
    direction = (random(2) == 1); // randomize if we scan from left to right or from right to left, so the grain doesn't always fall the same direction
    byte z = slice<n ? 0 : slice-n + 1;
    for (byte j = z; j <= slice-z; ++j) {
      y = direction ? (7-j) : (7-(slice-j));
      x = direction ? (slice-j) : j;
      // for (byte d=0; d<2; d++) { lc.invertXY(0, x, y); delay(50); }
      if (moveParticle(MATRIX_B, x, y)) {
        somethingMoved = true;
      };
      if (moveParticle(MATRIX_A, x, y)) {
        somethingMoved = true;
      }
    }
  }
  return somethingMoved;
}



/**
 * Deja que una partícula pase de una matriz a otra
 */
boolean dropParticle() {
  if (d.Timeout()) {
    d.Delay(getDelayDrop() * 1000);
    if (gravity == 0 || gravity == 180) {
      if ((lc.getRawXY(MATRIX_A, 0, 0) && !lc.getRawXY(MATRIX_B, 7, 7)) ||
          (!lc.getRawXY(MATRIX_A, 0, 0) && lc.getRawXY(MATRIX_B, 7, 7))
      ) {
        // for (byte d=0; d<8; d++) { lc.invertXY(0, 0, 7); delay(50); }
        lc.invertRawXY(MATRIX_A, 0, 0);
        lc.invertRawXY(MATRIX_B, 7, 7);
        tone(PIN_BUZZER, 440, 10);
        return true;
      }
    }
  }
  return false;
}



void alarm() {
  for (int i=0; i<5; i++) {
    tone(PIN_BUZZER, 440, 200);
    delay(1000);
  }
}

void resetCheck() {
  int z = analogRead(A3);
  if (z > ACC_THRESHOLD_HIGH || z < ACC_THRESHOLD_LOW) {
    resetCounter++;
    Serial.println(resetCounter);
  } else {
    resetCounter = 0;
  }
  if (resetCounter > 20) {
    Serial.println("RESET!");
    resetTime();
    resetCounter = 0;
  }
}

void displayLetter(char letter, int matrix) {
  // Serial.print("Letter: ");
  // Serial.println(letter);
  lc.clearDisplay(matrix);
  lc.setXY(matrix, 1,4, true);
  lc.setXY(matrix, 2,3, true);
  lc.setXY(matrix, 3,2, true);
  lc.setXY(matrix, 4,1, true);

  lc.setXY(matrix, 3,6, true);
  lc.setXY(matrix, 4,5, true);
  lc.setXY(matrix, 5,4, true);
  lc.setXY(matrix, 6,3, true);

  if (letter == 'M') {
    lc.setXY(matrix, 4,2, true);
    lc.setXY(matrix, 4,3, true);
    lc.setXY(matrix, 5,3, true);
  }
  if (letter == 'H') {
    lc.setXY(matrix, 3,3, true);
    lc.setXY(matrix, 4,4, true);
  }
}



void renderSetMinutes() {
  fill(getTopMatrix(), delayMinutes);
  displayLetter('M', getBottomMatrix());
}
void renderSetHours() {
  fill(getTopMatrix(), delayHours);
  displayLetter('H', getBottomMatrix());
}




void knobClockwise() {
  Serial.println("Clockwise");
  if (mode == MODE_SETHOURS) {
    delayHours = constrain(delayHours+1, 0, 64);
    renderSetHours();
  } else if(mode == MODE_SETMINUTES) {
    delayMinutes = constrain(delayMinutes+1, 0, 64);
    renderSetMinutes();
  }
  Serial.print("Delay: ");
  Serial.println(getDelayDrop());
}
void knobCounterClockwise() {
  Serial.println("Counterclockwise");
  if (mode == MODE_SETHOURS) {
    delayHours = constrain(delayHours-1, 0, 64);
    renderSetHours();
  } else if (mode == MODE_SETMINUTES) {
    delayMinutes = constrain(delayMinutes-1, 0, 64);
    renderSetMinutes();
  }
  Serial.print("Delay: ");
  Serial.println(getDelayDrop());
}



volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
long lastValue = 0;
void updateEncoder() {
  
/*
  int MSB = digitalRead(PIN_ENC_1); //MSB = most significant bit
  int LSB = digitalRead(PIN_ENC_2); //LSB = least significant bit
*/

  int MSB = 0; //MSB = most significant bit      //Agregado
  int LSB = 0; //LSB = least significant bit     //Agregado
 

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;

  // Serial.print("Value: ");
  // Serial.println(encoderValue);
  if ((encoderValue % 4) == 0) {
    int value = encoderValue / 4;
    if (value > lastValue) knobClockwise();
    if (value < lastValue) knobCounterClockwise();
    lastValue = value;
  }
  lastEncoded = encoded; //store this value for next time
}



/**
 * Button callback (incl. software debouncer)
 * Esto cambia entre los modos (normal, establecer minutos, establecer horas)
 */
volatile unsigned long lastButtonPushMillis;
void buttonPush() {
  
  if((long)(millis() - lastButtonPushMillis) >= DEBOUNCE_THRESHOLD) {
    mode = (mode+1) % 3;
    Serial.print("Modo cambiado a: ");
    Serial.println(mode);
    lastButtonPushMillis = millis();

    if (mode == MODE_SETMINUTES) {
      lc.backup(); // we only need to back when switching from MODE_HOURGLASS->MODE_SETMINUTES
      renderSetMinutes();
    }
    
    if (mode == MODE_SETHOURS) {
      renderSetHours();
    }
    
    if (mode == MODE_HOURGLASS) {
      lc.clearDisplay(0);
      lc.clearDisplay(1);
      lc.restore();
      resetTime();
    }
    
  }
}


void setup() {
  
  Serial.begin(9600);

   randomSeed(analogRead(A0));

  // init displays
  for (byte i=0; i<2; i++) {
    lc.shutdown(i,false);
    lc.setIntensity(i,0);     //Controla la intensidad de brillo en las particulas
  }
  resetTime();
}



/**
 * Main loop
 */
void loop() {
  delay(DELAY_FRAME);

  // actualizar la configuración de rotación del controlador. Para el resto del código pretendemos que "abajo" sigue siendo 0,0 y "arriba" es 7,7
  gravity = getGravity();
  lc.setRotation((ROTATION_OFFSET + gravity) % 360);
  

  // manejar modos especiales
  if (mode == MODE_SETMINUTES) {
    renderSetMinutes(); return;
  } else if (mode == MODE_SETHOURS) {
    renderSetHours(); return;
  }

  // resetCheck();                   // el reinicio ahora ocurre al presionar un botón
  bool moved = updateMatrix();
  bool dropped = dropParticle();

  // alarma cuando todo está en la parte inferior
  if (!moved && !dropped && !alarmWentOff && (countParticles(getTopMatrix()) == 0)) {
      //alarmWentOff = true;
     //alarm();
    //resetTime();            //Agregado
  }
  // reset alarm flag next time a particle was dropped
  if (dropped) {
    alarmWentOff = false;
  }
}
