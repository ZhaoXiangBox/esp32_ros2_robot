#include "motor_driver.h"

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index_ = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0 ; //NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index_ = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setSpeeds(0,0);
}

void loop() {
  while (Serial.available() > 0)
  {
    // Read the next character
    chr = Serial.read();
    // Terminate a command with a CR
    if (chr == 13)  // '\r'
    {
      if (arg == 1)
        argv1[index_] = 0 ;//NULL;
      else if (arg == 2)
        argv2[index_] = 0; //NULL;
      Serial.print(cmd);
      Serial.print(" ");
      Serial.print(atoi(argv1));
      Serial.print(" ");
      Serial.println(atoi(argv2));
      setSpeeds(atoi(argv1),atoi(argv2));
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ')
    {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)
      {
        argv1[index_] = NULL;
        arg = 2;
        index_ = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[index_] = chr;
        index_++;
      }
      else if (arg == 2)
      {
        argv2[index_] = chr;
        index_++;
      }
    }
  }


}
